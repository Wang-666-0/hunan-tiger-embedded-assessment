/* ES5, Qt 5.14 QML script import and Node vm compatible. No shared mutable state.
 * Transport passes decoded text; this engine never opens or writes a serial port.
 */
function createEngine(options) {
    options = options || {};
    function limit(name, fallback, min, max) {
        var n = Number(options[name]);
        return isFinite(n) ? Math.max(min, Math.min(max, Math.floor(n))) : fallback;
    }
    var cfg = {
        maxReports: limit("maxReports", 100, 1, 100),
        maxEvents: limit("maxEvents", 1000, 1, 5000),
        maxLinesPerReport: limit("maxLinesPerReport", 256, 1, 2000),
        maxCharsPerReport: limit("maxCharsPerReport", 65536, 256, 262144),
        maxEventChars: limit("maxEventChars", 262144, 256, 1048576),
        maxTotalReportChars: limit("maxTotalReportChars", 2097152, 512, 4194304),
        maxLineLength: limit("maxLineLength", 8192, 64, 32768),
        timeoutMs: limit("timeoutMs", 4000, 100, 60000),
        stepTimeoutMs: limit("stepTimeoutMs", 8000, 100, 60000),
        staleMs: limit("staleMs", 30000, 1000, 300000)
    };
    var e = {
        reports: [], events: [], warnings: [], pendingId: null,
        stats: { receivedLines: 0, telemetryLines: 0, otherLines: 0,
            droppedEvents: 0, droppedReports: 0, droppedReportLines: 0,
            truncatedLines: 0, gaps: 0 },
        limits: cfg, revision: 0
    };
    var groups = ["gimbal", "pitch", "feeder"];
    var seq = 0, nextId = 1, partial = "", partialAt = 0, partialOwner = null;
    var partialActive = false, overflow = 0, eventChars = 0, reportChars = 0;
    var stale = [], latestSteps = { P: null, Y: null };
    var numeric = /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/;
    function nowValue(n) { n = Number(n); return isFinite(n) && Math.abs(n) <= 8640000000000000 ? n : new Date().getTime(); }
    function changed() { e.revision += 1; }
    function result(ok, error, extra) {
        var r = { ok: ok, error: error || "", changed: !!extra };
        if (extra && typeof extra === "object") {
            var k; for (k in extra) { if (extra.hasOwnProperty(k)) { r[k] = extra[k]; } }
            r.changed = true;
        }
        return r;
    }
    function warn(message) {
        if (e.warnings.indexOf(message) < 0) {
            e.warnings.push(message);
            if (e.warnings.length > 20) { e.warnings.shift(); }
        }
        changed();
    }
    function warning(r, message) {
        if (r.warnings.indexOf(message) < 0) {
            r.warnings.push(message);
            if (r.warnings.length > 20) { r.warnings.shift(); }
        }
        changed();
    }
    function pending() {
        var i; for (i = e.reports.length - 1; i >= 0; i -= 1) {
            if (e.reports[i].id === e.pendingId) { return e.reports[i]; }
        }
        return null;
    }
    function level(text) {
        if (/\|(?:ERROR|UART_FAULT)(?:\||$)/.test(text)) { return "error"; }
        if (/\|(?:ABORTED|TIMEOUT)(?:\||$)/.test(text) ||
                /\|(?:FAULT|P_FAULT)\|(?:0[xX]0*[1-9a-fA-F][0-9a-fA-F]*|[1-9]\d*)(?:\||$)/.test(text)) { return "error"; }
        if (/^[PY]\|GUARD\|/.test(text)) { return "warning"; }
        return "info";
    }
    function addEvent(line) {
        e.events.push(line); eventChars += line.text.length;
        while (e.events.length > cfg.maxEvents || eventChars > cfg.maxEventChars) {
            eventChars -= e.events.shift().text.length; e.stats.droppedEvents += 1;
        }
        if (e.stats.droppedEvents) { warn("主动消息缓存已达上限，较早消息已移除；导出仅包含保留内容。"); }
        changed();
    }
    function evictReport() {
        var gone = e.reports.shift();
        if (gone) { reportChars -= gone.charCount; e.stats.droppedReports += 1; }
    }
    function trimReports() {
        while (e.reports.length > cfg.maxReports) { evictReport(); }
        while (reportChars > cfg.maxTotalReportChars && e.reports.length > 1) { evictReport(); }
        if (e.stats.droppedReports) { warn("历史报告已按数量或总容量上限移除较早记录。"); }
    }
    function addLine(r, line) {
        var stored = { text: line.text, at: line.at, seq: line.seq,
            uncertain: !!line.uncertain, level: line.level, truncated: !!line.truncated };
        r.lines.push(stored); r.charCount += stored.text.length; reportChars += stored.text.length;
        r.updatedAt = Math.max(r.updatedAt, stored.at);
        if (stored.uncertain) { r.uncertain = true; }
        while (r.lines.length > cfg.maxLinesPerReport || r.charCount > Math.min(cfg.maxCharsPerReport, cfg.maxTotalReportChars)) {
            var gone = r.lines.shift();
            r.charCount -= gone.text.length; reportChars -= gone.text.length;
            r.lineDrops += 1; e.stats.droppedReportLines += 1;
        }
        if (r.lineDrops) { warning(r, "报告缓存已达上限，较早正文已移除。"); }
        trimReports(); changed();
    }
    function markStale(r, at) {
        stale.push({ key: r._match.key, until: at + cfg.staleMs });
        while (stale.length > 100) { stale.shift(); }
    }
    function finish(r, status, at) {
        r.status = status; r.updatedAt = Math.max(r.updatedAt, at);
        if (e.pendingId === r.id) { e.pendingId = null; }
        changed();
    }
    function isTelemetry(text) {
        var colon = text.indexOf(":"), group, values, i, s;
        if (colon < 1) { return false; }
        group = text.substring(0, colon);
        if (groups.indexOf(group) < 0) { return false; }
        values = text.substring(colon + 1).split(",");
        if (!values.length) { return false; }
        for (i = 0; i < values.length; i += 1) {
            s = values[i].replace(/^\s+|\s+$/g, "");
            if (!numeric.test(s) || !isFinite(Number(s))) { return false; }
        }
        return true;
    }
    function descriptor(command) {
        var words = command.split(" "), axis = words[0], action = words[1] || "", d;
        d = { key: command, kind: "unknown", axis: "", action: action, timeout: cfg.timeoutMs };
        if (axis === "PITCH" || axis === "YAW") {
            d.axis = axis === "PITCH" ? "P" : "Y"; d.kind = "axis";
            if (!action) { d.kind = "read"; d.reply = "STATE"; }
            else if (action === "VIEW") { d.kind = "view"; d.value = axis; }
            else if (action === "RESULT") { d.kind = "ambiguous"; d.reply = "STEP"; }
            else if (action === "GUARD") { d.kind = "ambiguous"; d.reply = "GUARD"; }
            else if (action === "CHECK") { d.kind = "check"; }
            else if (action === "READY") { d.kind = "ready"; }
            else if (action === "STEP" && words[2] !== "CHECK") {
                d.kind = "step"; d.timeout = cfg.stepTimeoutMs;
                d.target = words.length === 2 ? 30 : Number(words[2]);
                d.beforeStep = latestSteps[d.axis]; d.accepted = false;
            } else if (action === "STEP" && words[2] === "CHECK") { d.kind = "read"; d.reply = "STEP_CHECK"; }
            else if (action === "STATUS" || action === "INFO") { d.kind = "read"; d.reply = "STATUS"; }
            else if (action === "HELP" || action === "?") { d.kind = "read"; d.reply = "HELP"; }
            else if (words.length === 2 && /^(PID|FF|BRAKE|SHAPE|SHAPEV|BUS)$/.test(action)) { d.kind = "read"; d.reply = action; }
            return d;
        }
        if (command === "REMOTE RX") { d.kind = "ambiguous"; d.axis = "R"; d.reply = "RX"; }
        else if (command === "REMOTE INFO") { d.kind = "read"; d.axis = "R"; d.reply = "INFO"; }
        else if (command === "HELP") { d.kind = "read"; d.axis = "C"; d.reply = "HELP"; }
        else if (/^TELEM (ON|OFF)$/.test(command)) { d.kind = "telem"; d.axis = "C"; d.value = words[1]; }
        else if (/^(PITCH|YAW|FEEDER) VIEW$/.test(command) || /^TELEM (PITCH|YAW|FEEDER)$/.test(command)) {
            d.kind = "view"; d.value = words[0] === "TELEM" ? words[1] : words[0];
        } else if (command === "STATUS") { d.kind = "diagStatus"; d.axis = "D"; }
        else if (/^READ DM \d+$/.test(command)) { d.kind = "diag"; d.axis = "D"; d.device = "DM"; d.field = String(Number(words[2])); d.began = false; }
        else if (/^READ VESC \d+ \d+$/.test(command)) { d.kind = "diag"; d.axis = "D"; d.device = "VESC"; d.node = String(Number(words[2])); d.field = String(Number(words[3])); d.began = false; }
        else if (command === "DIAG ON") { d.kind = "read"; d.axis = "D"; d.reply = "HELLO"; }
        return d;
    }
    function tokenValue(tokens, key) {
        var i; for (i = 2; i + 1 < tokens.length; i += 1) { if (tokens[i] === key) { return tokens[i + 1]; } }
        return null;
    }
    function startsAction(value, action) {
        return value === action || value.indexOf(action + "_") === 0;
    }
    function axisError(d, t) {
        var reason = t[2] || "", action = d.action;
        if (startsAction(reason, action)) { return true; }
        if ((action === "POS" || action === "VEL" || /^(PKP|PKI|PKD|VKP|VKI|VKD|PID)$/.test(action)) &&
                /^(PID_RANGE_OR_FORMAT|PARAMETER_OR_STATE|USE_YAW_POS_OR_VEL)$/.test(reason)) { return true; }
        if ((action === "KG" || action === "FF") && startsAction(reason, "KG")) { return true; }
        if ((action === "GOFF" || action === "GHORIZ") && startsAction(reason, "GOFF")) { return true; }
        if (action === "ZERO" && reason === "STOP_AND_STATIONARY_REQUIRED") { return true; }
        if (action === "VMAX" && reason === "OUTPUT_LIMITS_DISABLED") { return true; }
        if (reason === "UNKNOWN_COMMAND") { return true; }
        return false;
    }
    // Match only command-specific messages; interleaved state or unrelated axis errors remain events.
    function match(r, t) {
        var d = r._match, a = t[0], kind = t[1], v = t[2], query, state, target, id, same;
        if (d.kind === "unknown") { return null; }
        if (d.kind === "view") {
            if (kind === "VIEW" && v === d.value && /^[PYF]$/.test(a)) { return { terminal: true }; }
            return null;
        }
        if (a !== d.axis) { return null; }
        if (kind === "ERROR" && d.axis !== "D") {
            if (axisError(d, t)) { return { terminal: true, error: true }; }
            return null;
        }
        if (d.kind === "ambiguous") {
            return kind === d.reply ? { uncertain: true, candidate: true } : null;
        }
        if (d.kind === "read") {
            if (kind !== d.reply) { return null; }
            if (d.axis === "P" && d.reply === "STATUS" && tokenValue(t, "QUERY") !== "SNAPSHOT_ONLY") { return null; }
            return { terminal: true };
        }
        if (d.kind === "check") {
            if (kind === "BUSY" && v === "CHECK") {
                if (d.axis === "P" && tokenValue(t, "QUERY") === "CC") { return { terminal: false }; }
                return { terminal: true, error: true };
            }
            if (kind !== "CHECK") { return null; }
            if (d.axis === "Y") { return { terminal: true }; }
            query = tokenValue(t, "QUERY");
            return query === "NEW_FEEDBACK" || query === "TIMEOUT" ? { terminal: true, error: query === "TIMEOUT" } : null;
        }
        if (d.kind === "ready") {
            if (kind === "BUSY" && v === "READY") {
                return d.axis === "Y" || tokenValue(t, "QUERY") === "CC" ? { terminal: false } : { terminal: true, error: true };
            }
            if (kind === "OK" && (v === "READY" || v === "ALREADY_READY")) { return { terminal: true }; }
            return null;
        }
        if (d.kind === "step") {
            if (kind === "OK" && v === "STEP_STARTED") {
                target = Number(tokenValue(t, "TARGET"));
                if (tokenValue(t, "TARGET") === null || !isFinite(target) || !isFinite(d.target) || Math.abs(target - d.target) > 0.000001) { return null; }
                d.accepted = true; return { terminal: false };
            }
            if (kind !== "STEP" || !d.accepted) { return null; }
            state = tokenValue(t, "STATE"); target = Number(tokenValue(t, "TARGET")); id = Number(tokenValue(t, "ID"));
            if (tokenValue(t, "TARGET") === null || !/^\d+$/.test(tokenValue(t, "ID") || "") || !isFinite(target) || !isFinite(id) || Math.abs(target - d.target) > 0.000001 ||
                    (d.beforeStep !== null && id <= d.beforeStep)) { return null; }
            if (state === "DONE" || state === "ABORTED") {
                // ACK and target support correlation, but the current firmware has no request ID.
                return { terminal: true, error: state === "ABORTED", uncertain: true };
            }
            return state === "RUNNING" ? { terminal: false, uncertain: true } : null;
        }
        if (d.kind === "telem") { return kind === "OK" && v === "TELEM" && t[3] === d.value ? { terminal: true } : null; }
        if (d.kind === "diagStatus") {
            if (kind === "STATUS" || kind === "CAN") { return { terminal: false }; }
            return kind === "DONE" && v === "STATUS" ? { terminal: true } : null;
        }
        if (d.kind === "diag") {
            if (kind === "ERROR") {
                // Unkeyed diagnostic errors are ambiguous before a matching BEGIN.
                return { terminal: true, error: true, uncertain: !d.began };
            }
            if (kind === "BEGIN" || kind === "DONE") {
                same = v === d.device && (d.device === "DM" ? t[3] === d.field : t[3] === d.node && t[4] === d.field);
                if (!same) { return null; }
                if (kind === "BEGIN") { d.began = true; return { terminal: false }; }
                return d.began ? { terminal: true } : { uncertain: true, candidate: true };
            }
            same = kind === d.device && (d.device === "DM" ? v === d.field : v === d.node && t[3] === d.field);
            if (same && d.began) { return { terminal: false }; }
        }
        if (d.kind === "axis") {
            if (kind === "BUSY") { return { terminal: true, error: true, uncertain: v !== d.action }; }
            if (kind !== "OK") { return null; }
            same = startsAction(v || "", d.action);
            if (d.action === "OFF") { same = v === "OFF" || v === "OFF_REQUESTED"; }
            else if (d.action === "ZERO") { same = v === "SOFTWARE_ZERO"; }
            else if (d.action === "KG" || d.action === "FF") { same = v === "KG_SET" || (d.axis === "Y" && v === "FF"); }
            else if (d.action === "GOFF" || d.action === "GHORIZ") { same = v === "HORIZONTAL_SET"; }
            else if (/^(PKP|PKI|PKD|VKP|VKI|VKD)$/.test(d.action)) { same = v === "PARAMETER"; }
            return same ? { terminal: true } : null;
        }
        return null;
    }
    function consume(text, at, owner, truncated) {
        e.stats.receivedLines += 1;
        if (!truncated && isTelemetry(text)) { e.stats.telemetryLines += 1; return; }
        e.stats.otherLines += 1; seq += 1;
        var line = { text: text, at: at, seq: seq, uncertain: false, level: level(text), truncated: !!truncated };
        var r = pending(), t = text.split("|"), m = null, stepId;
        if (r && owner === r.id && !truncated) { m = match(r, t); }
        if (/^[PY]$/.test(t[0]) && t[1] === "STEP") {
            stepId = tokenValue(t, "ID");
            if (stepId !== null && /^\d+$/.test(stepId) && isFinite(Number(stepId)) && Number(stepId) <= 9007199254740000) {
                latestSteps[t[0]] = Number(stepId);
            }
        }
        if (m) {
            line.uncertain = !!(m.uncertain || r.ambiguousFromEarlier);
            addLine(r, line);
            if (line.uncertain) {
                warning(r, "归属待确认：现有固件未提供请求编号，该消息可能由主动上报或此前命令产生。");
            }
            if (m.candidate) {
                r.candidateCount += 1;
                addEvent(line); // Preserve uncertain candidates in the independent event timeline too.
            }
            if (m.terminal) {
                finish(r, r.uncertain ? "uncertain" : (m.error ? "error" : "complete"), at);
                r.deviceError = !!m.error;
            }
        } else { addEvent(line); }
    }
    e.start = function (command, at) {
        at = nowValue(at);
        if (e.pendingId !== null) { return result(false, "请先等待当前报告，或点击结束等待。"); }
        if (typeof command !== "string" || /[\r\n\x00-\x1f\x7f]/.test(command)) { return result(false, "命令必须是单行文本，不能含控制字符。"); }
        command = command.replace(/^ +| +$/g, "");
        if (!command || command.length > 63) { return result(false, "命令长度需为 1 至 63 个字符。"); }
        var normalized = command.replace(/ +/g, " ").toUpperCase(), d = descriptor(normalized), i, old = false;
        stale = stale.filter(function (item) { return item.until >= at; });
        for (i = 0; i < stale.length; i += 1) { if (stale[i].key === d.key) { old = true; } }
        var r = { id: nextId, command: command, sentAt: at, updatedAt: at, status: "waiting",
            lines: [], warnings: [], charCount: 0, lineDrops: 0, candidateCount: 0,
            uncertain: false, ambiguousFromEarlier: old, deviceError: false, _match: d,
            timeoutAt: at + d.timeout };
        nextId += 1; e.reports.push(r); e.pendingId = r.id; trimReports();
        if (old) { warning(r, "同一命令此前未确认完成；接下来的匹配回复将标记归属待确认。"); }
        if (d.kind === "unknown") { warning(r, "此命令没有内置报告匹配规则；非遥测文本将保留在主动消息，需手动结束等待。"); }
        changed(); return result(true, "", { id: r.id });
    };
    e.feed = function (text, at) {
        at = nowValue(at); var before = e.revision, i, c, value;
        if (typeof text !== "string") { return result(false, "接收数据必须是已解码的字符串。"); }
        // Tick before accepting new bytes: overdue replies must not silently revive an expired trial.
        e.tick(at);
        for (i = 0; i < text.length; i += 1) {
            c = text.charAt(i);
            if (!partialActive) { partialActive = true; partialAt = at; partialOwner = e.pendingId; }
            if (c === "\n") {
                value = partial;
                if (value.charAt(value.length - 1) === "\r") { value = value.substring(0, value.length - 1); }
                if (overflow) {
                    e.stats.truncatedLines += 1;
                    warn("接收行超过容量上限，部分文本被截断；截断行不会用于报告完成判定。");
                    var active = pending();
                    if (active && active.id === partialOwner) {
                        warning(active, "等待期间有接收行被截断；本次报告完整性与归属无法确认。");
                        active.uncertain = true; active.ambiguousFromEarlier = true;
                    }
                    value += " [接收行截断，丢失 " + overflow + " 字符]";
                }
                consume(value, partialAt, partialOwner, overflow > 0);
                partial = ""; partialActive = false; overflow = 0; partialOwner = null;
            } else if (partial.length < cfg.maxLineLength) { partial += c; }
            else { overflow += 1; }
        }
        return { ok: true, error: "", changed: e.revision !== before };
    };
    e.tick = function (at) {
        at = nowValue(at); var r = pending();
        if (r && at >= r.timeoutAt) {
            markStale(r, at);
            warning(r, r.candidateCount ? "等待结束：已收到候选报告，但无法确认其命令归属。" : "等待超时：未收到可确认的完整报告；未自动重发命令。");
            finish(r, r.candidateCount ? "uncertain" : "timeout", at);
            return result(true, "", true);
        }
        return result(true, "", false);
    };
    e.endWait = function (at) {
        at = nowValue(at); var r = pending();
        if (!r) { return result(false, "当前没有等待中的命令。"); }
        warning(r, "用户结束面板等待；没有向单片机发送停止指令。");
        markStale(r, at); finish(r, r.candidateCount ? "uncertain" : "ended", at);
        return result(true, "", true);
    };
    e.failSend = function (reason, at) {
        at = nowValue(at); var r = pending();
        if (!r) { return result(false, "当前没有等待中的命令。"); }
        warning(r, "发送失败或发送状态不明确：" + String(reason || "未知原因").substring(0, 256));
        markStale(r, at); finish(r, "error", at); return result(true, "", true);
    };
    e.noteGap = function (reason, at) {
        at = nowValue(at); e.stats.gaps += 1;
        var message = "接收缺口：" + String(reason || "传输层报告丢失").substring(0, 256), r = pending();
        warn(message);
        if (partialActive) {
            consume(partial + " [接收缺口，未完成行]", partialAt, null, true);
            partial = ""; partialActive = false; overflow = 0; partialOwner = null;
        }
        if (r) { warning(r, message + "；报告完整性无法确认。"); r.uncertain = true; r.ambiguousFromEarlier = true; }
        seq += 1; addEvent({ text: "[" + message + "]", at: at, seq: seq, uncertain: true, level: "warning" });
        return result(true, "", true);
    };
    e.setRules = function (values) {
        var next = [], i, value;
        if (!Array.isArray(values) || values.length > 32) { return result(false, "规则需为最多 32 个遥测组名组成的数组。"); }
        for (i = 0; i < values.length; i += 1) {
            if (typeof values[i] !== "string") { return result(false, "遥测组名必须是文本。"); }
            value = values[i].replace(/^\s+|\s+$/g, "");
            if (!/^[A-Za-z][A-Za-z0-9_]{0,31}$/.test(value)) { return result(false, "遥测组名只允许字母、数字和下划线，且必须以字母开头。"); }
            if (next.indexOf(value) < 0) { next.push(value); }
        }
        groups = next; changed(); return result(true, "", true);
    };
    e.getRules = function () { return groups.slice(0); };
    e.getReport = function (id) {
        var i; for (i = 0; i < e.reports.length; i += 1) { if (e.reports[i].id === id) { return e.reports[i]; } }
        return null;
    };
    e.serialize = function () {
        return JSON.stringify({ version: 1, reports: e.reports, events: e.events,
            warnings: e.warnings, stats: e.stats, nextId: nextId, sequence: seq, groups: groups });
    };
    e.restore = function (data) {
        var input, i, j, raw, r, item, seen = {}, maxSeq = 0, maxId = 0;
        function validTime(value) { return typeof value === "number" && isFinite(value) && Math.abs(value) <= 8640000000000000; }
        function safeCount(value) { return typeof value === "number" && isFinite(value) && value >= 0 && value <= 9007199254740000 ? Math.floor(value) : 0; }
        function restoreLine(v) {
            if (!v || typeof v.text !== "string" || !validTime(v.at) || typeof v.seq !== "number" || v.seq < 0 || v.seq > 9007199254740000 || !isFinite(v.seq)) { return null; }
            if (v.text.length > cfg.maxLineLength + 100) {
                e.stats.truncatedLines += 1; warn("部分历史正文超过容量上限，已截断并标记。");
            }
            return { text: v.text.substring(0, cfg.maxLineLength + 100), at: v.at, seq: Math.floor(v.seq),
                uncertain: !!v.uncertain, level: /^(info|warning|error)$/.test(v.level) ? v.level : level(v.text), truncated: !!v.truncated || v.text.length > cfg.maxLineLength + 100 };
        }
        if (typeof data === "string" && data.length > 33554432) { return result(false, "历史文件超过 32 MB 字符容量限制，当前数据已保留。"); }
        try { input = typeof data === "string" ? JSON.parse(data) : data; }
        catch (err) { return result(false, "历史文件无法解析，当前数据已保留。"); }
        if (!input || input.version !== 1 || !Array.isArray(input.reports) || !Array.isArray(input.events) ||
                input.reports.length > 10000 || input.events.length > 50000) { return result(false, "历史文件格式或容量无效，当前数据已保留。"); }
        // Validate the envelope before replacing current state; sanitize each persisted item.
        e.reports = []; e.events = []; e.warnings = []; e.pendingId = null;
        reportChars = 0; eventChars = 0; seq = 0; nextId = 1;
        partial = ""; partialActive = false; partialOwner = null; overflow = 0; stale = [];
        latestSteps = { P: null, Y: null };
        var k; for (k in e.stats) { if (e.stats.hasOwnProperty(k)) { e.stats[k] = 0; } }
        if (input.stats && typeof input.stats === "object") {
            for (k in e.stats) { if (e.stats.hasOwnProperty(k)) { e.stats[k] = safeCount(input.stats[k]); } }
        }
        e.stats.droppedReports += Math.max(0, input.reports.length - cfg.maxReports);
        e.stats.droppedEvents += Math.max(0, input.events.length - cfg.maxEvents);
        if (Array.isArray(input.groups)) { e.setRules(input.groups); }
        for (i = Math.max(0, input.reports.length - cfg.maxReports); i < input.reports.length; i += 1) {
            raw = input.reports[i];
            if (!raw || typeof raw.id !== "number" || !isFinite(raw.id) || raw.id < 1 || raw.id > 1000000000000 || raw.id !== Math.floor(raw.id) ||
                    seen[raw.id] || typeof raw.command !== "string" || raw.command.length > 63 || /[\r\n\x00-\x1f\x7f]/.test(raw.command) ||
                    !validTime(raw.sentAt) || !Array.isArray(raw.lines)) { warn("部分历史记录格式损坏，已跳过。"); continue; }
            seen[raw.id] = true;
            r = { id: raw.id, command: raw.command, sentAt: raw.sentAt, updatedAt: validTime(raw.updatedAt) ? raw.updatedAt : raw.sentAt,
                status: /^(complete|uncertain|error|timeout|ended|interrupted)$/.test(raw.status) ? raw.status : "interrupted",
                lines: [], warnings: [], charCount: 0, lineDrops: safeCount(raw.lineDrops),
                candidateCount: 0, uncertain: !!raw.uncertain, ambiguousFromEarlier: false,
                deviceError: !!raw.deviceError, _match: descriptor(raw.command.replace(/ +/g, " ").toUpperCase()), timeoutAt: 0 };
            if (raw.status === "waiting") { warning(r, "软件重启，原等待已结束；未重发命令。"); }
            if (/^(waiting|timeout|ended|interrupted)$/.test(raw.status)) { markStale(r, nowValue()); }
            if (Array.isArray(raw.warnings)) {
                for (j = 0; j < Math.min(raw.warnings.length, 20); j += 1) { if (typeof raw.warnings[j] === "string") { warning(r, raw.warnings[j].substring(0, 512)); } }
            }
            e.reports.push(r);
            r.lineDrops += Math.max(0, raw.lines.length - cfg.maxLinesPerReport);
            e.stats.droppedReportLines += Math.max(0, raw.lines.length - cfg.maxLinesPerReport);
            for (j = Math.max(0, raw.lines.length - cfg.maxLinesPerReport); j < raw.lines.length; j += 1) {
                item = restoreLine(raw.lines[j]); if (item) { addLine(r, item); maxSeq = Math.max(maxSeq, item.seq); }
            }
            maxId = Math.max(maxId, r.id);
        }
        for (i = Math.max(0, input.events.length - cfg.maxEvents); i < input.events.length; i += 1) {
            item = restoreLine(input.events[i]); if (item) { addEvent(item); maxSeq = Math.max(maxSeq, item.seq); }
        }
        seq = maxSeq; nextId = maxId + 1; trimReports();
        if (Array.isArray(input.warnings)) {
            for (i = 0; i < Math.min(input.warnings.length, 20); i += 1) { if (typeof input.warnings[i] === "string") { warn(input.warnings[i].substring(0, 512)); } }
        }
        changed(); return result(true, "", true);
    };
    e.exportText = function (id) {
        var list = [], r, i, lines, n;
        if (id === null || typeof id === "undefined" || id === "events") {
            list.push("VOFA 报告助手 — 主动消息（仅保留缓存范围）");
            lines = e.events;
        } else {
            r = e.getReport(id); if (!r) { return ""; }
            list.push("VOFA 报告助手 — #" + r.id + " " + r.command);
            list.push("发送时间: " + new Date(r.sentAt).toISOString() + " | 状态: " + r.status);
            for (i = 0; i < r.warnings.length; i += 1) { list.push("[提示] " + r.warnings[i]); }
            lines = r.lines;
        }
        for (i = 0; i < lines.length; i += 1) {
            n = lines[i]; list.push("[" + new Date(n.at).toISOString() + "]" + (n.uncertain ? " [归属待确认] " : " ") + n.text);
        }
        return list.join("\r\n") + "\r\n";
    };
    return e;
}
