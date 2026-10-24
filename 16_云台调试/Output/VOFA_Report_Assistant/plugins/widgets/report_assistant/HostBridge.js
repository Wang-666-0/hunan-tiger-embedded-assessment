/* Qt 5.14 ES5. Read-only observer of VOFA's already-rendered receive document.
 * NEVER read text_buffer.text: that getter consumes a host-owned receive queue.
 * NEVER call all_text(), or insert/remove/clear on the host's text document.
 * No serial port, host setting, native layout, or waveform is changed here.
 */
var _root = null, _manager = null, _hub = null, _buffer = null, _edit = null;
var _connections = [], _tap = null, _decoder = null, _receiveOK = false, _token = 0;
var _lastConnected = false, _qt = null, _clipboard = null;
var _storageBase = "", _protocolCombo = null;

function bridgeResult(ok, error, extra) {
    var result = {ok:!!ok, error:error || ""};
    if (extra) for (var name in extra) if (extra.hasOwnProperty(name)) result[name] = extra[name];
    return result;
}
function timestamp() { return new Date().getTime(); }
function boundedInt(value, fallback, minimum, maximum) {
    var n = Number(value);
    return isFinite(n) ? Math.max(minimum, Math.min(maximum, Math.floor(n))) : fallback;
}
function suffixPrefixOverlap(oldTail, newText) {
    // Linear KMP: suffix(oldTail) == prefix(newText). Both patterns are bounded.
    var pattern = newText.slice(0, oldTail.length), pi = [], i, q = 0;
    if (!pattern.length) return 0;
    pi[0] = 0;
    for (i = 1; i < pattern.length; ++i) {
        var k = pi[i - 1];
        while (k > 0 && pattern.charAt(i) !== pattern.charAt(k)) k = pi[k - 1];
        if (pattern.charAt(i) === pattern.charAt(k)) k++;
        pi[i] = k;
    }
    for (i = 0; i < oldTail.length; ++i) {
        while (q > 0 && (q === pattern.length || oldTail.charAt(i) !== pattern.charAt(q))) q = pi[q - 1];
        if (oldTail.charAt(i) === pattern.charAt(q)) q++;
    }
    return q;
}
function createDocumentTap(edit, callbacks, options) {
    callbacks = callbacks || {}; options = options || {};
    var maxRead = boundedInt(options.maxRead, 65536, 256, 262144);
    var anchorSize = boundedInt(options.anchorSize, 2048, 16, 8192);
    var snapshot = "", tail = "", stopped = false, processing = false, queued = false;
    function docLength() {
        var n = Number(edit.length);
        if (!isFinite(n) || n < 0 || n > 2147483647) throw new Error("宿主文本长度无效");
        return Math.floor(n);
    }
    function read(start, end) {
        if (end - start > maxRead) throw new Error("读取范围超过插件上限");
        var value = edit.getText(Math.max(0,start), Math.max(0,end));
        if (typeof value !== "string") throw new Error("宿主未返回明文文本");
        if (value.length > maxRead + 4) throw new Error("宿主返回文本超过读取上限");
        return value;
    }
    function gap(reason) { if (callbacks.gap) callbacks.gap(reason, timestamp()); }
    function readSnapshot() {
        // Native QTextDocument positions and returned plain-string indices need
        // not coincide (formatting/paragraph markers). Never reuse a string
        // length as a document position. The native window is itself bounded.
        var end = docLength(), start = Math.max(0, end - maxRead);
        var text = read(start, end).replace(/\u2029|\u2028/g,"\n");
        return {text:text,clipped:start>0};
    }
    function baseline() {
        snapshot = readSnapshot().text;
        tail = snapshot.slice(-anchorSize);
    }
    function emit(text) { if (text && callbacks.text) callbacks.text(text.replace(/\u2029|\u2028/g,"\n"), timestamp()); }
    function recover(recent, clipped) {
        var overlap = 0, match = -1;
        if (tail) match = recent.indexOf(tail);
        if (match >= 0) {
            if (recent.indexOf(tail, match + 1) >= 0) gap("宿主文本重排且重复内容使连续性无法确认");
            return recent.slice(match + tail.length);
        }
        if (tail) overlap = suffixPrefixOverlap(tail, recent);
        // VOFA crops the display buffer in ANSI-formatted storage before plain
        // getText conversion. Its first line may therefore acquire a different
        // formatting fragment even while all subsequent lines are continuous.
        // Retry overlap at the next whole plain-text line, never replay it.
        var newline = recent.indexOf("\n"), cleanOverlap = 0;
        if (tail && newline >= 0) cleanOverlap = suffixPrefixOverlap(tail, recent.slice(newline + 1));
        if (cleanOverlap > overlap) {
            recent = recent.slice(newline + 1);
            overlap = cleanOverlap;
        }
        if (overlap > 0) {
            if (overlap < Math.min(32, tail.length)) gap("宿主裁剪后只有很短的重叠内容，连续性无法完全确认");
            return recent.slice(overlap);
        }
        gap("宿主接收区被清空、替换或裁剪，部分原文可能不完整");
        if (clipped) {
            var nl = recent.indexOf("\n");
            recent = nl >= 0 ? recent.slice(nl + 1) : "";
        }
        return recent;
    }
    function processChange() {
        if (stopped || processing) return;
        processing = true;
        try {
            var current = readSnapshot(), next = "";
            if (current.text.slice(0,snapshot.length) === snapshot && !current.clipped) next = current.text.slice(snapshot.length);
            else next = recover(current.text,current.clipped);
            snapshot = current.text;
            tail = snapshot.slice(-anchorSize);
            emit(next);
            if (next && callbacks.batchEnd) callbacks.batchEnd(timestamp());
        } catch (err) {
            gap("读取宿主接收文档失败：" + err);
            if (callbacks.error) callbacks.error("" + err);
        } finally { processing = false; }
    }
    function changed() {
        if (stopped || queued) return;
        if (typeof options.defer === "function") {
            queued = true;
            options.defer(function(){queued=false;processChange();});
        } else processChange();
    }
    baseline();
    return {changed:changed, baseline:baseline, stop:function(){stopped=true;},
        limits:{maxRead:maxRead,anchorSize:anchorSize}};
}
function createPresentationDecoder(callbacks, options) {
    callbacks = callbacks || {}; options = options || {};
    var pending = "", max = boundedInt(options.maxLine, 16384, 256, 32768);
    var completeParagraphs = options.completeParagraphs === true, skipBoundary = false;
    function feed(text, at) {
        if (skipBoundary && text) {
            // A committed FireWater paragraph's display newline appears only
            // when the next native paragraph is appended. Do not emit it twice.
            if (text.charAt(0) === "\n") text = text.slice(1);
            skipBoundary = false;
        }
        pending += text;
        var parts = pending.split("\n"), complete = [];
        pending = parts.pop();
        for (var i = 0; i < parts.length; ++i) {
            var raw = parts[i].replace(/\r$/, "");
            // Confirmed VOFA 1.3.10 plain-document time decoration, only at BOL.
            complete.push(raw.replace(/^\[\d{2}:\d{2}:\d{2}\.\d{3}\] /,""));
        }
        if (complete.length && callbacks.text) callbacks.text(complete.join("\n") + "\n", at);
        if (pending.length > max) {
            pending = "";
            if (callbacks.gap) callbacks.gap("宿主尾行超过插件缓存上限，已丢弃不完整尾行");
        }
    }
    function batchEnd(at) {
        if (!completeParagraphs || !pending) return;
        var raw = pending.replace(/\r$/, ""); pending = "";
        if (callbacks.text) callbacks.text(raw.replace(/^\[\d{2}:\d{2}:\d{2}\.\d{3}\] /,"") + "\n",at);
        skipBoundary = true;
    }
    return {feed:feed,batchEnd:batchEnd,reset:function(){pending="";skipBoundary=false;},
        setCompleteParagraphs:function(value){completeParagraphs=value===true;},pendingLength:function(){return pending.length;}};
}
function findProtocolCombo(starts, engineNames, maxNodes) {
    var names = [], i;
    if (!engineNames || typeof engineNames.length !== "number" || engineNames.length < 2 || engineNames.length > 32) return null;
    for (i = 0; i < engineNames.length; ++i) names.push(String(engineNames[i]).toLowerCase());
    var todo = starts.slice(0), seen = [], matches = [], steps = 0, cap = maxNodes || 10000;
    while (todo.length && steps++ < cap) {
        var item = todo.pop();
        if (!item || seen.indexOf(item) >= 0) continue;
        seen.push(item);
        try {
            if (item.count === names.length && typeof item.textAt === "function") {
                var same = true;
                for (i = 0; i < names.length; ++i)
                    if (String(item.textAt(i)).toLowerCase() !== names[i]) { same=false; break; }
                if (same && typeof item.currentText === "string") matches.push(item);
            }
        } catch (ignoreCombo) {}
        // User-added widgets can have very large graphs. The native protocol
        // selector is part of the host left panel, never inside a widget.
        try { if (typeof item.get_widget_ctx === "function" && typeof item.path === "string") continue; } catch (ignoreWidget) {}
        try { if (item.children) for (i = 0; i < Math.min(item.children.length,512); ++i) todo.push(item.children[i]); } catch (ignoreChildren) {}
        try { if (item.contentItem) todo.push(item.contentItem); } catch (ignoreContent) {}
    }
    // More than one matching selector is ambiguous, even if both say FireWater.
    return matches.length === 1 ? matches[0] : null;
}
function fireWaterConfirmed() {
    try { return !!_protocolCombo && String(_protocolCombo.currentText).toLowerCase() === "firewater"; }
    catch (ignore) { return false; }
}
function findReceiveEdit(starts, document, maxNodes) {
    if (!document) return null;
    var todo = starts.slice(0), seen = [], steps = 0, cap = maxNodes || 5000;
    while (todo.length && steps++ < cap) {
        var item = todo.shift();
        if (!item || seen.indexOf(item) >= 0) continue;
        seen.push(item);
        try {
            if (sameDocument(item.textDocument, document) && typeof item.getText === "function" && typeof item.length === "number") return item;
        } catch (ignore) {}
        try {
            var children = item.children;
            if (children) for (var i = 0; i < Math.min(children.length, 512); ++i) todo.push(children[i]);
        } catch (ignoreChildren) {}
        try { if (item.contentItem) todo.push(item.contentItem); } catch (ignoreContent) {}
    }
    return null;
}
function sameDocument(left, right) {
    if (!left || !right) return false;
    if (left === right) return true;
    // The same native QObject may have different JavaScript wrappers in separate
    // QML contexts. Compare only verified QQuickTextDocument native identities.
    var a = /^QQuickTextDocument\((0x[0-9a-f]+)\)$/i.exec(String(left));
    var b = /^QQuickTextDocument\((0x[0-9a-f]+)\)$/i.exec(String(right));
    return !!a && !!b && a[1].toLowerCase() === b[1].toLowerCase();
}
function readReferenceHolder(owner) {
    var refs = {}, loader = null, wasActive;
    try {
        loader = owner && owner.ref;
        if (!loader || typeof loader.active !== "boolean") return refs;
        wasActive = loader.active;
        if (!wasActive) loader.active = true;
        var item = loader.item;
        if (item) {
            var names = Object.keys(item);
            for (var i = 0; i < names.length; ++i)
                if (/^ref_/.test(names[i])) refs[names[i]] = item[names[i]];
        }
    } catch (ignore) {}
    finally {
        // Ref Loaders contain lightweight references, not the native layout.
        // Restore their prior state before returning captured existing objects.
        if (loader && typeof wasActive === "boolean") try { loader.active = wasActive; } catch (ignoreRestore) {}
    }
    return refs;
}
function connectSignal(object, name, handler) {
    try {
        var signal = object && object[name];
        if (signal && typeof signal.connect === "function") {
            signal.connect(handler); _connections.push({signal:signal,handler:handler}); return true;
        }
    } catch (ignore) {}
    return false;
}
function receiveConfigurationOK() {
    try { return !!_buffer && _buffer.if_show_hex === false && _buffer.if_show_echo === true && fireWaterConfirmed(); }
    catch (ignore) { return false; }
}
function notifyGap(reason) {
    if (_decoder) _decoder.reset();
    try { if (_root && typeof _root.noteRxGap === "function") _root.noteRxGap(reason, timestamp()); } catch (ignore) {}
}
function publishStatus() {
    if (!_root) return;
    var connected = false;
    try { connected = _manager && _manager.connected === true; } catch (ignore) {}
    var rx = !!_edit && !!_tap && receiveConfigurationOK();
    if (_receiveOK && !rx) notifyGap("宿主协议、十六进制或接收显示设置改变，报告接收已中断");
    if (_lastConnected && !connected) notifyGap("VOFA 连接已断开，等待中的报告可能不完整");
    _receiveOK = rx; _lastConnected = connected;
    var message;
    if (!_edit || !_tap) message = "未找到 VOFA 接收文档接口；尚未接入，禁止发送。可重试接入。";
    else if (!fireWaterConfirmed()) message = "无法确认当前为 FireWater 协议；助手未接收，禁止发送。插件不会切换协议。";
    else if (!receiveConfigurationOK()) message = "宿主当前为十六进制或已隐藏接收；助手未接收，禁止发送。插件不会更改这些设置。";
    else message = "已接入 VOFA FireWater 显示文本（非原始串口流）· " + (connected ? "连接已建立" : "VOFA 尚未连接") + " · 清空/隐藏原接收区会影响完整性";
    try { _root.setBridgeStatus(message, rx, rx && connected && typeof _manager.send_string === "function"); } catch (ignoreStatus) {}
}
function attach(root) {
    detach(); _root = root; _token++;
    var token = _token, context = {};
    try { if (root && typeof root.getBridgeContext === "function") context = root.getBridgeContext() || {}; } catch (ignoreContext) {}
    try { _manager = context.manager || (typeof sys_manager !== "undefined" ? sys_manager : null); } catch (ignoreManager) {}
    try { _hub = context.hub || (typeof datahub_global !== "undefined" ? datahub_global : null); } catch (ignoreHub) {}
    try { _qt = context.qt || (typeof Qt !== "undefined" ? Qt : null); } catch (ignoreQt) {}
    var windowRefs = readReferenceHolder(context.window), waveRefs = {};
    try { waveRefs = readReferenceHolder(_manager && _manager.parent); } catch (ignoreWave) {}
    if (!_hub) _hub = windowRefs.ref_datahub_global || null;
    try { _buffer = _hub && _hub.text_buffer; } catch (ignoreBuffer) {}
    var rxZone = waveRefs.ref_tx_rx_zone;
    var starts = rxZone ? [rxZone] : [], ancestor = _manager;
    // The receive zone may itself expose the text editor through its own holder.
    if (rxZone) {
        var rxRefs = readReferenceHolder(rxZone);
        for (var refName in rxRefs) if (rxRefs.hasOwnProperty(refName)) starts.push(rxRefs[refName]);
    }
    starts.push(root);
    for (var i = 0; ancestor && i < 16; ++i) {
        starts.push(ancestor);
        try { ancestor = ancestor.parent; } catch (ignoreParent) { break; }
    }
    if (context.window) starts.push(context.window);
    var engineNames = null;
    try { engineNames = _hub.data_engine_loader.name_list(); } catch (ignoreNames) {}
    _protocolCombo = waveRefs.ref_left_panel ? findProtocolCombo([waveRefs.ref_left_panel],engineNames,10000) : null;
    if (!_protocolCombo) _protocolCombo = findProtocolCombo(starts,engineNames,10000);
    try { _edit = _buffer ? findReceiveEdit(starts, _buffer.doc, 5000) : null; } catch (ignoreFind) {}
    if (_edit) {
        try {
            _decoder = createPresentationDecoder({
                text:function(text,at){if(token===_token && _root && _receiveOK) _root.feedRx(text,at);},
                gap:function(reason){if(token===_token)notifyGap(reason);}
            },{completeParagraphs:fireWaterConfirmed()});
            _tap = createDocumentTap(_edit, {
                text:function(text, at) {
                    if (token !== _token || !_root) return;
                    publishStatus();
                    if (_receiveOK) _decoder.feed(text, at);
                },
                gap:function(reason) { if (token === _token) notifyGap(reason); },
                error:function(){ if (token === _token) { _receiveOK=false; _tap=null; publishStatus(); } },
                batchEnd:function(at) {
                    if (token === _token && _decoder && _receiveOK) {
                        _decoder.setCompleteParagraphs(fireWaterConfirmed());
                        _decoder.batchEnd(at);
                    }
                }
            }, {defer:_qt && typeof _qt.callLater === "function" ? function(fn){_qt.callLater(fn);} : null});
            if (!connectSignal(_edit,"textChanged",function(){
                if (token !== _token) return;
                publishStatus();
                if (_receiveOK) _tap.changed();
                else try { _tap.baseline(); } catch (ignoreBaseline) {}
            })) _tap = null;
        } catch (ignoreTap) { _tap = null; }
    }
    var statusHandler = function(){ if (token === _token) publishStatus(); };
    connectSignal(_manager,"connectedChanged",statusHandler);
    connectSignal(_buffer,"if_show_hex_changed",statusHandler);
    connectSignal(_protocolCombo,"currentTextChanged",statusHandler);
    connectSignal(_hub,"text_message_updated",statusHandler);
    publishStatus();
    return bridgeResult(!!_tap, _tap ? "" : "尚未找到兼容的显示文本接口");
}
function detach() {
    _token++;
    for (var i = 0; i < _connections.length; ++i)
        try { _connections[i].signal.disconnect(_connections[i].handler); } catch (ignore) {}
    _connections = [];
    if (_tap) _tap.stop();
    if (_clipboard) try { _clipboard.destroy(); } catch (ignoreClipboard) {}
    _root = null; _manager = null; _hub = null; _buffer = null; _edit = null;
    _tap = null; _decoder = null; _protocolCombo = null; _clipboard = null; _receiveOK = false; _lastConnected = false; _storageBase = "";
}
function sendCommand(command) {
    if (typeof command !== "string" || !command.trim() || command.length > 63 || /[\x00-\x1f\x7f]/.test(command))
        return bridgeResult(false,"命令需为 1 至 63 个字符的单行文本");
    publishStatus();
    if (!_receiveOK || !_manager || _manager.connected !== true || typeof _manager.send_string !== "function")
        return bridgeResult(false,"VOFA 接收或连接尚未就绪，未发送");
    try {
        // One invocation, one LF. Do not retry or alter the host's suffix control.
        _manager.send_string(command.trim() + "\n");
        return bridgeResult(true,"");
    } catch (err) { return bridgeResult(false,"发送接口失败：" + err); }
}
function storageContext() {
    var reader = _manager && _manager.file_reader;
    if (!reader || typeof reader.applicationDirPath !== "function") throw new Error("宿主文件接口不可用");
    if (!_storageBase) {
        var appdir = String(reader.applicationDirPath()).replace(/\\/g,"/").replace(/\/+$/,"");
        if (!/^(?:[A-Za-z]:\/|\/)/.test(appdir) || /(?:^|\/)\.\.(?:\/|$)/.test(appdir)) throw new Error("宿主安装路径无效");
        _storageBase = appdir + "/plugins/widgets/report_assistant";
    }
    return {reader:reader, base:_storageBase};
}
function readOwnFile(file) {
    var context = storageContext(), reader = context.reader, old = reader.source;
    try {
        var target = context.base + "/" + file;
        if (typeof reader.exist === "function" && !reader.exist(target)) return "";
        reader.source = target;
        var text = reader.read();
        if (typeof text !== "string") throw new Error("读取未返回文本");
        if (text.length > 6291456) throw new Error("历史文件超过 6 MB 上限");
        return text;
    } finally { reader.source = old; }
}
function writeOwnFile(folder, name, text) {
    var context = storageContext(), reader = context.reader, old = reader.source;
    var directory = context.base + "/" + folder, target = directory + "/" + name;
    try {
        if (typeof reader.mkdir !== "function") throw new Error("宿主缺少创建插件数据目录接口");
        var exists = typeof reader.exist === "function" && reader.exist(directory);
        if (!exists && reader.mkdir(directory) === false) throw new Error("无法创建插件数据目录");
        reader.source = target;
        if (reader.write(text) !== true) throw new Error("写入未成功，当前内存数据仍保留");
        return bridgeResult(true,"",{path:target});
    } catch (err) { return bridgeResult(false,"" + err); }
    finally { reader.source = old; }
}
function readState() { return readOwnFile("state/history-v1.json"); }
function writeState(text) {
    if (typeof text !== "string" || text.length > 6291456) return bridgeResult(false,"历史内容超过 6 MB 上限");
    try { JSON.parse(text); } catch (err) { return bridgeResult(false,"历史内容不是有效 JSON"); }
    try { return writeOwnFile("state","history-v1.json",text); } catch (err) { return bridgeResult(false,"" + err); }
}
function copyText(text) {
    if (typeof text !== "string" || text.length > 6291456) return bridgeResult(false,"复制内容无效或过大");
    if (!_root || !_qt || typeof _qt.createQmlObject !== "function") return bridgeResult(false,"请先启用助手，再使用复制；原文也可选中复制");
    try {
        if (!_clipboard) _clipboard = _qt.createQmlObject('import QtQuick 2.12; TextEdit { visible:false; width:0; height:0; readOnly:true }',_root,"report_assistant_clipboard");
        _clipboard.text = text;
        _clipboard.selectAll(); _clipboard.copy(); _clipboard.text = "";
        return bridgeResult(true,"");
    } catch (err) { return bridgeResult(false,"复制接口未完成：" + err); }
}
function exportText(text, suggestedName) {
    if (typeof text !== "string" || text.length > 6291456) return bridgeResult(false,"导出内容无效或过大");
    var name = typeof suggestedName === "string" ? suggestedName : "Pitch_Report_" + timestamp() + ".txt";
    if (!/^[A-Za-z0-9][A-Za-z0-9_.-]{0,100}\.txt$/.test(name) || name.indexOf("..") >= 0)
        return bridgeResult(false,"导出文件名无效");
    try { return writeOwnFile("exports",name,text); } catch (err) { return bridgeResult(false,"" + err); }
}
