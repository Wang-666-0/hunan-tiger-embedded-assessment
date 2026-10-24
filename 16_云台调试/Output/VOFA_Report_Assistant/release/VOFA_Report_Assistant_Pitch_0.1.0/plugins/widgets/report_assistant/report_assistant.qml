import QtQuick 2.14
import QtQuick.Window 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0
import "ReportEngine.js" as ReportEngine
import "HostBridge.js" as HostBridge

// This is an additional widget. It never loads, saves, or replaces a VOFA layout.
ResizableRectangle {
    id: root
    property string path: "report_assistant"
    width: 940
    height: 600
    minimumWidth: 700
    minimumHeight: 450
    full_parent_enabled: true
    radius: 6
    color: "#f4f6f8"
    border.color: "#cbd3da"
    border.width: 1

    property var reportEngine: null
    property bool activeStage: false
    property bool rxAvailable: false
    property bool sendAvailable: false
    property bool demoMode: false
    property string bridgeStatus: "接收接口尚未验证；可先查看离线演示"
    property string notice: ""
    property int selectedReportId: -1
    property int selectedTab: 0
    property bool followLatest: true
    property int unseenLines: 0
    property int hiddenTelemetry: 0
    property int pendingReportId: -1
    property int commandHistoryIndex: -1
    property var commandHistory: []
    property var favoriteCommands: [
        {name:"Pitch 状态", command:"PITCH STATUS"},
        {name:"PID 参数", command:"PITCH PID"},
        {name:"检查反馈", command:"PITCH CHECK"},
        {name:"阶跃成绩", command:"PITCH RESULT"},
        {name:"保护状态", command:"PITCH GUARD"},
        {name:"命令帮助", command:"PITCH HELP"}
    ]
    property string preferences_json: ""
    property string realStateBeforeDemo: ""
    property var demoChunks: []
    property int demoChunkIndex: 0
    property int renderedLineCount: 0
    property string renderedKey: ""
    property bool modelUpdating: false

    // Required by VOFA's widget-context mechanism; only this widget's geometry
    // and small preferences enter the containing layout, never report payloads.
    property Item ref: Loader { active: false; sourceComponent: Component { Item {} } }

    ListModel { id: reportsModel }
    ListModel { id: linesModel }

    function now() { return new Date().getTime(); }
    function getBridgeContext() {
        return {manager:typeof sys_manager !== "undefined" ? sys_manager : null,
            hub:typeof datahub_global !== "undefined" ? datahub_global : null,
            window:root.Window.window, qt:Qt};
    }
    function reportById(id) {
        if (!reportEngine) return null;
        for (var i = reportEngine.reports.length - 1; i >= 0; --i)
            if (reportEngine.reports[i].id === id) return reportEngine.reports[i];
        return null;
    }
    function timeText(ms) {
        var d = new Date(ms);
        function p(v) { return v < 10 ? "0" + v : "" + v; }
        return p(d.getHours()) + ":" + p(d.getMinutes()) + ":" + p(d.getSeconds());
    }
    function statusText(status) {
        var names = {waiting:"等待回复", complete:"完成", uncertain:"归属待确认",
            error:"错误", timeout:"等待超时", ended:"已结束等待", interrupted:"连接已中断"};
        return names[status] || status;
    }
    function scheduleRefresh() { if (!refreshTimer.running) refreshTimer.start(); }
    function scheduleSave() { if (!saveTimer.running) saveTimer.start(); }
    function feedRx(text, receivedAt) {
        if (!reportEngine || demoMode) return;
        var result = reportEngine.feed(text, receivedAt || now());
        // Telemetry-only feeds do not cause delegate creation or UI refresh.
        if (result.changed) { scheduleRefresh(); scheduleSave(); }
    }
    function setBridgeStatus(text, canReceive, canSend) {
        bridgeStatus = text || "接收接口尚未验证";
        rxAvailable = !!canReceive;
        sendAvailable = !!canSend;
    }
    function noteRxGap(reason, receivedAt) {
        if (!reportEngine || demoMode) return;
        reportEngine.noteGap(reason || "接收数据不连续", receivedAt || now());
        scheduleRefresh();
        scheduleSave();
    }
    function activate() {
        if (activeStage) return;
        activeStage = true;
        // A damaged history must not prevent the receive adapter from attaching.
        try {
            HostBridge.attach(root);
        } catch (e) { activeStage = false; notice = "接入未完成：" + e; }
        if (activeStage) {
            try {
                var saved = HostBridge.readState();
                if (saved) loadState(saved);
            } catch (historyError) { notice = "历史读取未完成：" + historyError; }
        }
        scheduleRefresh();
    }
    function retryAttach() {
        try { HostBridge.attach(root); } catch (e) { notice = "接入未完成：" + e; }
    }
    function saveState() {
        return JSON.stringify({version:1, engine:reportEngine ? reportEngine.serialize() : "",
            history:commandHistory.slice(-100), favorites:favoriteCommands,
            rules:reportEngine ? reportEngine.getRules() : []});
    }
    function loadState(data) {
        try {
            var obj = typeof data === "string" ? JSON.parse(data) : data;
            if (!obj || obj.version !== 1) return;
            if (obj.engine) {
                var restored = reportEngine.restore(obj.engine);
                if (restored && !restored.ok) notice = restored.error;
            }
            if (obj.history instanceof Array)
                commandHistory = obj.history.filter(function(v) { return typeof v === "string"; }).slice(-100);
            if (obj.favorites instanceof Array && obj.favorites.length > 0 && obj.favorites.length <= 12)
                favoriteCommands = obj.favorites.filter(function(v) {
                    return v && typeof v.name === "string" && typeof v.command === "string";
                });
            if (obj.rules instanceof Array) reportEngine.setRules(obj.rules);
            if (reportEngine.reports.length) selectedReportId = reportEngine.reports[reportEngine.reports.length - 1].id;
        } catch (e) { notice = "历史文件未能读取，当前会话仍可使用：" + e; }
    }
    function flushState() {
        if (!activeStage || demoMode || !reportEngine) return;
        try {
            var result = HostBridge.writeState(saveState());
            if (result && result.ok === false) notice = "历史尚未保存：" + result.error;
        } catch (e) { notice = "历史尚未保存：" + e; }
    }
    function selectReport(id) {
        selectedReportId = id;
        selectedTab = 0;
        followLatest = true;
        unseenLines = 0;
        renderedKey = "";
        refreshView();
    }
    function refreshView() {
        if (!reportEngine) return;
        pendingReportId = reportEngine.pendingId || -1;
        hiddenTelemetry = reportEngine.stats.telemetryLines;
        var reports = reportEngine.reports;
        // At most 100 small summaries. Retain delegates/selection whenever possible.
        if (reportsModel.count !== reports.length ||
            (reports.length && reportsModel.get(0).reportId !== reports[reports.length - 1].id)) {
            reportsModel.clear();
            for (var r = reports.length - 1; r >= 0; --r)
                reportsModel.append({reportId:reports[r].id, command:reports[r].command,
                    sentAt:reports[r].sentAt, state:reports[r].status});
        } else {
            for (var n = 0; n < reports.length; ++n) {
                var report = reports[reports.length - 1 - n];
                if (reportsModel.get(n).state !== report.status) reportsModel.setProperty(n, "state", report.status);
            }
        }
        if (selectedReportId >= 0 && !reportById(selectedReportId)) selectedReportId = -1;
        var selected = reportById(selectedReportId);
        reportHeading.text = selectedTab === 1 ? "主动消息 / 未归档文字" :
            (selected ? selected.command + " · " + statusText(selected.status) : "选择左侧报告，或发送一条查询命令");
        var warnings = reportEngine.warnings.slice(-2);
        if (selected && selected.warnings) warnings = warnings.concat(selected.warnings);
        warningLabel.text = warnings.join("；");
        otherTab.text = "主动消息 · " + reportEngine.events.length;
        var source = selectedTab === 1 ? reportEngine.events : (selected ? selected.lines : []);
        var key = selectedTab === 1 ? "events" : "report-" + selectedReportId;
        var reset = renderedKey !== key;
        var oldY = transcript.contentY;
        var anchorIndex = transcript.indexAt(0, oldY + 1);
        var anchorSeq = anchorIndex >= 0 && anchorIndex < linesModel.count ? linesModel.get(anchorIndex).seq : -1;
        var anchorItem = anchorIndex >= 0 ? transcript.itemAtIndex(anchorIndex) : null;
        var anchorOffset = anchorItem ? oldY - anchorItem.y : 0;
        var keepFollowing = followLatest;
        modelUpdating = true;
        if (!reset) {
            // The bounded engine may evict old lines. Remove only the evicted
            // prefix instead of replacing the entire text model each update.
            if (!source.length) reset = linesModel.count > 0;
            else {
                while (linesModel.count && linesModel.get(0).seq < source[0].seq) linesModel.remove(0);
                if (linesModel.count && linesModel.get(0).seq !== source[0].seq) reset = true;
            }
        }
        if (reset) { linesModel.clear(); renderedLineCount = 0; renderedKey = key; }
        else renderedLineCount = linesModel.count;
        var added = source.length - renderedLineCount;
        for (var j = renderedLineCount; j < source.length; ++j) {
            var line = source[j];
            linesModel.append({body:line.text, stamp:timeText(line.at), seq:line.seq,
                level:line.level || "info", uncertain:!!line.uncertain});
        }
        renderedLineCount = source.length;
        modelUpdating = false;
        if (added > 0 && !keepFollowing) unseenLines += added;
        var anchorNewIndex = -1;
        if (!keepFollowing && anchorSeq >= 0)
            for (var a = 0; a < source.length; ++a) if (source[a].seq === anchorSeq) { anchorNewIndex = a; break; }
        Qt.callLater(function() {
            if (keepFollowing) transcript.positionViewAtEnd();
            else if (anchorNewIndex >= 0) {
                transcript.positionViewAtIndex(anchorNewIndex, ListView.Beginning);
                var anchor = transcript.itemAtIndex(anchorNewIndex);
                if (anchor) transcript.contentY = anchor.y + anchorOffset;
            } else if (!reset) transcript.contentY = oldY;
        });
    }
    function rememberCommand(command) {
        var list = commandHistory.slice(0);
        if (!list.length || list[list.length - 1] !== command) list.push(command);
        commandHistory = list.slice(-100);
        commandHistoryIndex = commandHistory.length;
    }
    function sendCommand(command) {
        var text = (command || "").trim();
        if (!text) return;
        if (/[\r\n]/.test(text)) { notice = "一次只发送一条命令，请移除换行。"; return; }
        if (!demoMode && !activeStage) activate();
        if (!demoMode && (!rxAvailable || !sendAvailable)) {
            notice = "接收接口尚未就绪，未发送。请先启用接收并完成接口验证。"; return;
        }
        var result = reportEngine.start(text, now());
        if (!result.ok) { notice = result.error; return; }
        // Record before TX: a synchronous host reply must have a pending report.
        selectedReportId = result.id;
        selectedTab = 0;
        followLatest = true;
        unseenLines = 0;
        renderedKey = "";
        rememberCommand(text);
        notice = "";
        if (demoMode) beginDemoReply(text);
        else {
            var sent;
            try { sent = HostBridge.sendCommand(text); }
            catch (e) { sent = {ok:false,error:"" + e}; }
            if (!sent || !sent.ok) {
                reportEngine.failSend(sent ? sent.error : "发送接口无确认", now());
                notice = "发送失败：" + (sent ? sent.error : "发送接口无确认");
            }
        }
        refreshView();
        scheduleSave();
        commandInput.forceActiveFocus();
    }
    function stopWaiting() {
        if (!reportEngine || !reportEngine.pendingId) return;
        reportEngine.endWait(now());
        notice = "已结束面板等待；单片机正在执行的动作不会因此停止。";
        refreshView();
        scheduleSave();
    }
    function commandHistoryMove(delta) {
        if (!commandHistory.length) return;
        if (commandHistoryIndex < 0) commandHistoryIndex = commandHistory.length;
        commandHistoryIndex = Math.max(0, Math.min(commandHistory.length, commandHistoryIndex + delta));
        commandInput.text = commandHistoryIndex === commandHistory.length ? "" : commandHistory[commandHistoryIndex];
    }
    function currentExportText() {
        if (!reportEngine) return "";
        if (selectedTab === 0) return reportEngine.exportText(selectedReportId);
        return reportEngine.events.map(function(line) { return timeText(line.at) + " " + line.text; }).join("\r\n");
    }
    function copyCurrent() {
        try {
            var result = HostBridge.copyText(currentExportText());
            notice = result && result.ok ? "已复制正文。" : "复制未完成：" + (result ? result.error : "接口不可用");
        } catch (e) { notice = "复制未完成：" + e; }
    }
    function exportCurrent() {
        try {
            var result = HostBridge.exportText(currentExportText(), "Pitch_Report_" + now() + ".txt");
            notice = result && result.ok ? "已导出：" + result.path : "导出未完成：" + (result ? result.error : "接口不可用");
        } catch (e) { notice = "导出未完成：" + e; }
    }
    function findNext() {
        var query = searchInput.text.toLowerCase();
        if (!query || !linesModel.count) return;
        var first = Math.max(0, transcript.currentIndex + 1);
        for (var i = 0; i < linesModel.count; ++i) {
            var n = (first + i) % linesModel.count;
            if (linesModel.get(n).body.toLowerCase().indexOf(query) >= 0) {
                followLatest = false;
                transcript.currentIndex = n;
                transcript.positionViewAtIndex(n, ListView.Contain);
                return;
            }
        }
        notice = "正文中没有找到：" + searchInput.text;
    }
    function setDemo(enabled) {
        if (enabled === demoMode) return;
        demoTimer.stop();
        if (enabled) {
            flushState();
            realStateBeforeDemo = saveState();
            reportEngine = ReportEngine.createEngine({maxReports:100});
            demoMode = true;
            selectedReportId = -1;
            notice = "离线演示：按钮只生成模拟数据，不发送到串口。";
        } else {
            demoMode = false;
            reportEngine = ReportEngine.createEngine({maxReports:100});
            loadState(realStateBeforeDemo);
            notice = "已退出演示，恢复真实会话历史。";
        }
        renderedKey = "";
        refreshView();
    }
    function beginDemoReply(command) {
        var replies = {
            "PITCH STATUS":"P|STATUS|demo|QUERY|SNAPSHOT_ONLY|ONLINE|1|ACTIVE|0|FAULT|0|ANGLE|0.012\r\n",
            "PITCH PID":"P|PID|POS|16.63,0.12,0.269|VEL|0.0081,0.003,0.000209|RUNTIME_RAM_ONLY|1\r\n",
            "PITCH CHECK":"P|CHECK|demo|QUERY|NEW_FEEDBACK|CTRL|CASCADE|ONLINE|1|FAULT|0\r\n",
            "PITCH RESULT":"P|STEP|ID|12|STATE|DONE|TARGET|30.000|VALID|1|SETTLE02_MS|238|OS_DEG|0.12|PASS02|1\r\n",
            "PITCH GUARD":"P|GUARD|demo|TRACE|12|P_FAULT|0\r\n",
            "PITCH HELP":"P|HELP|READ|PITCH;PID;RESULT;STATUS|DEMO_ONLY|1\r\n"
        };
        var reply = replies[command.toUpperCase()] || "P|ERROR|DEMO|仅演示默认查询命令\r\n";
        demoChunks = ["pitch:1,2,3\r\nR|RX|DEMO_UNSOLICITED\r\n",
            command.toUpperCase() === "PITCH CHECK" ? "P|BUSY|CHECK|QUERY|CC\r\npitch:2,3,4\r\n" : "pitch:2,3,4\r\n",
            reply.slice(0, Math.floor(reply.length / 2)),
            reply.slice(Math.floor(reply.length / 2)) + "pitch:3,4,5\r\n"];
        demoChunkIndex = 0;
        demoTimer.start();
    }
    function openSettings() {
        favoritesEditor.text = favoriteCommands.map(function(v) { return v.name + " | " + v.command; }).join("\n");
        groupsEditor.text = reportEngine.getRules().join("\n");
        settingsError.text = "";
        settingsDialog.open();
    }
    function applySettings() {
        var rows = favoritesEditor.text.split(/\r?\n/), favorites = [];
        for (var i = 0; i < rows.length; ++i) {
            if (!rows[i].trim()) continue;
            var split = rows[i].indexOf("|");
            if (split < 1 || !rows[i].slice(split + 1).trim()) {
                settingsError.text = "每行填写：按钮名称 | 单条命令"; return;
            }
            var command = rows[i].slice(split + 1).trim();
            if (command.length > 63 || /[\x00-\x1f\x7f]/.test(command)) {
                settingsError.text = "命令需为 1 至 63 个字符，不能含控制字符。"; return;
            }
            favorites.push({name:rows[i].slice(0, split).trim().slice(0,20), command:command});
        }
        if (!favorites.length || favorites.length > 12) { settingsError.text = "请保留 1 至 12 个常用命令。"; return; }
        var groups = groupsEditor.text.split(/\r?\n/).map(function(v) { return v.trim(); }).filter(function(v) { return v.length; });
        var result = reportEngine.setRules(groups);
        if (result && result.ok === false) { settingsError.text = result.error; return; }
        favoriteCommands = favorites;
        settingsDialog.close();
        scheduleSave();
    }
    function onBind() { /* Reception is explicitly enabled in this widget only. */ }
    function onUnbind() { flushState(); }
    function get_widget_ctx() {
        return {path:path, ctx:{".":{ctx:get_ctx(), preferences_json:JSON.stringify({favorites:favoriteCommands})}}};
    }
    function set_widget_ctx(ctx) {
        if (ctx && ctx.ctx) __set_ctx__(root, ctx.ctx, ref);
    }
    onPreferences_jsonChanged: {
        try { var p = JSON.parse(preferences_json); if (p.favorites instanceof Array) favoriteCommands = p.favorites; } catch (e) {}
    }
    Component.onCompleted: { reportEngine = ReportEngine.createEngine({maxReports:100}); refreshView(); }
    Component.onDestruction: {
        if (activeStage) {
            flushState();
            try { HostBridge.detach(); } catch (e) {}
        }
    }

    Timer { id: refreshTimer; interval: 80; onTriggered: root.refreshView() }
    Timer { id: saveTimer; interval: 2000; onTriggered: root.flushState() }
    Timer {
        interval: 1000
        repeat: true
        running: root.activeStage || root.demoMode
        onTriggered: { if (root.reportEngine) root.hiddenTelemetry = root.reportEngine.stats.telemetryLines; }
    }
    Timer {
        interval: 250
        repeat: true
        running: !!root.reportEngine && root.pendingReportId > 0
        onTriggered: {
            var result = root.reportEngine.tick(root.now());
            if (result.changed) { root.scheduleRefresh(); root.scheduleSave(); }
        }
    }
    Timer {
        id: demoTimer
        interval: 180
        repeat: true
        onTriggered: {
            if (!root.demoMode || root.demoChunkIndex >= root.demoChunks.length) { stop(); return; }
            root.reportEngine.feed(root.demoChunks[root.demoChunkIndex++], root.now());
            root.scheduleRefresh();
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 7
        RowLayout {
            Layout.fillWidth: true
            Label { text: "PITCH / 调试报告"; color: "#18354b"; font.bold: true; font.pixelSize: 19 }
            Item { Layout.fillWidth: true }
            CheckBox { text: "离线演示"; checked: root.demoMode; enabled: root.pendingReportId < 0; onToggled: root.setDemo(checked) }
            Button {
                text: !root.activeStage ? "启用接收" : (root.rxAvailable ? "已启用" : "重试接入")
                enabled: !root.demoMode && (!root.activeStage || !root.rxAvailable)
                onClicked: { if (root.activeStage) root.retryAttach(); else root.activate(); }
            }
            Button { text: "设置"; onClicked: root.openSettings() }
        }
        Label {
            Layout.fillWidth: true
            text: root.demoMode ? "演示模式 · 不发送串口命令" : root.bridgeStatus
            color: root.demoMode ? "#a86b11" : (root.rxAvailable ? "#18735f" : "#8c531a")
            wrapMode: Text.Wrap
            font.pixelSize: 12
        }
        Flow {
            Layout.fillWidth: true
            spacing: 4
            Repeater {
                model: root.favoriteCommands
                Button {
                    height: 32
                    text: modelData.name
                    enabled: root.pendingReportId < 0
                    onClicked: { commandInput.text = modelData.command; root.sendCommand(modelData.command); }
                    ToolTip.visible: hovered
                    ToolTip.text: modelData.command
                }
            }
        }
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 8
            Rectangle {
                Layout.preferredWidth: 195
                Layout.minimumWidth: 155
                Layout.fillHeight: true
                color: "#ffffff"
                border.color: "#d4dce2"
                radius: 3
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 7
                    Label { text: "历史报告 · " + reportsModel.count + " / 100"; font.bold: true; color: "#375366" }
                    ListView {
                        id: reportsList
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        model: reportsModel
                        spacing: 3
                        ScrollBar.vertical: ScrollBar {}
                        delegate: ItemDelegate {
                            width: reportsList.width
                            height: 63
                            highlighted: model.reportId === root.selectedReportId && root.selectedTab === 0
                            background: Rectangle { color: parent.highlighted ? "#e2eff5" : (parent.hovered ? "#f0f5f8" : "transparent"); radius: 3 }
                            contentItem: Column {
                                spacing: 4
                                Text { width: parent.width; text: model.command; color: "#213d50"; font.bold: true; font.pixelSize: 12; elide: Text.ElideRight }
                                Text { text: root.timeText(model.sentAt) + " · " + root.statusText(model.state); color: model.state === "complete" ? "#18735f" : "#687985"; font.pixelSize: 11 }
                            }
                            onClicked: root.selectReport(model.reportId)
                        }
                    }
                }
            }
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "white"
                border.color: "#d4dce2"
                radius: 3
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 6
                    RowLayout {
                        Layout.fillWidth: true
                        Button { text: "报告正文"; flat: root.selectedTab !== 0; onClicked: { root.selectedTab = 0; root.renderedKey = ""; root.refreshView(); } }
                        Button { id: otherTab; text: "主动消息 · 0"; flat: root.selectedTab !== 1; onClicked: { root.selectedTab = 1; root.renderedKey = ""; root.refreshView(); } }
                        Item { Layout.fillWidth: true }
                        Button { text: "复制"; onClicked: root.copyCurrent() }
                        Button { text: "导出"; onClicked: root.exportCurrent() }
                    }
                    Label { id: reportHeading; Layout.fillWidth: true; color: "#234357"; font.bold: true; wrapMode: Text.Wrap; font.pixelSize: 12 }
                    Label { id: warningLabel; Layout.fillWidth: true; visible: text.length > 0; color: "#986519"; wrapMode: Text.Wrap; font.pixelSize: 11 }
                    ListView {
                        id: transcript
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        model: linesModel
                        spacing: 3
                        boundsBehavior: Flickable.StopAtBounds
                        onMovementStarted: { if (!root.modelUpdating) root.followLatest = false; }
                        ScrollBar.vertical: ScrollBar { onPressedChanged: { if (pressed) root.followLatest = false; } }
                        delegate: Rectangle {
                            width: transcript.width
                            height: lineText.implicitHeight + 10
                            color: searchInput.text.length && model.body.toLowerCase().indexOf(searchInput.text.toLowerCase()) >= 0 ? "#fff4cf" :
                                (model.level === "error" ? "#fff0ee" : (model.uncertain ? "#fff9ed" : "#f7f9fb"))
                            radius: 2
                            TextEdit {
                                id: lineText
                                x: 6; y: 5
                                width: parent.width - 12
                                text: "[" + model.stamp + "] " + (model.uncertain ? "[归属待确认] " : "") + model.body
                                textFormat: TextEdit.PlainText
                                readOnly: true
                                selectByMouse: true
                                wrapMode: TextEdit.WrapAnywhere
                                color: model.level === "error" ? "#b83b32" : "#243d50"
                                selectionColor: "#aed2e6"
                                font.family: "Consolas, Microsoft YaHei"
                                font.pixelSize: 12
                            }
                        }
                        Label { anchors.centerIn: parent; visible: linesModel.count === 0; text: "等待报告。遥测不会进入正文。"; color: "#8999a4"; font.pixelSize: 12 }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        TextField { id: searchInput; Layout.fillWidth: true; placeholderText: "搜索正文"; selectByMouse: true; onAccepted: root.findNext() }
                        Button { text: "下一处"; onClicked: root.findNext() }
                        Button {
                            text: root.unseenLines > 0 ? "查看最新 · " + root.unseenLines : "查看最新"
                            onClicked: { root.followLatest = true; root.unseenLines = 0; transcript.positionViewAtEnd(); }
                        }
                    }
                }
            }
        }
        RowLayout {
            Layout.fillWidth: true
            TextField {
                id: commandInput
                Layout.fillWidth: true
                placeholderText: "输入单条调试命令，例如 PITCH CHECK"
                selectByMouse: true
                maximumLength: 63
                onAccepted: root.sendCommand(text)
                Keys.onUpPressed: { root.commandHistoryMove(-1); event.accepted = true; }
                Keys.onDownPressed: { root.commandHistoryMove(1); event.accepted = true; }
            }
            Button { text: root.demoMode ? "演示发送" : "发送"; enabled: root.pendingReportId < 0; onClicked: root.sendCommand(commandInput.text) }
            Button {
                text: "结束等待"
                enabled: root.pendingReportId > 0
                onClicked: root.stopWaiting()
                ToolTip.visible: hovered
                ToolTip.text: "只结束面板等待，不会停止单片机动作"
            }
        }
        Label {
            Layout.fillWidth: true
            text: root.notice || ("已隐藏数值遥测 " + root.hiddenTelemetry + " 行 · 保留非遥测文字 · " + (root.followLatest ? "跟随最新" : "阅读位置已锁定"))
            color: root.notice ? "#8c531a" : "#718593"
            wrapMode: Text.Wrap
            font.pixelSize: 11
        }
    }

    Dialog {
        id: settingsDialog
        parent: root
        x: Math.max(0, (root.width - width) / 2)
        y: Math.max(0, (root.height - height) / 2)
        width: Math.min(620, root.width - 30)
        height: Math.min(515, root.height - 25)
        modal: true
        title: "报告助手设置"
        closePolicy: Popup.CloseOnEscape
        contentItem: ColumnLayout {
            spacing: 6
            Label { text: "常用按钮：每行填写「按钮名称 | 单条命令」，最多 12 个"; Layout.fillWidth: true; wrapMode: Text.Wrap }
            ScrollView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                TextArea { id: favoritesEditor; selectByMouse: true; wrapMode: TextArea.NoWrap; font.pixelSize: 12 }
            }
            Label { text: "要隐藏的数值遥测组名：每行一个，不含冒号。只过滤完整数值 CSV。"; Layout.fillWidth: true; wrapMode: Text.Wrap }
            ScrollView {
                Layout.fillWidth: true
                Layout.preferredHeight: 80
                TextArea { id: groupsEditor; selectByMouse: true; font.pixelSize: 12 }
            }
            Label { text: "非数值文字、错误和未知报告始终保留。设置仅作用于此插件。"; font.pixelSize: 11; color: "#718593"; Layout.fillWidth: true; wrapMode: Text.Wrap }
            Label { id: settingsError; visible: text.length > 0; color: "#b83b32"; Layout.fillWidth: true; wrapMode: Text.Wrap }
            RowLayout {
                Item { Layout.fillWidth: true }
                Button { text: "取消"; onClicked: settingsDialog.close() }
                Button { text: "保存设置"; onClicked: root.applySettings() }
            }
        }
    }
}
