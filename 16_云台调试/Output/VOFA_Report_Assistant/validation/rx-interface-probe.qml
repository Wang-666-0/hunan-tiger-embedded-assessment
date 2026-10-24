import QtQuick 2.12
import QtQuick.Window 2.12
import MyModules 1.0

ResizableRectangle {
    id: root
    property string path: "rx_probe"
    width: 160
    height: 80
    minimumWidth: 160
    minimumHeight: 80
    color: "#e5f2ee"
    property string logText: ""
    property int checks: 0
    function line(s) { logText += String(s) + "\n"; }
    function dump(label, obj) {
        line(label + " = " + String(obj));
        if (!obj) return;
        try {
            var keys = Object.keys(obj);
            for (var i = 0; i < keys.length; ++i) {
                var k = keys[i];
                try { line("  " + k + " : " + typeof obj[k] + " : " + String(obj[k]).slice(0, 180)); }
                catch (e) { line("  " + k + " error " + e); }
            }
        } catch (e) { line("keys error " + e); }
    }
    function findEdit(start, doc) {
        var todo=[start], seen=[], steps=0;
        while(todo.length && steps++<3000) {
            var o=todo.shift(); if(!o || seen.indexOf(o)>=0) continue; seen.push(o);
            try { if(o.textDocument) { line("EDIT candidate="+o+" doc="+o.textDocument+" expected="+doc); if(o.textDocument===doc || String(o.textDocument)===String(doc)) return o; } } catch(e) {}
            try { for(var i=0;i<o.children.length;i++) todo.push(o.children[i]); } catch(e) {}
        }
        line("SCAN count="+steps);
        return null;
    }
    function runProbe() {
        line("RX interface probe; no serial operations");
        try { dump("sys_manager", sys_manager); } catch (e) { line(e); }
        try { dump("datahub_global", datahub_global); dump("text_buffer", datahub_global.text_buffer); } catch (e) { line(e); }
        try { line("myapp.context_dir_path=" + myapp.context_dir_path()); } catch (e) { line(e); }
        try { line("myapp.config_dir_path=" + myapp.config_dir_path()); line("layoutfile=" + myapp.get_config_file("vofa+.tabviews.json")); dump("myapp",myapp); } catch (e) { line(e); }
        try { line("all_text=" + datahub_global.text_buffer.all_text()); } catch (e) { line(e); }
        try { datahub_global.text_message_updated.connect(function() { line("SIGNAL argc="+arguments.length+" text="+JSON.stringify(datahub_global.text_buffer.text)+" updated="+datahub_global.text_buffer.updated); saveLog(); }); } catch (e) { line("signal "+e); }
        try { datahub_global.PutErrorData("PROBE_ERROR_ONLY|status\nP|STATUS|QUERY|SNAPSHOT_ONLY\n"); line("after PutErrorData text="+JSON.stringify(datahub_global.text_buffer.text)); } catch (e) { line("local error injection "+e); }
        try { dump("manager-parent",sys_manager.parent); dump("attached-window",root.Window.window); } catch(e) { line(e); }
        try {
            var refs=[sys_manager.parent.ref,root.Window.window.ref,root.Window.window.ref_new_window];
            for(var r=0;r<refs.length;r++) if(refs[r]) { var active=refs[r].active; refs[r].active=true; dump("REF "+r,refs[r].item); refs[r].active=active; }
        } catch(e) { line("REF error="+e); }
        try {
            var l=sys_manager.parent.ref, a=l.active; l.active=true; var zone=l.item.ref_tx_rx_zone; l.active=a;
            dump("RXZONE",zone);
            var zr=zone.ref; if(zr) { var za=zr.active; zr.active=true; dump("ZONEREF",zr.item); zr.active=za; }
            var ze=findEdit(zone,datahub_global.text_buffer.doc); line("ZONE EDIT="+ze);
            if(ze) { ze.textChanged.connect(function(){line("ZONE CHANGED raw="+JSON.stringify(ze.getText(0,ze.length)));saveLog();}); ze.insert(ze.length,"P|STATUS|QUERY|SNAPSHOT_ONLY\n"); }
        } catch(e) {line("ZONE error="+e);}
        try {
            var hub=datahub_global, manager=sys_manager;
            var top=manager.parent; for(var k=0;k<8 && top.parent;k++) top=top.parent;
            var edit=findEdit(top,hub.text_buffer.doc);
            line("RX EDIT="+edit+" length="+(edit?edit.length:-1));
            if(edit) {
                line("RX ORIGINAL="+JSON.stringify(edit.getText(0,edit.length)));
                edit.textChanged.connect(function(){ line("RX EDIT CHANGED length="+edit.length+" raw="+JSON.stringify(edit.getText(0,edit.length))); saveLog(); });
                edit.insert(edit.length,"P|STATUS|QUERY|SNAPSHOT_ONLY\n");
                edit.insert(edit.length,"pitch:1,2,3\nR|RX|EPOCH|1\nP|CHECK|QUERY|NEW_FEEDBACK\n");
            }
        } catch(e) { line("RX EDIT lookup="+e); }
        saveLog();
    }
    function saveLog() {
        try {
            var f = sys_manager.file_reader;
            var p = f.applicationDirPath() + "/rx-probe-" + String(root).replace(/[^0-9a-z]/gi,"_") + ".txt";
            var old = f.source;
            f.source = p;
            var ok = f.write(logText);
            f.source = old;
            console.log("PROBE WRITE " + ok + " " + p);
        } catch (e) { console.log("PROBE ERROR " + e + "\n" + logText); }
    }
    Component.onCompleted: runProbe()
    Timer {
        interval: 400; running: true; repeat: true
        onTriggered: {
            root.checks++;
            try {
                root.line("TICK="+root.checks+" text="+JSON.stringify(datahub_global.text_buffer.text)+" updated="+datahub_global.text_buffer.updated);
                if(root.checks===1) {
                    var top=sys_manager.parent; for(var k=0;k<8 && top.parent;k++)top=top.parent;
                    root.line("LATER EDIT="+findEdit(top,datahub_global.text_buffer.doc));
                }
                if (root.checks === 2) { root.line("all_text ret="+datahub_global.text_buffer.all_text()); root.line("AFTER all_text text="+JSON.stringify(datahub_global.text_buffer.text)); }
                if (root.checks === 3) datahub_global.PutErrorData("P|CHECK|QUERY|NEW_FEEDBACK\n");
                root.saveLog();
            } catch(e) { root.line("tick error="+e); }
            if (root.checks >= 8) stop();
        }
    }
    Text { anchors.centerIn: parent; text: "RX probe / offline" }
    function get_widget_ctx() { return {path:path,ctx:{'.':{ctx:get_ctx()}}}; }
    function set_widget_ctx(ctx) { __set_ctx__(root,ctx.ctx); }
}
