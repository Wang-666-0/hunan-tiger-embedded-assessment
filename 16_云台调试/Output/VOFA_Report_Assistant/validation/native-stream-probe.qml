import QtQuick 2.12
import QtQuick.Window 2.12
import MyModules 1.0
import mine.qt.DebugClient 1.0
ResizableRectangle {
    id: root
    property string path: "rx_probe"
    width: 740; height: 480; minimumWidth: 500; minimumHeight: 300
    color: "#edf3ef"
    property var hub: null
    property var receiver: null
    property var samples: []
    property int notifications: 0
    property int nativeMessages: 0
    property int beforeHead: -1
    property var clientMessages: []
    property string error: ""
    property var assistant: null
    property var loaderInfo: ({})
    DebugClient { id: client; serverAddr: "127.0.0.1"; port: 39875 }
    function refs(o) {
        if(!o || !o.ref) return null;
        var l=o.ref, old=l.active; l.active=true; var item=l.item;
        var result={};
        if(item) { var keys=Object.keys(item); for(var i=0;i<keys.length;i++)if(keys[i].indexOf("ref_")===0)result[keys[i]]=item[keys[i]]; }
        l.active=old; return result;
    }
    function findEdit(o, doc) {
        var stack=[o],count=0,seen=[];
        while(stack.length && count++<1200) {
            var item=stack.pop(); if(!item || seen.indexOf(item)>=0)continue; seen.push(item);
            try { if(item.textDocument && String(item.textDocument)===String(doc))return item; }catch(e){}
            try { for(var i=0;i<item.children.length;i++)stack.push(item.children[i]); }catch(e){}
        }
        return null;
    }
    function initialize() {
        try {
            hub=datahub_global;
            var r=refs(sys_manager.parent);
            try {
                var targets=[r.ref_settingsPanel], settings=sys_manager.settings_obj;
                for(var si=0;settings && si<settings.count;si++)targets.push(settings.itemAt(si));
                loaderInfo.settings=[];
                for(var ti=0;ti<targets.length;ti++) {
                    var item=targets[ti],entry={object:String(item),refs:{}},properties=Object.keys(item||{});
                    for(var pi=0;pi<properties.length;pi++)if(/combo|protocol|engine|ref|index|text/i.test(properties[pi]))try{entry[properties[pi]]=String(item[properties[pi]]);}catch(e){}
                    var reference=refs(item);for(var key in reference)entry.refs[key]=String(reference[key]);
                    loaderInfo.settings.push(entry);
                }
            }catch(e){loaderInfo.settingsError=String(e);}
            try {
                var scan=[root.Window.window.contentItem,sys_manager.parent],seen=[],steps=0;
                loaderInfo.combos=[];
                while(scan.length && steps++<10000) {
                    var object=scan.pop();if(!object || seen.indexOf(object)>=0)continue;seen.push(object);
                    if(typeof object.currentText==='string') loaderInfo.combos.push({object:String(object),currentText:object.currentText,count:object.count});
                    if(object.children)for(var ci=0;ci<object.children.length;ci++)scan.push(object.children[ci]);
                }
                loaderInfo.scanCount=steps;
            }catch(e){loaderInfo.scanError=String(e);}
            receiver=findEdit(r.ref_tx_rx_zone, hub.text_buffer.doc);
            if(!receiver)throw Error("No matching RX document");
            beforeHead=hub.head;
            var keys=Object.keys(hub.data_engine_loader);
            for(var k=0;k<keys.length;k++)try { loaderInfo[keys[k]]=String(hub.data_engine_loader[keys[k]]); }catch(e){}
            try{loaderInfo.getNames=hub.data_engine_loader.name_list();}catch(e){loaderInfo.getNamesError=String(e);}
            try{loaderInfo.getFiles=hub.data_engine_loader.file_list();}catch(e){loaderInfo.getFilesError=String(e);}
            var component=Qt.createComponent("../report_assistant/report_assistant.qml");
            if(component.status!==Component.Ready)throw Error(component.errorString());
            assistant=component.createObject(root,{x:0,y:0,width:740,height:480});
            if(!assistant)throw Error(component.errorString());
            assistant.activate();
            var command=assistant.reportEngine.start("PITCH CHECK",new Date().getTime());
            assistant.selectedReportId=command.id;
            assistant.scheduleRefresh();
            receiver.textChanged.connect(function(){ notifications++; Qt.callLater(root.capture); });
            hub.text_message_updated.connect(function(){nativeMessages++;});
            client.datahub=hub;
            client.newStringCome.connect(function(data){clientMessages.push(String(data));});
            hub.start_paint();
            client.open();
            capture(); save();
        } catch(e){error=String(e);save();}
    }
    function capture() {
        if(!receiver)return;
        var value=receiver.getText(0,receiver.length);
        if(assistant && hub.head>=400 && assistant.reportEngine.pendingId===null && assistant.reportEngine.reports.length===1) {
            var next=assistant.reportEngine.start("PITCH PID",new Date().getTime());
            assistant.selectedReportId=next.id;assistant.scheduleRefresh();
        }
        if(!samples.length || samples[samples.length-1].text!==value) {
            samples.push({time:new Date().getTime(),text:value,docLength:receiver.length,rawLength:value.length,
                tail:receiver.getText(Math.max(0,receiver.length-180),receiver.length),lastFormatted:receiver.getFormattedText(Math.max(0,receiver.length-200),receiver.length)});
            if(samples.length>12)samples.shift();
        }
    }
    function save() {
        try {
            var f=sys_manager.file_reader,old=f.source;
            f.source=f.applicationDirPath()+"/native-stream.json";
            f.write(JSON.stringify({error:error,receiver:String(receiver),connected:client.connected,
                headBefore:beforeHead,headAfter:hub?hub.head:-1,values:hub?hub.rt_values():[],
                notifications:notifications,nativeMessages:nativeMessages,samples:samples,clientMessages:clientMessages,bufferUpdated:hub?hub.text_buffer.updated:false,
                loaderInfo:loaderInfo,assistant:assistant?{status:assistant.bridgeStatus,rx:assistant.rxAvailable,send:assistant.sendAvailable,notice:assistant.notice,engine:assistant.reportEngine.serialize()}:null}));
            f.source=old;
        }catch(e){console.log(e);}
    }
    Timer { interval: 1500; running:true; repeat:false; onTriggered:root.initialize() }
    Timer { interval: 500; running:true; repeat:true; onTriggered:root.save() }
    Text { anchors.centerIn:parent; text:root.error || "Native RX test / localhost only" }
    function get_widget_ctx(){return {path:path,ctx:{'.':{ctx:get_ctx()}}};}
    function set_widget_ctx(ctx){__set_ctx__(root,ctx.ctx);}
}
