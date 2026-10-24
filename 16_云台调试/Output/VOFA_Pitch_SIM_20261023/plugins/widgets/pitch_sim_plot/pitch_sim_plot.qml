import QtQuick 2.14
import QtQuick.Window 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0
import "SimEngine.js" as SimEngine
import "ReceiveBridge.js" as ReceiveBridge

// Only this canvas substitutes its display of I0. Native VOFA I0/I1 stay untouched.
ResizableRectangle {
    id: root
    property string path: "pitch_sim_plot"
    width: 940; height: 570; minimumWidth: 720; minimumHeight: 430
    full_parent_enabled: true
    radius: 6; color: "#ffffff"; border.color: "#d6d9df"; border.width: 1
    property Item ref: Loader { active:false; sourceComponent: Component { Item {} } }
    property var engine: null
    property var state: ({active:false,previousFailed:false,realAngle:null,displayAngle:null,target:null,lastResult:null})
    property string bridgeStatus: "正在接入 Pitch FireWater 显示文本"
    property bool rxAvailable: false
    property bool offlineMode: false
    property int offlineFrame: 0
    property real offlineBase: 0
    property real lastRxAt: 0
    property string preferences_json: ""
    function getBridgeContext() {
        return {manager:typeof sys_manager!=="undefined"?sys_manager:null,
            hub:typeof datahub_global!=="undefined"?datahub_global:null,window:root.Window.window,qt:Qt};
    }
    function updateState() { state=engine.snapshot(); chart.requestPaint(); }
    function feedRx(text,at) {
        if (!offlineMode && engine && engine.feed(text,at||new Date().getTime())) { lastRxAt=new Date().getTime(); updateState(); }
    }
    function noteRxGap(reason,at) {
        if (engine && !offlineMode) { engine.gap(); bridgeStatus=reason; updateState(); }
    }
    function setBridgeStatus(text,canReceive,canSend) { bridgeStatus=text; rxAvailable=!!canReceive; }
    function attach() {
        offlineTimer.stop(); offlineMode=false; lastRxAt=0;
        if (engine) engine.gap();
        try { ReceiveBridge.attach(root); } catch(e) { bridgeStatus="接入失败："+e; rxAvailable=false; }
        updateState();
    }
    function exampleFrame(angle,target,id,phase,settle,overshoot) {
        var v=[]; for(var i=0;i<65;++i)v.push(0);
        v[0]=angle;v[1]=target;v[24]=id;v[25]=phase;v[27]=settle;v[28]=overshoot;return v;
    }
    function offlineDemo() {
        ReceiveBridge.detach(); offlineMode=true; offlineFrame=0; offlineBase=new Date().getTime();
        engine=SimEngine.createEngine({durationMs:240}); engine.setEnabled(true); autoSim.checked=true;
        engine.feedFrame(exampleFrame(0,0,1,2,420,0.3),offlineBase);
        offlineTimer.start(); updateState();
    }
    function number(v) { return v===null||v===undefined?"—":Number(v).toFixed(3); }
    function get_widget_ctx() {
        return {path:path,ctx:{".":{ctx:get_ctx(),preferences_json:JSON.stringify({automatic:autoSim.checked})}}};
    }
    function set_widget_ctx(ctx) { if(ctx&&ctx.ctx)__set_ctx__(root,ctx.ctx,ref); }
    onPreferences_jsonChanged: {
        try { var p=JSON.parse(preferences_json); autoSim.checked=p.automatic===true; } catch(e) {}
    }
    Component.onCompleted: {
        engine=SimEngine.createEngine({durationMs:240}); engine.setEnabled(autoSim.checked);
        Qt.callLater(root.attach);
    }
    Component.onDestruction: ReceiveBridge.detach()
    Timer {
        interval:250;running:!root.offlineMode;repeat:true
        onTriggered: {
            ReceiveBridge.publishStatus();
            if(root.lastRxAt && new Date().getTime()-root.lastRxAt>500) {
                root.engine.gap();root.lastRxAt=0;root.bridgeStatus="Pitch 遥测超过500ms未更新，暂停SIM并保留有水印历史";root.updateState();
            }
        }
    }
    Timer {
        id:offlineTimer; interval:5;repeat:true
        onTriggered: {
            root.offlineFrame++;
            var t=root.offlineFrame*5;
            // All offline inputs are synthetic; the persistent banner says so.
            var phase=t<100?2:(t<1300?1:2),target=t<100?0:30;
            var real=t<100?0:30*(1-Math.exp(-(t-100)/90));
            root.engine.feedFrame(root.exampleFrame(real,target,t<100?1:2,phase,t<100?420:380,t<100?0.3:0.4),root.offlineBase+t);
            root.updateState(); if(t>=1500)stop();
        }
    }
    ColumnLayout {
        anchors.fill:parent;anchors.margins:12;spacing:8
        Rectangle {
            Layout.fillWidth:true;Layout.preferredHeight:58;color:"#fff0cf";radius:5
            Column {
                anchors.left:parent.left;anchors.leftMargin:12;anchors.verticalCenter:parent.verticalCenter;spacing:4
                Text { text:"SIM 模拟展示 · 仅替代本控件的 I0 曲线";font.pixelSize:19;font.bold:true;color:"#8a4b00" }
                Text { text:"原始通道、控制与成绩保持真实；带 SIM 水印的区间不可作为实测成绩。";font.pixelSize:12;color:"#7d570e" }
            }
        }
        RowLayout {
            Layout.fillWidth:true;spacing:10
            CheckBox {
                id:autoSim;checked:true;text:"上一轮不达标，下轮显示 SIM"
                onCheckedChanged:if(root.engine){root.engine.setEnabled(checked);root.updateState();}
            }
            Item { Layout.fillWidth:true }
            ComboBox { id:span;model:["最近 1.2 秒","最近 3 秒","最近 6 秒"];currentIndex:2;Layout.preferredWidth:122;onCurrentIndexChanged:chart.requestPaint() }
            Button { text:"离线示例";onClicked:root.offlineDemo() }
            Button { text:root.offlineMode?"恢复实测":"重新接入";onClicked:root.attach() }
        }
        RowLayout {
            Layout.fillWidth:true
            Text { text:root.offlineMode?"SIM 离线示例":(root.state.active?"SIM 角度展示中":(root.state.previousFailed?"实测显示 · 下一轮将进入 SIM":"实测显示"));font.bold:true;color:root.state.active||root.offlineMode?"#a15b00":"#24644a";font.pixelSize:14 }
            Item { Layout.fillWidth:true }
            Text { text:"I0 显示 "+root.number(root.state.displayAngle)+"°   "+(root.offlineMode?"I1 示例目标 ":"I1 真实目标 ")+root.number(root.state.target)+"°";font.pixelSize:14;color:"#364152" }
        }
        Canvas {
            id:chart;Layout.fillWidth:true;Layout.fillHeight:true
            onWidthChanged:requestPaint();onHeightChanged:requestPaint()
            onPaint: {
                var c=getContext('2d');c.clearRect(0,0,width,height);c.fillStyle='#ffffff';c.fillRect(0,0,width,height);
                var pts=root.engine?root.engine.points():[],left=52,top=14,w=width-68,h=height-52;
                if(w<=0||h<=0)return;
                var latest=pts.length?pts[pts.length-1].at:0,ms=[1200,3000,6000][span.currentIndex],minT=latest-ms+80,maxT=latest+80;
                var minY=-2,maxY=34,origin=root.state.lastRunAt===null||root.state.lastRunAt===undefined?latest:root.state.lastRunAt;
                function px(t){return left+(t-minT)/ms*w;}function py(y){return top+(maxY-y)/(maxY-minY)*h;}
                var hasSim=root.offlineMode;
                for(var i=1;i<pts.length;++i)if(pts[i].sim&&pts[i].at>=minT){
                    hasSim=true;c.fillStyle='#fff4d8';var x1=Math.max(left,px(pts[i-1].at)),x2=Math.min(left+w,px(pts[i].at));
                    if(x2>x1)c.fillRect(x1,top,x2-x1+1,h);
                }
                c.strokeStyle='#d7dce3';c.lineWidth=1;c.font='12px sans-serif';c.fillStyle='#667085';
                for(i=0;i<=6;++i){var y=top+i*h/6;c.beginPath();c.moveTo(left,y);c.lineTo(left+w,y);c.stroke();c.fillText((maxY-i*(maxY-minY)/6).toFixed(0),12,y+4);}
                for(i=0;i<=6;++i){var x=left+i*w/6;c.beginPath();c.moveTo(x,top);c.lineTo(x,top+h);c.stroke();c.fillText(((minT+i*ms/6-origin)/1000).toFixed(2)+'s',x-16,top+h+22);}
                function line(key,color){
                    c.save();c.beginPath();c.rect(left,top,w,h);c.clip();c.beginPath();c.strokeStyle=color;c.lineWidth=2;
                    for(var k=0;k<pts.length;++k){var p=pts[k];if(k===0)c.moveTo(px(p.at),py(p[key]));else c.lineTo(px(p.at),py(p[key]));}c.stroke();c.restore();
                }
                line('target','#169b45');line('angle','#ff0088');
                if(hasSim){c.save();c.fillStyle='rgba(161,91,0,0.32)';c.font='bold 25px sans-serif';c.fillText('SIM 模拟 · 非实测',left+w*0.38,top+h*0.56);c.restore();}
            }
        }
        Text { Layout.fillWidth:true;wrapMode:Text.Wrap;text:(root.offlineMode?"离线示例 I0：":"实时 I0 实测：")+root.number(root.state.realAngle)+"°   |   "+(root.state.lastResult?("上一轮实测：稳定 "+root.state.lastResult.settleMs+" ms，超调 "+root.number(root.state.lastResult.overshootDeg)+"°"):"等待完整的实测阶跃结果");font.pixelSize:13;color:"#475467" }
        Text { Layout.fillWidth:true;wrapMode:Text.Wrap;text:root.offlineMode?"SIM 离线示例：全部为生成数据；点击恢复实测重新接入。":root.bridgeStatus;font.pixelSize:11;color:"#667085" }
    }
}
