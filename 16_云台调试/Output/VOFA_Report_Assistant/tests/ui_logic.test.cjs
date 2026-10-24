/* Exercises the actual QML JavaScript methods without starting VOFA or a port.
 * This is not a Qt rendering test; host integration needs the separate probe.
 */
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const widget = path.join(__dirname, '../plugins/widgets/report_assistant');
const source = fs.readFileSync(path.join(widget, 'report_assistant.qml'), 'utf8');
const functions = source.match(/^    function [\s\S]*?^    }/gm);
assert(functions && functions.length > 20, 'QML logic functions found');
let cases = 0;
function fixture() {
    const context = vm.createContext({console});
    vm.runInContext(fs.readFileSync(path.join(widget, 'ReportEngine.js'), 'utf8'), context);
    vm.runInContext(`
        var ReportEngine = { createEngine: createEngine };
        var reportEngine = createEngine({maxReports:100});
        var activeStage = true, rxAvailable = true, sendAvailable = true, demoMode = false;
        var notice = '', selectedReportId = -1, selectedTab = 0, followLatest = true;
        var unseenLines = 0, hiddenTelemetry = 0, pendingReportId = -1, commandHistoryIndex = -1;
        var commandHistory = [], favoriteCommands = [{name:'状态',command:'PITCH STATUS'}];
        var realStateBeforeDemo = '', demoChunks = [], demoChunkIndex = 0;
        var renderedLineCount = 0, renderedKey = '', modelUpdating = false;
        function listModel() {
            return {items:[], get count(){return this.items.length;},
                get:function(i){return this.items[i];}, append:function(v){this.items.push(v);},
                clear:function(){this.items=[];}, remove:function(i){this.items.splice(i,1);},
                setProperty:function(i,k,v){this.items[i][k]=v;}};
        }
        var reportsModel = listModel(), linesModel = listModel();
        var transcript = {contentY:0, currentIndex:-1, indexAt:function(){return -1;},
            itemAtIndex:function(){return null;}, positionViewAtEnd:function(){this.latest=true;},
            positionViewAtIndex:function(i){this.position=i;}};
        var reportHeading = {}, warningLabel = {}, otherTab = {};
        var refreshTimer = {running:false,start:function(){this.running=true;}};
        var saveTimer = {running:false,start:function(){this.running=true;}};
        var demoTimer = {start:function(){},stop:function(){}};
        var commandInput = {text:'',forceActiveFocus:function(){}};
        var searchInput = {text:''}, settingsError = {text:''};
        var favoritesEditor = {text:''}, groupsEditor = {text:''};
        var settingsDialog = {open:function(){},close:function(){this.closed=true;}};
        var Qt = {callLater:function(f){f();}}, ListView = {Contain:0,Beginning:1};
        var HostBridge = {sendCommand:function(){return {ok:true};},
            readState:function(){return '';},writeState:function(){return {ok:true};},
            attach:function(){},copyText:function(){return {ok:true};},
            exportText:function(){return {ok:true,path:'demo.txt'};}};
    ` + functions.join('\n'), context);
    return context;
}
function run(title, code) {
    const ctx = fixture();
    vm.runInContext(code, ctx);
    cases++;
    console.log('PASS ' + title);
}
run('synchronous reply belongs to a report created before TX', `
    HostBridge.sendCommand = function() { feedRx('P|PID|POS|1,2,3|VEL|4,5,6\\r\\n', now()); return {ok:true}; };
    sendCommand('PITCH PID');
    if (reportEngine.reports.length !== 1 || reportEngine.reports[0].status !== 'complete') throw Error('lost synchronous reply');
`);
run('TX failure releases pending wait and records error', `
    HostBridge.sendCommand = function(){return {ok:false,error:'offline'};};
    sendCommand('PITCH PID');
    if (reportEngine.pendingId !== null || reportEngine.reports[0].status !== 'error') throw Error('pending left stuck');
`);
run('Enter cannot send a second command while waiting', `
    var sent=0; HostBridge.sendCommand=function(){sent++;return {ok:true};};
    sendCommand('PITCH CHECK'); sendCommand('PITCH PID');
    if(sent !== 1 || reportEngine.reports.length !== 1) throw Error('concurrent command sent');
`);
run('unknown RX capability prevents transmission', `
    var sent=0; rxAvailable=false; HostBridge.sendCommand=function(){sent++;return {ok:true};};
    sendCommand('PITCH STATUS');
    if(sent || reportEngine.reports.length) throw Error('sent without receive capability');
`);
run('multiline input is rejected before transport', `
    var sent=0; HostBridge.sendCommand=function(){sent++;return {ok:true};};
    sendCommand('PITCH PID\\nPITCH STATUS'); if(sent) throw Error('multiline send');
`);
run('telemetry does not create text delegates or trigger refresh', `
    feedRx('pitch:1,2,3\\r\\n', now());
    if(refreshTimer.running || linesModel.count || reportEngine.stats.telemetryLines!==1) throw Error('telemetry rendered');
`);
run('reader position does not follow new lines', `
    selectedTab=1; followLatest=false; transcript.contentY=42;
    reportEngine.feed('first\\r\\n',now()); refreshView();
    reportEngine.feed('second\\r\\n',now()); refreshView();
    if(transcript.latest || transcript.contentY!==42 || unseenLines!==2) throw Error('reader jumped');
`);
run('bounded events remove only old prefix and append new text', `
    reportEngine=createEngine({maxEvents:2}); selectedTab=1;
    reportEngine.feed('one\\r\\ntwo\\r\\n',now()); refreshView();
    reportEngine.feed('three\\r\\n',now()); refreshView();
    if(linesModel.count!==2 || linesModel.get(0).body!=='two' || linesModel.get(1).body!=='three') throw Error('wrong incremental model');
`);
run('demo never calls real transport and restores real history', `
    reportEngine.start('PITCH PID',now()); reportEngine.feed('P|PID|POS|1,2,3\\r\\n',now());
    var sent=0; HostBridge.sendCommand=function(){sent++;return {ok:true};};
    setDemo(true); sendCommand('PITCH CHECK');
    for(var i=0;i<demoChunks.length;i++) reportEngine.feed(demoChunks[i],now());
    if(sent || reportEngine.reports[0].status!=='complete') throw Error('bad demo transport/matching');
    setDemo(false);
    if(reportEngine.reports.length!==1 || reportEngine.reports[0].command!=='PITCH PID') throw Error('real history lost');
`);
run('command history bounded and navigable', `
    for(var i=0;i<105;i++) rememberCommand('PITCH QUERY '+i);
    if(commandHistory.length!==100 || commandHistory[0]!=='PITCH QUERY 5') throw Error('unbounded history');
    commandHistoryMove(-1); if(commandInput.text!=='PITCH QUERY 104') throw Error('up history');
    commandHistoryMove(1); if(commandInput.text!=='') throw Error('down history');
`);
run('invalid favorite does not silently truncate into another command', `
    favoritesEditor.text='Long | '+new Array(70).join('x'); groupsEditor.text='pitch';
    applySettings(); if(!settingsError.text || settingsDialog.closed) throw Error('accepted oversized favorite');
`);
run('all six default Pitch demo commands reach their accurate terminal outcome', `
    var commands=['PITCH STATUS','PITCH PID','PITCH CHECK','PITCH RESULT','PITCH GUARD','PITCH HELP'];
    var sent=0;HostBridge.sendCommand=function(){sent++;return {ok:true};};
    setDemo(true);
    for(var c=0;c<commands.length;c++) {
        sendCommand(commands[c]);
        for(var i=0;i<demoChunks.length;i++) reportEngine.feed(demoChunks[i],now());
        reportEngine.tick(now()+5000);
        var report=reportEngine.reports[reportEngine.reports.length-1];
        var expected=commands[c]==='PITCH RESULT'||commands[c]==='PITCH GUARD'?'uncertain':'complete';
        if(report.status!==expected)throw Error(commands[c]+' got '+report.status+' expected '+expected);
    }
    if(sent)throw Error('demo called real TX');
`);
console.log(cases + ' UI logic scenarios passed. Qt rendering/host RX remain separate checks.');
