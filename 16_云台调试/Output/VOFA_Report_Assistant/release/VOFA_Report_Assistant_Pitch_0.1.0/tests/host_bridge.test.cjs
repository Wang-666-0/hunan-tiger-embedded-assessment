/* No VOFA process and no COM port. Host API mocks throw if consuming RX getters
 * or host-document mutations are attempted. Actual runtime route is a separate
 * integration check, not established by these tests.
 */
const fs = require('fs'), path = require('path'), vm = require('vm'), assert = require('assert');
const code = fs.readFileSync(path.join(__dirname,'../plugins/widgets/report_assistant/HostBridge.js'),'utf8');
let passed = 0;
function signal() {
    const listeners=[];
    return {listeners,connect(f){listeners.push(f);},disconnect(f){const n=listeners.indexOf(f);if(n>=0)listeners.splice(n,1);},
        emit(){listeners.slice().forEach(f=>f());}};
}
function makeDoc(text='') {
    let contents=text;
    const edit={textDocument:{},children:[],textChanged:signal(),reads:[],get length(){return contents.length;},
        getText(a,b){this.reads.push([a,b]);return contents.slice(a,b);},
        insert(){throw Error('host insert forbidden');},remove(){throw Error('host remove forbidden');},
        setText(s){contents=s;this.textChanged.emit();},append(s){contents+=s;this.textChanged.emit();}};
    Object.defineProperty(edit,'text',{get(){throw Error('host full text getter forbidden');},set(){throw Error('host full text mutation forbidden');}});
    return edit;
}
function fixture(initial='') {
    const context=vm.createContext({console}); vm.runInContext(code,context);
    const edit=makeDoc(initial), flags={if_show_hex:false,if_show_echo:true,if_show_hex_changed:signal(),doc:edit.textDocument};
    Object.defineProperty(flags,'text',{get(){throw Error('CONSUMING text_buffer.text forbidden');}});
    flags.all_text=()=>{throw Error('all_text forbidden');};
    const combo={count:3,currentText:'FireWater',currentTextChanged:signal(),textAt(i){return ['FireWater','JustFloat','RawData'][i];},children:[]};
    const hub={text_buffer:flags,text_message_updated:signal(),data_engine_loader:{name_list(){return ['FireWater','JustFloat','RawData'];}}};
    const files=new Map(),dirs=new Set();let writes=0;
    const reader={source:'original/native/file.json',fail:false, applicationDirPath(){return 'D:/VOFA+/x64';},
        exist(p){return files.has(p)||dirs.has(p);},mkdir(p){dirs.add(p);return true;},
        read(){if(this.fail)throw Error('read fail');return files.get(this.source)||'';},
        write(s){if(this.fail)throw Error('write fail');writes++;files.set(this.source,s);return true;}};
    const manager={parent:{children:[edit,combo]},connected:true,connectedChanged:signal(),file_reader:reader,sent:[],
        send_string(s){this.sent.push(s);}};
    const root={chunks:[],gaps:[],statuses:[],getBridgeContext(){
            return {manager,hub,qt:{
                createQmlObject(qml,parent,name){
                    return {text:'',selectAll(){},copy(){root.copied=this.text;},destroy(){}};
                }
            }};
        },
        feedRx(s){this.chunks.push(s);},noteRxGap(s){this.gaps.push(s);},
        setBridgeStatus(...args){this.statuses.push(args);}};
    return {context,edit,combo,flags,hub,manager,root,reader,files,dirs,get writes(){return writes;}};
}
function test(name,action){action();passed++;console.log('PASS '+name);}
test('manual attach reads a bounded baseline, never consumes RX or changes host',()=>{
    const f=fixture('pre-existing report\n');f.context.attach(f.root);
    assert.equal(f.root.chunks.length,0);assert.equal(f.manager.sent.length,0);assert.equal(f.writes,0);
    assert.equal(f.flags.if_show_hex,false);assert.equal(f.flags.if_show_echo,true);
    f.edit.append('P|PID|POS|1,2,3\n');assert.equal(f.root.chunks.join(''),'P|PID|POS|1,2,3\n');
    assert.equal(f.root.statuses.at(-1)[1],true);
});
test('split text updates are incremental and retain arbitrary unknown text',()=>{
    const f=fixture();const decoder=f.context.createPresentationDecoder({text:t=>f.root.chunks.push(t)});
    decoder.feed('P|CH');decoder.batchEnd();decoder.feed('ECK|ok\npitch:1,2\nR|RX|1\n');
    assert.equal(f.root.chunks.join(''),'P|CHECK|ok\npitch:1,2\nR|RX|1\n');assert.equal(f.root.gaps.length,0);
});
test('Qt paragraph separators become newlines',()=>{
    const f=fixture();f.context.attach(f.root);f.edit.append('P|STATUS|ok\u2029next\u2028');
    assert.equal(f.root.chunks.join(''),'P|STATUS|ok\nnext\n');
});
test('unchanged text notifications do not duplicate data',()=>{
    const f=fixture();f.context.attach(f.root);f.edit.append('abc\n');f.edit.textChanged.emit();
    assert.equal(f.root.chunks.join(''),'abc\n');
});
test('rolling buffer suffix overlap yields only new bytes',()=>{
    const f=fixture('obsolete\nold1\nold2\n');f.context.attach(f.root);
    f.edit.setText('old1\nold2\nnew\n');assert.equal(f.root.chunks.join(''),'new\n');
});
test('same-length replacement is detected rather than mistaken for no data',()=>{
    const f=fixture('old report\n');f.context.attach(f.root);f.edit.setText('new report\n');
    assert.equal(f.root.chunks.join(''),'new report\n');assert.equal(f.root.gaps.length,1);
});
test('clear announces gap and subsequent appends remain observable',()=>{
    const f=fixture('old report\n');f.context.attach(f.root);f.edit.setText('');f.edit.append('new\n');
    assert.equal(f.root.chunks.join(''),'new\n');assert.equal(f.root.gaps.length,1);
});
test('large burst is bounded, marked incomplete, and clipped partial line discarded',()=>{
    const f=fixture();f.context.attach(f.root);f.edit.append('x'.repeat(70000)+'\nP|CHECK|ok\n');
    assert.equal(f.root.chunks.join(''),'P|CHECK|ok\n');assert.equal(f.root.gaps.length,1);
    assert(f.edit.reads.every(([a,b])=>b-a<=65536));
});
test('detach disconnects every signal and blocks late callbacks',()=>{
    const f=fixture();f.context.attach(f.root);f.context.detach();f.edit.append('late\n');
    assert.equal(f.root.chunks.length,0);assert.equal(f.edit.textChanged.listeners.length,0);
    assert.equal(f.manager.connectedChanged.listeners.length,0);assert.equal(f.hub.text_message_updated.listeners.length,0);
});
test('reattach does not duplicate signals or consume old document',()=>{
    const f=fixture();f.context.attach(f.root);f.context.attach(f.root);f.edit.append('one\n');
    assert.equal(f.root.chunks.join(''),'one\n');assert.equal(f.edit.textChanged.listeners.length,1);
});
test('hex display disables sending without flipping native setting',()=>{
    const f=fixture();f.context.attach(f.root);f.flags.if_show_hex=true;f.flags.if_show_hex_changed.emit();
    assert.equal(f.context.sendCommand('PITCH PID').ok,false);assert.equal(f.manager.sent.length,0);
    assert.equal(f.flags.if_show_hex,true);assert.equal(f.root.statuses.at(-1)[1],false);
});
test('hidden receive disables attachment capability',()=>{
    const f=fixture();f.flags.if_show_echo=false;f.context.attach(f.root);
    assert.equal(f.context.sendCommand('PITCH PID').ok,false);assert.equal(f.flags.if_show_echo,false);
});
test('disconnect is visible and disallows TX',()=>{
    const f=fixture();f.context.attach(f.root);f.manager.connected=false;f.manager.connectedChanged.emit();
    assert.equal(f.root.gaps.length,1);assert.equal(f.context.sendCommand('PITCH PID').ok,false);
});
test('one send invocation appends one LF; control characters rejected',()=>{
    const f=fixture();f.context.attach(f.root);assert(f.context.sendCommand(' PITCH PID ').ok);
    assert.deepEqual(f.manager.sent,['PITCH PID\n']);
    ['PITCH PID\n','PITCH\rSTATUS','PITCH\0PID','x'.repeat(64)].forEach(x=>assert(!f.context.sendCommand(x).ok));
    assert.equal(f.manager.sent.length,1);
});
test('missing compatible document remains explicitly unavailable',()=>{
    const f=fixture();f.manager.parent.children=[];f.context.attach(f.root);
    assert.equal(f.root.statuses.at(-1)[1],false);assert(!f.context.sendCommand('PITCH PID').ok);
});
test('reference-holder capture restores its prior active state',()=>{
    const f=fixture();const loader={active:false,item:{ref_tx_rx_zone:{children:[f.edit,f.combo]}}};
    f.manager.parent={children:[],ref:loader};f.context.attach(f.root);
    assert.equal(loader.active,false);assert.equal(f.root.statuses.at(-1)[1],true);
});
test('native document identity supports separate verified QObject wrappers',()=>{
    const f=fixture();f.edit.textDocument={toString(){return 'QQuickTextDocument(0xabc123)';}};
    f.flags.doc={toString(){return 'QQuickTextDocument(0xABC123)';}};f.context.attach(f.root);
    assert.equal(f.root.statuses.at(-1)[1],true);
});
test('storage writes only own subtree and always restores shared reader source',()=>{
    const f=fixture();f.context.attach(f.root);const before=f.reader.source;
    assert(f.context.writeState('{"version":1}').ok);assert.equal(f.reader.source,before);
    assert.equal(f.context.readState(),'{"version":1}');assert.equal(f.reader.source,before);
    assert([...f.files.keys()].every(p=>p.startsWith('D:/VOFA+/x64/plugins/widgets/report_assistant/state/')));
});
test('failed file read/write restores source and leaves failure visible',()=>{
    const f=fixture();f.context.attach(f.root);f.context.writeState('{"version":1}');
    const before=f.reader.source;f.reader.fail=true;
    assert(!f.context.writeState('{"version":2}').ok);assert.equal(f.reader.source,before);
    assert.throws(()=>f.context.readState(),/read fail/);assert.equal(f.reader.source,before);
});
test('exports reject path traversal and remain inside own exports directory',()=>{
    const f=fixture();f.context.attach(f.root);
    ['../a.txt','D:/a.txt','a/escape.txt','..x.txt'].forEach(n=>assert(!f.context.exportText('report',n).ok));
    const r=f.context.exportText('report','Pitch_Report_123.txt');assert(r.ok);
    assert(r.path.endsWith('/report_assistant/exports/Pitch_Report_123.txt'));
});
test('clipboard uses own invisible TextEdit and never mutates receive document',()=>{
    const f=fixture('native original');f.context.attach(f.root);assert(f.context.copyText('my report').ok);
    assert.equal(f.root.copied,'my report');assert.equal(f.edit.getText(0,f.edit.length),'native original');
});
test('same-event-loop clear/replace is coalesced without a false gap',()=>{
    const f=fixture('baseline\n'),tasks=[],ctx=f.root.getBridgeContext();
    ctx.qt.callLater=fn=>tasks.push(fn);f.root.getBridgeContext=()=>ctx;f.context.attach(f.root);
    f.edit.setText('');f.edit.setText('baseline\n[09:30:42.305] P|PID|POS|1,2,3\n');
    assert.equal(tasks.length,1);assert.equal(f.root.chunks.length,0);tasks.shift()();
    assert.equal(f.root.chunks.join(''),'P|PID|POS|1,2,3\n');assert.equal(f.root.gaps.length,0);
});
test('VOFA time decoration is removed only at line beginning',()=>{
    const f=fixture();f.context.attach(f.root);
    f.edit.append('[09:30:42.305] P|STATUS|note [09:30:43.001] middle\n[other] keep\n');
    assert.equal(f.root.chunks.join(''),'P|STATUS|note [09:30:43.001] middle\n[other] keep\n');
});
test('generic presentation decoder never commits partial paragraphs arbitrarily',()=>{
    const f=fixture();const decoder=f.context.createPresentationDecoder({text:t=>f.root.chunks.push(t)});
    decoder.feed('[09:30:42.305] P|CH');decoder.batchEnd();
    decoder.feed('ECK|ok');decoder.batchEnd();assert.equal(f.root.chunks.length,0);
    decoder.feed('\n[09:30:42.310] pitch:1,2');assert.equal(f.root.chunks.join(''),'P|CHECK|ok\n');
    decoder.feed('\n');assert.equal(f.root.chunks.join(''),'P|CHECK|ok\npitch:1,2\n');
});
test('clear resets pending presentation fragment rather than joining unrelated reports',()=>{
    const f=fixture();const decoder=f.context.createPresentationDecoder({text:t=>f.root.chunks.push(t)});
    decoder.feed('partial OLD');decoder.reset();decoder.feed('NEW\n');
    assert.equal(f.root.chunks.join(''),'NEW\n');
});
test('deferred callback after detach is inert',()=>{
    const f=fixture(),tasks=[],ctx=f.root.getBridgeContext();ctx.qt.callLater=fn=>tasks.push(fn);
    f.root.getBridgeContext=()=>ctx;f.context.attach(f.root);f.edit.append('late\n');f.context.detach();tasks.shift()();
    assert.equal(f.root.chunks.length,0);
});
test('cropped native ANSI head changes do not replay stable report lines',()=>{
    const f=fixture('2mfirst\n[09:37:21.578] pitch:376,2,3\n[09:37:21.593] pitch:377,2,3\n[09:37:21.608] R|RX|EPOCH|377');
    f.context.attach(f.root);
    f.edit.setText(';0;125;255m[09:37:21.578] pitch:376,2,3\n[09:37:21.593] pitch:377,2,3\n[09:37:21.608] R|RX|EPOCH|377\n[09:37:21.624] pitch:378,2,3');
    f.edit.setText('2mpitch:377,2,3\n[09:37:21.608] R|RX|EPOCH|377\n[09:37:21.624] pitch:378,2,3\n[09:37:21.639] P|PID|POS|1,2,3');
    f.edit.append('\n');
    assert.equal(f.root.chunks.join(''),'\npitch:378,2,3\nP|PID|POS|1,2,3\n');
    assert.equal(f.root.gaps.length,0);
});
test('actual native rolling snapshots append each newly visible line once',()=>{
    const nativeFile=path.join(__dirname,'../validation/native-snapshot-fixture.json');
    const samples=JSON.parse(fs.readFileSync(nativeFile,'utf8')).samples;
    const f=fixture(samples[0].text);f.context.attach(f.root);
    samples.slice(1).forEach(s=>f.edit.setText(s.text));
    f.edit.append('\n');
    const output=f.root.chunks.join('');
    for(let n=391;n<=400;n++) assert.equal((output.match(new RegExp('pitch:'+n+',2,3','g'))||[]).length,1,'pitch '+n);
    assert.equal((output.match(/P\|PID\|/g)||[]).length,1);assert.equal(f.root.gaps.length,0);
});
test('native document positions differing from plaintext length never lose the tail',()=>{
    const f=fixture('old\n');let plain='old\n';
    f.edit.getText=(a,b)=>{assert(b-a<=65536);return plain.slice(a,Math.min(b,plain.length));};
    // Simulate native character positions including formatting/paragraph units.
    const nativeEdit={children:[],textDocument:f.edit.textDocument,textChanged:f.edit.textChanged,
        get length(){return plain.length+14;},getText:f.edit.getText};
    f.manager.parent.children=[nativeEdit,f.combo];f.context.attach(f.root);
    plain+='[09:30:42.305] P|PID|POS|1,2,3\n';nativeEdit.textChanged.emit();
    assert.equal(f.root.chunks.join(''),'P|PID|POS|1,2,3\n');assert.equal(f.root.gaps.length,0);
});
test('confirmed FireWater commits final complete display paragraph without waiting for noise',()=>{
    const f=fixture();f.context.attach(f.root);f.edit.append('[09:30:42.305] P|PID|POS|1,2,3');
    assert.equal(f.root.chunks.join(''),'P|PID|POS|1,2,3\n');
    f.edit.append('\n[09:30:42.310] pitch:1,2,3');
    assert.equal(f.root.chunks.join(''),'P|PID|POS|1,2,3\npitch:1,2,3\n');
});
test('switching native protocol disables RX and TX without changing combo',()=>{
    const f=fixture();f.context.attach(f.root);f.combo.currentText='RawData';f.combo.currentTextChanged.emit();
    f.edit.append('P|PID|POS|bad raw fragment');
    assert.equal(f.root.chunks.length,0);assert.equal(f.root.statuses.at(-1)[1],false);
    assert.equal(f.combo.currentText,'RawData');assert(!f.context.sendCommand('PITCH PID').ok);
});
test('unknown, wrong-list, or ambiguous protocol selectors are unavailable',()=>{
    for(const variant of ['missing','wrong','ambiguous']) {
        const f=fixture();
        if(variant==='missing')f.manager.parent.children=[f.edit];
        if(variant==='wrong')f.combo.textAt=i=>['FireWater','BadName','RawData'][i];
        if(variant==='ambiguous')f.manager.parent.children.push({...f.combo});
        f.context.attach(f.root);assert.equal(f.root.statuses.at(-1)[1],false,variant);
        assert(!f.context.sendCommand('PITCH PID').ok);
    }
});
test('verified FireWater tail commit follows coalescing rather than transient document updates',()=>{
    const f=fixture(),tasks=[],ctx=f.root.getBridgeContext();ctx.qt.callLater=fn=>tasks.push(fn);
    f.root.getBridgeContext=()=>ctx;f.context.attach(f.root);
    f.edit.setText('[09:30:42.305] P|CH');f.edit.setText('[09:30:42.305] P|CHECK|ok');
    assert.equal(f.root.chunks.length,0);assert.equal(tasks.length,1);tasks.shift()();
    assert.equal(f.root.chunks.join(''),'P|CHECK|ok\n');
});
test('protocol search skips dense custom-widget graphs before the visit cap',()=>{
    const f=fixture();let subtree={children:[]},chain=subtree;
    for(let i=0;i<300;i++){const next={children:[]};chain.children=[next];chain=next;}
    const widget={path:'heavy_custom_widget',get_widget_ctx(){},children:[subtree]};
    const found=f.context.findProtocolCombo([{children:[f.combo,widget]}],['FireWater','JustFloat','RawData'],10);
    assert.equal(found,f.combo);
});
test('native left-panel protocol reference takes priority over unrelated graph selectors',()=>{
    const f=fixture(),other={...f.combo,currentText:'RawData'};
    f.manager.parent={children:[f.edit,other],ref:{active:false,item:{
        ref_tx_rx_zone:{children:[f.edit]},ref_left_panel:{children:[f.combo]}}}};
    f.context.attach(f.root);assert.equal(f.root.statuses.at(-1)[1],true);
    assert.equal(f.manager.parent.ref.active,false);
});
console.log(passed+' host bridge mock scenarios passed; native UI route requires runtime verification.');
