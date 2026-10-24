const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');const path=require('node:path');
const {createEngine}=require('../plugins/widgets/pitch_sim_plot/SimEngine.js');
function frame(angle=0,target=30,id=1,phase=1,settle=-1,overshoot=0){
 const v=Array(65).fill(0);v[0]=angle;v[1]=target;v[24]=id;v[25]=phase;v[27]=settle;v[28]=overshoot;return v;
}
function armed(settle=420,overshoot=0){const e=createEngine();e.feedFrame(frame(0,0,1,2,settle,overshoot),1000);return e;}
test('normal first run remains real and does not mutate its input',()=>{
 const e=createEngine(),v=frame(3,30);const before=JSON.stringify(v);
 assert.equal(e.feedFrame(v,1000),true);assert.equal(e.snapshot().active,false);
 assert.equal(e.snapshot().displayAngle,3);assert.equal(JSON.stringify(v),before);
});
test('only the next new step after a failed completed result is simulated',()=>{
 const e=armed();assert.equal(e.snapshot().previousFailed,true);
 e.feedFrame(frame(0,0,1,2,420),1100);assert.equal(e.snapshot().active,false);
 e.feedFrame(frame(0,30,2),1200);assert.equal(e.snapshot().active,true);
 e.feedFrame(frame(2,30,2),1320);assert.equal(e.snapshot().displayAngle,15);
 assert.equal(e.snapshot().realAngle,2);assert.equal(e.snapshot().target,30);
 assert.deepEqual(e.snapshot().lastResult,{id:1,settleMs:420,overshootDeg:0,failed:true});
});
test('ideal curve is monotone, stays within endpoints and settles at 240ms in both directions',()=>{
 for(const reverse of [false,true]){
  const start=reverse?30:0,target=reverse?0:30,e=armed();
  e.feedFrame(frame(start,start,1,2,420),1005);e.feedFrame(frame(start,target,2),1010);
  let prev=start;
  for(let dt=0;dt<=300;dt+=5){
   e.feedFrame(frame(start,target,2),1010+dt);const value=e.snapshot().displayAngle;
   assert.ok(value>=-1e-9&&value<=30+1e-9);assert.ok(reverse?value<=prev+1e-9:value>=prev-1e-9);
   if(dt>=240)assert.equal(value,target);prev=value;
  }
 }
});
test('completion restores real I0 immediately and retains the actual failed score',()=>{
 const e=armed();e.feedFrame(frame(0,30,2),1100);e.feedFrame(frame(25,30,2),1400);
 assert.equal(e.snapshot().displayAngle,30);
 e.feedFrame(frame(27.5,30,2,2,450,0.7),6100);
 assert.equal(e.snapshot().active,false);assert.equal(e.snapshot().displayAngle,27.5);
 assert.equal(e.snapshot().lastResult.settleMs,450);assert.equal(e.snapshot().lastResult.overshootDeg,0.7);
 assert.ok(e.points().some(p=>p.sim));assert.equal(e.points().at(-1).sim,false);
});
test('overshoot threshold triggers while a passing time and overshoot do not',()=>{
 for(const [settle,os,wanted] of [[300,0.2,false],[301,0,true],[250,0.201,true],[-1,0,true]]){
  const e=armed(settle,os);e.feedFrame(frame(0,30,2),1200);assert.equal(e.snapshot().active,wanted);
 }
});
test('aborts, disable, goal change and lost reception cancel simulation',()=>{
 for(const action of ['abort','disable','target','gap']){
  const e=armed();e.feedFrame(frame(0,30,2),1200);assert.equal(e.snapshot().active,true);
  if(action==='abort')e.feedFrame(frame(3,30,2,3),1205);
  if(action==='disable')e.setEnabled(false);
  if(action==='target')e.feedFrame(frame(3,20,2),1205);
  if(action==='gap')e.gap();
  assert.equal(e.snapshot().active,false);
  if(action==='gap')assert.equal(e.snapshot().displayAngle,null);
 }
 const e=armed();e.gap();e.feedFrame(frame(3,30,2),1200);assert.equal(e.snapshot().active,false);
});
test('an aborted old run does not arm the following run',()=>{
 const e=armed();e.feedFrame(frame(0,0,1,3),1100);e.feedFrame(frame(0,30,2),1200);
 assert.equal(e.snapshot().active,false);
});
test('stream decoder accepts split lines and native pitch prefix on its own line',()=>{
 const e=createEngine();const v=frame(12,30,1,1);const text='pitch:'+v.join(',')+'\r\n';
 assert.equal(e.feed(text.slice(0,26),1000),0);assert.equal(e.feed(text.slice(26),1005),1);
 assert.equal(e.snapshot().displayAngle,12);
 assert.equal(e.feed('[12:34:56.789] pitch:\n'+frame(13).join(',')+'\n',1010),1);
 assert.equal(e.snapshot().displayAngle,13);
});
test('other groups, malformed columns and nonfinite values cannot masquerade as pitch',()=>{
 const e=createEngine();assert.equal(e.feed('gimbal:'+frame(12).join(',')+'\n',1000),0);
 assert.equal(e.feed('pitch:1,2\n',1005),0);
 const v=frame();v[4]=NaN;assert.equal(e.feedFrame(v,1010),false);
 v[4]=0;v[25]=4;assert.equal(e.feedFrame(v,1010),false);
 assert.equal(e.snapshot().accepted,0);
});
test('time reset clears automatic arming and storage is bounded',()=>{
 const e=armed();e.feedFrame(frame(0,30,2),900);assert.equal(e.snapshot().active,false);
 for(let i=0;i<1700;i++)e.feedFrame(frame(1,30,2),1000+i*5);
 assert.equal(e.points().length,1600);
 assert.equal(e.feed('x'.repeat(65537),10000),0);assert.ok(e.snapshot().gaps>0);
});
test('QML always identifies SIM and the receive bridge has no serial-send entry point',()=>{
 const folder=path.join(__dirname,'../plugins/widgets/pitch_sim_plot');
 const qml=fs.readFileSync(path.join(folder,'pitch_sim_plot.qml'),'utf8');
 const bridge=fs.readFileSync(path.join(folder,'ReceiveBridge.js'),'utf8');
 assert.match(qml,/SIM 模拟 · 非实测/);assert.match(qml,/实时 I0 实测/);
 assert.doesNotMatch(bridge,/function sendCommand|\.send_string\s*\(|function writeState/);
 assert.doesNotMatch(qml,/PutData|send_command|send_string|\.sendCommand\s*\(/);
});
