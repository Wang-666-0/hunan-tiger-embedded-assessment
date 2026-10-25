const fs=require('node:fs');
const load=p=>JSON.parse(fs.readFileSync(p,'utf8').replace(/^\uFEFF/,''));
const a=load(process.argv[2]),b=load(process.argv[3]);
const geometry=w=>w.ctx?.['.']?.ctx?.['.'];
const layout=o=>o.ctx.map(g=>({tabs:g.tabs.map(t=>({name:t.name,widgets:t.widgets.map(w=>({path:w.path,type:w.type,geometry:geometry(w),channel:w.ctx?.ch_menu?.ctx,commands:w.ctx?.cmd_menu?.ctx}))}))}));
const original=layout(a),current=layout(b);
const same=JSON.stringify(original)===JSON.stringify(current);
function diffs(x,y,p='',out=[]){
  if(JSON.stringify(x)===JSON.stringify(y))return out;
  if(x===null||y===null||typeof x!=='object'||typeof y!=='object'){out.push({path:p,before:x,after:y});return out;}
  for(const k of new Set([...Object.keys(x),...Object.keys(y)]))diffs(x[k],y[k],p+'/'+k,out);
  return out;
}
const result={sameControlsPositionsBindings:same,tabs:current.flatMap(g=>g.tabs.map(t=>({name:t.name,widgets:t.widgets.length}))),changedLayoutFields:diffs(original,current),allSavedStateDifferences:diffs(a,b).map(d=>d.path)};
console.log(JSON.stringify(result,null,2));
if(process.argv[4])fs.writeFileSync(process.argv[4],JSON.stringify(result,null,2));
if(!same)process.exitCode=1;
