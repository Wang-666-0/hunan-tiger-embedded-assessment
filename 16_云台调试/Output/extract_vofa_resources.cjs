const fs = require('node:fs');
const zlib = require('node:zlib');
const path = require('node:path');
const source = fs.readFileSync(process.argv[2]);
const out = path.resolve(process.argv[3]);
fs.mkdirSync(out, {recursive:true});
const extracted=[];
for (let i=4;i<source.length-10;i++) {
  if(source[i]!==0x78 || ![0x01,0x9c,0xda,0x5e].includes(source[i+1]))continue;
  const expected=source.readUInt32BE(i-4);
  if(expected<40 || expected>8*1024*1024)continue;
  try {
    const inflated=zlib.inflateSync(source.subarray(i),{maxOutputLength:8*1024*1024});
    if(inflated.length!==expected)continue;
    const text=inflated.toString('utf8');
    if(!/import Qt|function |<svg|uniform |<!DOCTYPE|\{\s*"/.test(text.slice(0,1000)))continue;
    const name='resource-'+i.toString(16)+(text.includes('import Qt')?'.qml':'.txt');
    fs.writeFileSync(path.join(out,name),inflated);
    extracted.push({name,bytes:expected,head:text.slice(0,100)});
  } catch {}
}
fs.writeFileSync(path.join(out,'index.json'),JSON.stringify(extracted,null,2));
console.log(JSON.stringify({extracted:extracted.length,output:out}));
