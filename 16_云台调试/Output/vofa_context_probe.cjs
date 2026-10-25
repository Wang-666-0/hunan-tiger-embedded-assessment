const fs=require('node:fs');
const b=fs.readFileSync(process.argv[2]);
const edits=[];
for(const enc of ['utf8','utf16le']) {
  const from=Buffer.from('/context',enc),to=Buffer.from('/vrepair',enc);
  for(let i=b.indexOf(from);i>=0;i=b.indexOf(from,i+from.length)) {
    to.copy(b,i);edits.push({offset:i,encoding:enc});
  }
}
if(edits.length!==1)throw Error('Expected one context directory literal, got '+JSON.stringify(edits));
fs.writeFileSync(process.argv[3],b);
console.log(JSON.stringify(edits));
