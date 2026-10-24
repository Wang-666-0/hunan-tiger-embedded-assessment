const net = require('node:net');
const fs = require('node:fs');
const path = require('node:path');
const log = { connections:0, frames:0, sent:[], received:[] };
const output = path.join(__dirname,'native-server-result.json');
function persist(){fs.writeFileSync(output,JSON.stringify(log,null,2));}
const server = net.createServer(socket=>{
  log.connections++;
  let frame=0;
  function send(text){socket.write(text);log.sent.push(text);if(log.sent.length>100)log.sent.shift();}
  socket.on('data',b=>log.received.push(b.toString()));
  const timer=setInterval(()=>{
    frame++;log.frames++;
    send(`pitch:${frame},2,3\r\n`);
    if(frame===5)send('P|BUSY|CHECK|QUERY|CC|123\r\n');
    if(frame%30===0)send(`R|RX|EPOCH|${frame}\r\n`);
    if(frame===60)send('P|CHECK|QUERY|NEW_FEEDBACK|ANGLE|30.12\r\n');
    if(frame===100){send('P|STATUS|QUERY|SNAP');setTimeout(()=>send('SHOT_ONLY|ANGLE|31\r\n'),1);}
    if(frame===400){clearInterval(timer);setTimeout(()=>send('P|PID|PKP|1|PKI|0|PKD|0|VKP|2|VKI|0|VKD|0\r\n'),500);persist();}
  },5);
  socket.on('close',()=>{clearInterval(timer);persist();});
  socket.on('error',()=>clearInterval(timer));
});
server.listen(Number(process.argv[2]||39873),'127.0.0.1',()=>console.log('Local native VOFA stream fixture ready'));
setTimeout(()=>{persist();server.close();process.exit(0);},120000);
