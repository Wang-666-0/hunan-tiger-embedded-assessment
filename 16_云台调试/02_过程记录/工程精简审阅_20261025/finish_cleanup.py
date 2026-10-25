from pathlib import Path
import json,re,xml.etree.ElementTree as ET
REC=Path(__file__).resolve().parent; ROOT=REC.parents[1]; FW=ROOT/'Output/Gimbal_Control'; APP=FW/'App'
def edit(rel,fn):
    p=APP/rel;p.write_text(fn(p.read_text(encoding='utf-8')),encoding='utf-8',newline='\n')
edit('motors/yaw_backend.c',lambda s:re.sub(r'^.*(?:ff_enabled|ff_revision).*\n','',s,flags=re.M))
edit('console/app_console.c',lambda s:s.replace('    if (diagnostic_mode) { return "DIAGNOSTIC_MODE"; }\n','')
     .replace('HAL_GetTick(), diagnostic_mode)', 'HAL_GetTick(), 0U)')
     .replace('        if (diagnostic_mode) { SendText("D|ERROR|UART_FAULT\\r\\n"); }\n','')
     .replace('!diagnostic_mode && ','').replace('    else { PublishTelemetry(HAL_GetTick()); }','    PublishTelemetry(HAL_GetTick());'))
edit('drivers/can_bus.c',lambda s:s.replace('TX_OWNER_PITCH, TX_OWNER_DIAGNOSTIC,','TX_OWNER_PITCH,'))
edit('control/yaw_cascade.c',lambda s:s.replace('0U ? 0U : 0.0f','0.0f'))

# Documentation uses actual public entry names, never a parser's control words.
for p in APP.rglob('*.c'):
    s=p.read_text(encoding='utf-8'); body=re.sub(r'/\*.*?\*/|//[^\n]*','',s,flags=re.S)
    names=[m[1] for m in re.finditer(r'(?m)^(?!static\b)(?:[A-Za-z_]\w*[ *]+)+([A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{',body) if m[1] not in ('if','while','switch','for')]
    s=re.sub(r' \* 入口：[^\n]*', ' * 入口：'+', '.join(names[:5])+(' 等' if len(names)>5 else '')+'。',s)
    p.write_text(s,encoding='utf-8',newline='\n')

groups=['system','drivers','control','motors','console']
locations=json.loads((REC/'source_locations.json').read_text(encoding='utf-8'))
cfiles=sorted(p for p in locations.values() if p.endswith('.c'))

# Keep the original VS Code/EIDE checkout, toolchain and O0 setting.
p=FW/'MDK-ARM/.eide/eide.yml';s=p.read_text(encoding='utf-8')
start=s.index('    - name: App\n');end=s.index('\ndependenceList:',start)
block='    - name: App\n      files: []\n      folders:\n'
for group in groups:
    block+=f'        - name: {group}\n          files:\n'
    for rel in cfiles:
        if rel.startswith(group+'/'):block+=f'            - path: ../App/{rel}\n'
    block+='          folders: []\n'
s=s[:start]+block.rstrip()+s[end:]
s=s.replace('        - ../App\n','        - ../App\n'+''.join(f'        - ../App/{g}\n' for g in groups))
p.write_text(s,encoding='utf-8',newline='\n')
p=FW/'MDK-ARM/Gimbal_Control.uvprojx';s=p.read_text(encoding='utf-8')
m=re.search(r'        <Group>\s*<GroupName>App</GroupName>[\s\S]*?</Group>',s);assert m
blocks=[]
for group in groups:
    block='        <Group>\n          <GroupName>App/'+group+'</GroupName>\n          <Files>\n'
    for rel in cfiles:
        if rel.startswith(group+'/'):
            block+='            <File>\n              <FileName>'+Path(rel).name+'</FileName>\n              <FileType>1</FileType>\n              <FilePath>../App/'+rel+'</FilePath>\n            </File>\n'
    blocks.append(block+'          </Files>\n        </Group>')
s=s[:m.start()]+'\n'.join(blocks)+s[m.end():]
s=re.sub(r'(<IncludePath>)([^<]*)(</IncludePath>)',lambda m:m[1]+m[2]+';'+ ';'.join('../App/'+g for g in groups)+m[3] if '../App' in m[2] else m[0],s)
ET.fromstring(s);p.write_text(s,encoding='utf-8',newline='\n')
p=FW/'MDK-ARM/build/Gimbal_Control/builder.params';d=json.loads(p.read_text(encoding='utf-8'))
d['sourceList']=[x for x in d['sourceList'] if not x.replace('\\','/').startswith('../App/')]+['../App/'+r for r in cfiles]
d['incDirs']+=['../App/'+g for g in groups if '../App/'+g not in d['incDirs']]
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
p=FW/'Scripts/build.ps1';s=p.read_text(encoding='utf-8').replace("optimization = 'level-3'", "optimization = 'level-0'")
p.write_text(s,encoding='utf-8',newline='\n')
print('Updated original EIDE / Keil / builder source lists:',len(cfiles),'App sources')
