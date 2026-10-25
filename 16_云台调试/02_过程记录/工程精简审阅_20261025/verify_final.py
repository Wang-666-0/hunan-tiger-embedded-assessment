from pathlib import Path
import re,json,hashlib,xml.etree.ElementTree as ET,shutil
REC=Path(__file__).resolve().parent;ROOT=REC.parents[1];FW=ROOT/'Output/Gimbal_Control';APP=FW/'App';TESTS=FW/'Tests'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
n={'__file__':str(REC/'annotate_app.py')}
exec((REC/'annotate_app.py').read_text(encoding='utf-8').split('prepared={};audit=[];all_docs={}')[0],n)
function_count=0
for name,expected in n['inventory'].items():
    s=(APP/name).read_text(encoding='utf-8')
    matches=[m for m in n['PAT'].finditer(n['mask'](s)) if m[1] not in ('if','while','switch','for')]
    assert [m[1] for m in matches]==expected,name
    for m in matches:
        assert re.search(r'/\* 功能：[^*]+\*/\s*$',s[:m.end()-1]),(name,m[1])
        function_count+=1
for file,variables in json.loads((REC/'global_inventory.json').read_text(encoding='utf-8')).items():
    header=(APP/file).read_text(encoding='utf-8').split('*/',1)[0]
    for name in variables:assert ' *   '+name+'：' in header,(file,name)
    assert '（'+str(len(variables))+'；局部变量见对应函数）' in header,file
for p in APP.rglob('*.c'):
    header=p.read_text(encoding='utf-8').split('*/',1)[0]
    assert '对外函数' in header and '内部函数' in header and '文件级变量/常量' in header,p
for p in APP.rglob('*.h'):
    if p.name=='app_config.h':continue
    defs=re.findall(r'(?m)^#define (\w+)',p.read_text(encoding='utf-8'))
    assert len(defs)==1,(p,defs)

# Check unchanged live control code, independent of the emulator fixtures.
h={'__file__':str(REC/'refactor_app.py')}
exec((REC/'refactor_app.py').read_text(encoding='utf-8').split('texts={p.name:')[0],h)
manifest=json.loads((REC/'before/Tests/app_source_map.json').read_text(encoding='utf-8'))
old={}
for filename,sections in manifest['modules'].items():
    s=(REC/'before/App'/filename).read_text(encoding='utf-8')
    for name in sections:
        old[name]=re.search(r'/\* APP_SECTION_BEGIN '+re.escape(name)+r' \*/\n([\s\S]*?)/\* APP_SECTION_END '+re.escape(name)+r' \*/',s)[1]
preserved=[]
for name in ['pitch_controller.c','feeder_controller.c','flywheel_speed.c']:
    current=next(APP.rglob(name))
    assert n['tokens'](h['select_branches'](old[name]))==n['tokens'](current.read_text(encoding='utf-8')),name
    preserved.append(str(current.relative_to(APP)))
a=h['select_branches'](old['yaw_cascade.c']);b=(APP/'control/yaw_cascade.c').read_text(encoding='utf-8')
sa,ea=h['function_span'](a,'update');sb,eb=h['function_span'](b,'update')
assert n['tokens'](a[sa:ea])==n['tokens'](b[sb:eb]),'Shared PID changed'
preserved.append('control/yaw_cascade.c::update')
params=json.loads((REC/'preservation.json').read_text(encoding='utf-8'))['unchanged_parameter_macros']
s=(APP/'app_config.h').read_text(encoding='utf-8')
definitions={m[1]:m[2].split('/*')[0].strip() for m in re.finditer(r'(?m)^#define\s+(\w+)\s+([^\n]+)',s)}
for name,value in params.items():assert definitions[name]==value,(name,value,definitions.get(name))

# Project views, the build input and actual files must agree.
actual={p.resolve() for p in APP.rglob('*.c')}
yml=(FW/'MDK-ARM/.eide/eide.yml').read_text(encoding='utf-8')
yml_paths={ (FW/'MDK-ARM'/x).resolve() for x in re.findall(r'(?m)^\s*- path: (\.\./App/[^\n]+\.c)\s*$',yml)}
assert actual==yml_paths,'EIDE source list differs'
project=ET.parse(FW/'MDK-ARM/Gimbal_Control.uvprojx')
keil_sources={(FW/'MDK-ARM'/e.findtext('FilePath')).resolve() for e in project.findall('.//Groups/Group/Files/File') if e.findtext('FileType') in ('1','2')}
params_file=json.loads((FW/'MDK-ARM/build/Gimbal_Control/builder.params').read_text(encoding='utf-8'))
builder_sources={(FW/'MDK-ARM'/x).resolve() for x in params_file['sourceList']}
assert builder_sources==keil_sources,'Keil/builder source list differs'
assert all(p.is_file() for p in builder_sources)
assert params_file['options']['c/cpp-compiler']['optimization']=='level-0'
assert "optimization = 'level-0'" in (FW/'Scripts/build.ps1').read_text(encoding='utf-8')
workspace=FW/'MDK-ARM/Gimbal_Control.code-workspace'
assert sha(workspace)==sha(REC/'before/MDK-ARM/Gimbal_Control.code-workspace')
assert not list(APP.rglob('*diag*')),'Retired diagnostic source remains'

report=json.loads((TESTS/'selftest_results.json').read_text(encoding='utf-8'))
baseline=json.loads((REC/'before/Tests/selftest_results.json').read_text(encoding='utf-8'))
base={r['name']:r for r in baseline['results']};expected_failures={'yaw_performance':1,'pitch_shape':2,'pitch_manual90':15}
results=report['results'];assert len(results)==51
assert all(r['finished']==1 and r['returned'] and not r.get('compile_error') for r in results)
failed={r['name']:r['failures'] for r in results if r['failures']}
assert failed==expected_failures,failed
for name,value in failed.items():assert base[name]['failures']==value
hashes={}
for r in results:
    for rel,value in r['source_sha256'].items():
        assert hashes.get(rel,value)==value,rel;hashes[rel]=value
for rel,value in hashes.items():assert sha(FW/rel)==value,'Stale test evidence: '+rel
log=(REC/'build_final.log').read_text(encoding='utf-8')
assert 'build successfully' in log and not re.search(r'\b(?:Error:|Warning:)',log)
rom=int(re.search(r'Total ROM Size[^\n]*?\s(\d+)\s+\(',log)[1])
ram=int(re.search(r'Total RW\s+Size[^\n]*?\s(\d+)\s+\(',log)[1])
hexfile=FW/'MDK-ARM/build/Gimbal_Control/Gimbal_Control.hex'
data={
    'date':'2026-10-25','authorization':'A+B, plus function comments before braces and per-C function/variable inventories',
    'app_c_files':len(actual),'function_comments':function_count,'file_inventories':len(list(APP.rglob('*.c'))),
    'preserved_control_tokens':preserved,'unchanged_parameter_macros':params,
    'test_groups':len(results),'test_passed':len(results)-len(failed),'test_checks':sum(r['checks'] for r in results),
    'existing_failures_only':failed,'verified_test_input_hashes':len(hashes),
    'rom_before':141416,'rom_after':rom,'ram_before':37256,'ram_after':ram,
    'rom_saved':141416-rom,'ram_saved':37256-ram,
    'build':'Original VS Code/EIDE, AC5, O0, 65 C + 1 ASM, no compiler warnings/errors',
    'hex_path':str(hexfile),'hex_sha256':sha(hexfile),'flashed':False,
    'workspace_unchanged':True,'source_lists_agree':True,
}
(REC/'validation.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
shutil.copy2(TESTS/'selftest_results.json',REC/'runtime_tests_final.json')
after={str(p.relative_to(FW)).replace('\\','/'):sha(p) for p in APP.rglob('*') if p.is_file()}
after.update({str(p.relative_to(FW)).replace('\\','/'):sha(p) for p in TESTS.rglob('*') if p.is_file() and '.deps' not in p.parts and '__pycache__' not in p.parts})
(REC/'after_sha256.json').write_text(json.dumps(after,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in data.items() if k not in ('unchanged_parameter_macros','preserved_control_tokens')},ensure_ascii=False,indent=2))
