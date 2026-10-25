"""Verify final artifacts and preserve evidence for the completed refactor."""
from pathlib import Path
import datetime
import difflib
import hashlib
import json
import re
import shutil
import sys
import xml.etree.ElementTree as ET

sys.stdout.reconfigure(encoding='utf-8')
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
FW=ROOT/'Output/Gimbal_Control'
before=HERE/'before/Output/Gimbal_Control'
baseline=json.loads((HERE/'baseline_tests.json').read_text(encoding='utf-8'))
after=json.loads((HERE/'tests.json').read_text(encoding='utf-8'))
equivalent=json.loads((HERE/'preprocessed_equivalence.json').read_text(encoding='utf-8'))
assert equivalent['success'] and len(equivalent['checks'])==160
old={r['name']:r for r in baseline['results']}
new={r['name']:r for r in after['results']}
assert old.keys()==new.keys()
keys=('checks','failures','first_failed_line','stage','finished','returned','telemetry_max_bytes','telemetry_max_wire_us')
changes=[{'group':n,'field':k,'before':old[n].get(k),'after':new[n].get(k)}
         for n in old for k in keys if old[n].get(k)!=new[n].get(k)]
assert not changes,changes
input_hashes={}
for r in after['results']:
    for name,expected in r['source_sha256'].items():
        actual=hashlib.sha256((FW/name).read_bytes()).hexdigest()
        assert actual==expected,name
        input_hashes[name]=actual
assert all(r['finished']==1 and r['returned'] for r in after['results'])

modules=sorted(p.name for p in (FW/'App').glob('*.c'))
assert len(modules)==10
headers=list((FW/'App').glob('*.h'))
for p in [*(FW/'App').glob('*.c'),*headers]:
    text=p.read_text(encoding='utf-8')
    assert '@file '+p.name in text[:150] and '@brief' in text[:180],p
    assert not re.search(r'#include\s+"[^"\n]*\.c"',text),p
    if p.name!='app_config.h':
        defines=re.findall(r'^\s*#define\s+(\w+)',text,re.M)
        assert defines==[re.search(r'^#ifndef\s+(\w+)',text,re.M)[1]] if p.suffix=='.h' else not defines,p
eide=(FW/'MDK-ARM/.eide/eide.yml').read_text(encoding='utf-8')
eide_sources=re.findall(r'path: ../App/(\w+\.c)',eide)
assert sorted(eide_sources)==modules
keil=ET.parse(FW/'MDK-ARM/Gimbal_Control.uvprojx')
keil_sources=[f.findtext('FileName') for g in keil.findall('.//Group')
              if g.findtext('GroupName')=='App' for f in g.findall('./Files/File')]
assert sorted(keil_sources)==modules
params=json.loads((FW/'MDK-ARM/build/Gimbal_Control/builder.params').read_text(encoding='utf-8'))
assert sorted(p.split('/')[-1] for p in params['sourceList'] if '/App/' in p)==modules
assert params['options']['c/cpp-compiler']['optimization']=='level-0'

# Move the newly added navigation entries ahead of the retained history.
for path,title in [(FW/'README.md','# 2026-10-25：App 文件及配置导航'),
                   (FW/'Tests/README.md','# 2026-10-25：十个实现文件的测试入口')]:
    text=path.read_text(encoding='utf-8')
    index=text.index(title)
    if index:
        text=text[index:].strip()+'\n\n---\n\n'+text[:index].strip()+'\n'
        path.write_text(text,encoding='utf-8',newline='\n')
failed=[{k:r[k] for k in ('name','checks','failures','first_failed_line','stage')} for r in after['results'] if r['failures']]
hex_path=FW/'MDK-ARM/build/Gimbal_Control/Gimbal_Control.hex'
hex_sha=hashlib.sha256(hex_path.read_bytes()).hexdigest()
summary={
 'recorded_at':datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(timespec='seconds'),
 'scope':'Equivalent App implementation consolidation and same-value macro centralization; no hardware activity.',
 'before_c_files':32,'after_c_files':len(modules),'documented_headers':len(headers),'modules':modules,
 'test_groups':len(new),'checks':sum(r['checks'] for r in new.values()),
 'baseline_unchanged':True,'new_failures':0,'baseline_failed_groups':failed,
 'passed_groups':sum(r['failures']==0 for r in new.values()),
 'preprocessed_checks':len(equivalent['checks']),
 'equal_statement_cases':sum(r['kind']=='same_statements' for r in equivalent['checks']),
 'same_guard_rejection_cases':sum(r['kind']=='same_guard_rejection' for r in equivalent['checks']),
 'verified_source_hashes':len(input_hashes),'source_sha256':input_hashes,
 'eide_keil_builder_lists_match':True,'build':'ARM Compiler 5.06, O0, 46 C + 1 ASM, full rebuild, success',
 'rom_bytes':141372,'ram_bytes':37256,'hex':str(hex_path),'hex_sha256':hex_sha,
 'flashed':False,'hardware_verified':False,
}
(HERE/'validation.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

paths=[*sorted((FW/'App').glob('*.*')),FW/'README.md',FW/'Tests/README.md',
       FW/'Tests/run_selftests.py',FW/'Tests/source_layout.py',FW/'Tests/app_source_map.json',
       FW/'MDK-ARM/.eide/eide.yml',FW/'MDK-ARM/Gimbal_Control.uvprojx']
diff=[]
for path in paths:
    relative=path.relative_to(FW)
    dest=HERE/'after/Output/Gimbal_Control'/relative
    dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(path,dest)
    previous=before/relative
    old_text=previous.read_text(encoding='utf-8-sig') if previous.exists() else ''
    diff.extend(difflib.unified_diff(old_text.splitlines(True),path.read_text(encoding='utf-8-sig').splitlines(True),
       fromfile='before/'+relative.as_posix(),tofile='after/'+relative.as_posix()))
for path in sorted((before/'App').glob('*.c')):
    if not (FW/'App'/path.name).exists():
        diff.extend(difflib.unified_diff(path.read_text(encoding='utf-8-sig').splitlines(True),[],
           fromfile='before/App/'+path.name,tofile='after/App/'+path.name))
(HERE/'changes.diff').write_text(''.join(diff),encoding='utf-8')
rows='\n'.join('| '+r['name']+' | '+str(r['failures'])+' | '+str(r['first_failed_line'])+' |' for r in failed)
record=f'''# App 十模块整理：实现与验证

日期：2026-10-25。用户要求减少 App 实现文件、统一宏配置，并为每个文件添加职责及主要函数说明。本次在原 VS Code/EIDE 工程完成，未烧录或连接电机。

## 改动

- 32 个 C 实现合并为 10 个，原公开头文件和接口保留；完整职责及函数入口见 `Output/Gimbal_Control/App/README.md`。
- 迁移 150 个分散的宏名称到 `app_config.h`；原条件编译和 `-D` 覆盖包装保留。局部通用宏增加设备前缀避免全局重名。
- 新增 31 个同值默认参数宏，覆盖拨盘初始化值、历史单环默认、缓冲大小等。没有改变 PID 数值、控制周期、运动逻辑、保护、串口命令或通道顺序。
- 合并后重名的私有符号使用原逻辑模块前缀，公开符号不改名。没有 `#include .c` 或运行时分发新开销。
- 全部 10 个实现及 39 个头文件添加文件说明；集中参数入口增加导航、单位和生效方式。清理迁走宏后悬空的注释，并说明共用 PID、实测与演示边界。
- 更新原 `.eide/eide.yml`、`.uvprojx`、EIDE builder 缓存及测试依赖映射。测试从正式模块分区读取代码，哈希跟踪完整的正式文件。

## 等价验证

整理前后完整运行 85 组，均完成并返回，共 {summary['checks']:,} 个检查。每组检查数、失败数、首个失败位置、阶段、完成状态以及已有串口字节预算记录全部相同。**79 组通过，6 组原有失败保留，没有新增失败。** 没有修改测试断言或生产逻辑来掩盖原有失败。

| 原有失败组 | 失败断言数 | 首个失败行 |
| --- | --- | --- |
{rows}

另对 32 个逻辑分区在 5 种配置下比较宏展开后的 C 语句：159 项语句一致，1 项不支持的历史限幅双环/Pitch 组合在前后均由相同 `#error` 拒绝，共 160 项符合预期。比较忽略注释、记录过的私有重命名，以及同值数组长度无符号后缀和超时表达式括号；不忽略其他运算或常量。详见 `verify_equivalence.py`、`preprocessed_equivalence.json`。

测试报告的 {len(input_hashes)} 个唯一源码输入哈希已与最终源码逐一复核。测试方法为 ARM GCC `-Wall -Wextra -Werror` + Unicorn，HAL/RTOS 模拟；完整固件另以 AC5 验证十个合并模块实际共同编译/链接。

## 编译交付

原工程 ARM Compiler 5.06、O0 全量重编通过，46 C + 1 ASM。ROM 141372 字节、RAM 37256 字节。

HEX：`{hex_path}`

SHA256：`{hex_sha}`

继续打开原 `MDK-ARM/Gimbal_Control.code-workspace` 使用 VS Code/EIDE 编译烧录，无须另建工程。原文件备份在 `before/`，最终文件在 `after/`，具体差异为 `changes.diff`。

本次验证证明已覆盖软件行为与整理前一致，不代表 6 组既有测试失败已修复，也不代替实机响应、时序及归位确认。
'''
(HERE/'实现与验证.md').write_text(record,encoding='utf-8')
print(json.dumps({k:summary[k] for k in ('after_c_files','documented_headers','test_groups','checks','passed_groups','new_failures','preprocessed_checks','verified_source_hashes','hex_sha256')},ensure_ascii=False))
