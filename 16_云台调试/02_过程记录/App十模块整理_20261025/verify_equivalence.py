"""Check preprocessed production statements across normal/historical profiles.

This complements execution tests: ignore comments and the documented private
symbol renames, then compare the actual macro-expanded C statements. Includes
are resolved separately against the before/current headers, using identical
mock system headers. No source file is modified.
"""
from pathlib import Path
import difflib
import importlib.util
import json
import re
import subprocess
import sys
import tempfile

sys.stdout.reconfigure(encoding='utf-8')

HERE = Path(__file__).resolve().parent
FW = HERE.parents[1] / 'Output/Gimbal_Control'
BEFORE = HERE / 'before/Output/Gimbal_Control'
GCC = Path.home() / '.eide/tools/gcc_arm/bin/arm-none-eabi-gcc.exe'
layout_spec = importlib.util.spec_from_file_location('layout',FW/'Tests/source_layout.py')
layout = importlib.util.module_from_spec(layout_spec)
layout_spec.loader.exec_module(layout)
out = Path(tempfile.mkdtemp(prefix='gimbal-equivalence-'))
sections = layout.materialize(FW/'App',out/'sections')
mapping = json.loads((FW/'Tests/app_source_map.json').read_text(encoding='utf-8'))

def preprocess(path,app,flags):
    command = [str(GCC),'-E','-std=c99','-D__attribute__(x)=','-DSTM32F405xx','-DUSE_HAL_DRIVER',
               '-I',str(BEFORE/'Tests/mocks'),'-I',str(app),
               '-I',str(FW/'Core/Inc'),
               '-I',str(FW/'Drivers/STM32F4xx_HAL_Driver/Inc'),
               '-I',str(FW/'Drivers/CMSIS/Include'),
               '-I',str(FW/'Drivers/CMSIS/Device/ST/STM32F4xx/Include'),*flags,str(path)]
    result = subprocess.run(command,capture_output=True,text=True,encoding='utf-8',errors='replace')
    if result.returncode:
        rejected = re.findall(r'error: #error (\w+)', result.stderr)
        if rejected == ['Pitch_requires_protocol_only_cascade_output_limits']:
            # The historical limited-cascade tests intentionally don't compile
            # Pitch. Verify both trees still reject this unsupported pairing.
            return ['PREPROCESS_GUARD_REJECTION',*rejected]
        raise RuntimeError(result.stderr)
    selected,active = [],False
    for line in result.stdout.splitlines():
        marker = re.match(r'#\s+\d+\s+"(.*?)"',line)
        if marker:
            # GCC's path diagnostics may use the Windows ANSI code page. The
            # ASCII C basename is unique here and avoids path-decoding errors.
            active = marker[1].replace('\\\\','/').replace('\\','/').split('/')[-1] == path.name
        elif active:
            selected.append(line)
    text = '\n'.join(selected)
    # Inverse only the explicitly recorded private identifiers.
    inverse = {v:k for k,v in mapping['private_renames'][path.name].items()}
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[A-Za-z_]\w*|\d+(?:\.\d*)?(?:[Ee][+-]?\d+)?[fFuUlL]*|[^\s]',text)
    if len(tokens) < 5:
        raise RuntimeError('No executable tokens selected: '+str(path))
    return [inverse.get(t,t) for t in tokens]

profiles = {
 'production':[],
 'historical_single_guarded':['-DYAW_CONTROLLER_CASCADE_ENABLED=0','-DYAW_CONTROLLER_SPEED_GUARD_ENABLED=1',
     '-DAPP_PITCH_ENABLED=0','-DAPP_FEEDER_ENABLED=0'],
 'historical_limited_cascade':['-DYAW_CASCADE_OUTPUT_LIMITS_ENABLED=1','-DAPP_PITCH_ENABLED=0','-DAPP_FEEDER_ENABLED=0'],
 'experimental_tracking':['-DAPP_YAW_EXPERIMENTAL_TRACKING=1'],
 'legacy_wheels_byte_rx':['-DAPP_REMOTE_PROFILE_ASSESSMENT=0','-DAPP_YAW_ENABLED=0','-DAPP_PITCH_ENABLED=0',
     '-DAPP_FEEDER_ENABLED=0','-DAPP_CONSOLE_RX_DMA_ENABLED=0','-DAPP_REMOTE_RX_DMA_ENABLED=0'],
}
results=[]
for profile,flags in profiles.items():
    for name,(section,production) in sections.items():
        old = preprocess(BEFORE/'App'/name,BEFORE/'App',flags)
        new = preprocess(section,FW/'App',flags)
        # Array sizes use size_t context; adding U to the same positive bound is
        # equivalent. The new timeout macro adds only grouping parentheses.
        if name=='app_console.c':
            for tokens in (old,new):
                for i,t in enumerate(tokens):
                    if t in ('64U','1024U') and i and tokens[i-1]=='[':
                        tokens[i]=t[:-1]
            # Normalize just the known SendText timeout expression grouping.
            for flags_value in ('1','1U','0','0U'):
                pattern=['(',flags_value,'?','120U',':','60U',')']
                for i in range(len(new)-len(pattern),-1,-1):
                    if new[i:i+len(pattern)]==pattern:
                        new[i:i+len(pattern)]=pattern[1:-1]
        equal=old==new
        result={'profile':profile,'section':name,'module':production.name,'equal':equal,
                'before_tokens':len(old),'after_tokens':len(new),
                'kind':'same_guard_rejection' if old[0]=='PREPROCESS_GUARD_REJECTION' else 'same_statements'}
        results.append(result)
        if not equal:
            difference='\n'.join(difflib.unified_diff([' '.join(old)],[' '.join(new)]))
            (HERE/(profile+'_'+name+'.diff')).write_text(difference,encoding='utf-8')
            # Compact first difference, full evidence is stored separately.
            i=next((i for i,(a,b) in enumerate(zip(old,new)) if a!=b),min(len(old),len(new)))
            print(json.dumps({**result,'offset':i,'old':old[max(0,i-8):i+12],'new':new[max(0,i-8):i+12]}))
    print(profile+': checked 32 sections',flush=True)
success=all(r['equal'] for r in results)
(HERE/'preprocessed_equivalence.json').write_text(json.dumps({'success':success,'checks':results},indent=2)+'\n',encoding='utf-8')
print(json.dumps({'success':success,'sections_and_profiles':len(results)}))
raise SystemExit(0 if success else 1)
