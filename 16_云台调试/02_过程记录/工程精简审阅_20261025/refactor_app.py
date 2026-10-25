"""One-shot, backed-up App reorganization authorized on 2026-10-25."""
from pathlib import Path
import re, ast, json, hashlib

REC = Path(__file__).resolve().parent
ROOT = REC.parents[1]
FW = ROOT / 'Output/Gimbal_Control'
APP = FW / 'App'
BEFORE = REC / 'before/App'
WORK = REC / 'work'
FIXED = {
    'APP_REMOTE_PROFILE_ASSESSMENT': 1, 'APP_REMOTE_PROFILE_LEGACY': 0,
    'APP_FLYWHEEL_SPEED_ENABLED': 1, 'APP_FLYWHEEL_SPEED_ACTIVE': 1,
    'APP_FLYWHEEL_JOG_MAX_DURATION_MS': 0, 'FLYWHEEL_JOG_MAX_DURATION_MS': 0,
    'APP_YAW_EXPERIMENTAL_TRACKING': 0, 'YAW_CONTROLLER_CASCADE_ENABLED': 1,
    'YAW_CASCADE_OUTPUT_LIMITS_ENABLED': 0,
    'APP_YAW_CASCADE_LEGACY_VELOCITY_DPS': 0,
}

def condition(expr):
    expr = re.sub(r'/\*.*?\*/|//.*', '', expr)
    expr = re.sub(r'\b(\d+)(?:[uUlL]+)\b', r'\1', expr)
    expr = expr.replace('&&', ' and ').replace('||', ' or ')
    expr = re.sub(r'!(?!=)', ' not ', expr).strip()
    try:
        node = ast.parse(expr, mode='eval').body
    except SyntaxError:
        return None
    def ev(n):
        if isinstance(n, ast.Constant): return n.value
        if isinstance(n, ast.Name): return FIXED.get(n.id)
        if isinstance(n, ast.UnaryOp):
            a=ev(n.operand)
            if a is None: return None
            if isinstance(n.op, ast.Not): return not a
            if isinstance(n.op, ast.USub): return -a
        if isinstance(n, ast.BoolOp):
            xs=[ev(x) for x in n.values]
            if isinstance(n.op, ast.And):
                return False if any(x is not None and not x for x in xs) else (None if None in xs else True)
            return True if any(x is not None and x for x in xs) else (None if None in xs else False)
        if isinstance(n, ast.Compare) and len(n.ops)==1:
            a,b=ev(n.left),ev(n.comparators[0])
            if a is None or b is None:return None
            op=n.ops[0]
            if isinstance(op,ast.Eq):return a==b
            if isinstance(op,ast.NotEq):return a!=b
            if isinstance(op,ast.Lt):return a<b
            if isinstance(op,ast.LtE):return a<=b
            if isinstance(op,ast.Gt):return a>b
            if isinstance(op,ast.GtE):return a>=b
        return None
    return ev(node)

def select_branches(text):
    # Preserve unknown production feature gates; resolve only retired profiles.
    lines=text.splitlines(keepends=True)
    def seq(i, stop=False):
        out=[]
        while i<len(lines):
            m=re.match(r'\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)',lines[i])
            if m and m[1] in ('elif','else','endif'):
                return out,i
            if not m:
                out.append(lines[i]);i+=1;continue
            branches=[]
            while True:
                directive=lines[i]; kind,arg=m[1],m[2].strip()
                if kind=='else': value=True
                elif kind in ('ifdef','ifndef'):
                    value=(kind=='ifdef') if arg in FIXED else None
                else:value=condition(arg)
                body,j=seq(i+1,True);branches.append((directive,value,body))
                end=re.match(r'\s*#\s*(elif|else|endif)\b(.*)',lines[j]);assert end,(i,j)
                if end[1]=='endif':i=j+1;break
                i=j;m=end
            kept=[];unknown=False
            for directive,value,body in branches:
                if value is not None and not value:continue
                if value is not None:
                    if unknown:kept.append('#else\n')
                    kept.extend(body);break
                if not unknown:
                    directive=re.sub(r'(#\s*)elif\b',r'\1if',directive)
                kept.append(directive);kept.extend(body);unknown=True
            if unknown:kept.append('#endif\n')
            out.extend(kept)
        return out,i
    out,end=seq(0);assert end==len(lines)
    return ''.join(out)

def masked(s):
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda m: ''.join('\n' if c=='\n' else ' ' for c in m[0]),s,flags=re.S)

def function_span(s,name):
    code=masked(s)
    m=re.search(r'(?m)^[\w *]+\b'+re.escape(name)+r'\s*\([^;{}]*\)\s*\{',code)
    assert m,name
    start=code.index('{',m.start());depth=1;i=start+1
    while depth:
        depth += (code[i]=='{')-(code[i]=='}');i+=1
    while i<len(s) and s[i]=='\n':i+=1
    return m.start(),i

def drop_function(s,name):
    a,b=function_span(s,name);return s[:a]+s[b:]

def drop_if(s,anchor):
    a=s.index(anchor);start=s.index('{',a);code=masked(s);depth=1;b=start+1
    while depth:
        depth+=(code[b]=='{')-(code[b]=='}');b+=1
    while b<len(s) and s[b]=='\n':b+=1
    return s[:a]+s[b:]

def replace_once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)

def clean_blanks(s): return re.sub(r'\n{3,}', '\n\n',s).strip()+'\n'

texts={p.name:select_branches(p.read_text(encoding='utf-8')) for p in WORK.glob('*.c')}
headers={p.name:select_branches(p.read_text(encoding='utf-8')) for p in BEFORE.glob('*.h')}

# Remove the complete approved device-query service, never actuator protocols.
for name in ['app_diagnostics.c','dm_diag.c','vesc_diag.c']:texts.pop(name)
for name in ['app_diagnostics.h','dm_diag.h','vesc_diag.h']:headers.pop(name)
for name,s in texts.items():
    s=re.sub(r'^#include "(?:app_diagnostics|dm_diag|vesc_diag)\.h"\n','',s,flags=re.M)
    texts[name]=s

s=texts['app_tasks.c']
s=s.replace('    /* 先撤销诊断/旧邮箱，再排双零，不能让Cancel撤销刚提交的停机。 */\n    AppDiagnostics_Cancel();\n','')
s=s.replace('        AppDiagnostics_Poll(HAL_GetTick());\n','').replace('                AppDiagnostics_OnFrame(&frame);\n','')
texts['app_tasks.c']=s

s=texts['app_console.c']
for name in ['HexBytes','ReadNumber','PublishDiagnostics']:
    s=drop_function(s,name)
s=s.replace('static uint8_t diagnostic_mode, status_pending, status_index;\n','')
s=s.replace('    const char *cursor;\n    uint8_t node, field;\n    DiagKind kind;\n    CanFrame check;\n','')
s=drop_if(s,'    if (strcmp(line, "DIAG ON") == 0) {')
s=drop_if(s,'    if (strcmp(line, "DIAG OFF") == 0) {')
start=s.index('    if (!diagnostic_mode) { return; }')
end=s.index('\n}\n',start)
s=s[:start]+s[end:]
s=re.sub(r'\s*strcmp\(line, "DIAG ON"\) == 0 \|\|', '', s)
s=s.replace('        status_pending = 0U;\n','').replace('        if (diagnostic_mode) { SendText("D|ERROR|CANCELLED\\r\\n"); }\n','')
s=s.replace('        AppDiagnostics_Cancel();\n','').replace('        status_pending = 0U;\n','')
s=s.replace('    if (diagnostic_mode) { PublishDiagnostics(HAL_GetTick()); }\n','')
s=re.sub(r'^uint8_t AppConsole_IsDiagnosticMode\(void\)[^\n]*\n','',s,flags=re.M)
# Current assessment profile never accepts serial LEFT/RIGHT/BOTH selection.
for line in ['    if (strcmp(line, "WHEEL LEFT") == 0) { SelectWheel(FLYWHEEL_JOG_LEFT); return; }\n',
             '    if (strcmp(line, "WHEEL RIGHT") == 0) { SelectWheel(FLYWHEEL_JOG_RIGHT); return; }\n',
             '    if (strcmp(line, "WHEEL BOTH") == 0) { SelectWheel(FLYWHEEL_JOG_BOTH); return; }\n']:
    s=s.replace(line,'')
texts['app_console.c']=s

# Retain the backend's explicit inhibit argument (also exercised by tests), but
# remove the console DIAG mode and its cross-module query.
for name,s in texts.items():
    s=re.sub(r'^\s*if \(AppConsole_IsDiagnosticMode\(\)\) \{ return "DIAGNOSTIC"; \}\n','\n',s,flags=re.M)
    s=s.replace(' && !AppConsole_IsDiagnosticMode()','').replace('!AppConsole_IsDiagnosticMode() &&\n            ','')
    s=s.replace(' || AppConsole_IsDiagnosticMode()','')
    s=s.replace('!AppConsole_IsDiagnosticMode()', '1U').replace('AppConsole_IsDiagnosticMode()', '0U')
    texts[name]=s
headers['app_console.h']=headers['app_console.h'].replace('uint8_t AppConsole_IsDiagnosticMode(void);\n','')

s=texts['can_bus.c']
for name in ['SendRead','CanBus_ReadDm','CanBus_ReadVesc','CanBus_CancelReads','CanBus_Send']:
    s=drop_function(s,name)
texts['can_bus.c']=s
for name in ['CanBus_ReadDm','CanBus_ReadVesc','CanBus_CancelReads','CanBus_Send']:
    headers['can_bus.h']=re.sub(r'(?m)^[\w *]+\b'+name+r'\([^;]*;\n','',headers['can_bus.h'])

# Remove Yaw single-loop and experimental APIs, state and validation. The
# shared PID/reference/anti-windup implementation required by Pitch is kept.
s=texts['yaw_controller.c']
for name in ['valid_ff','YawController_SetPid','YawController_SetShape','YawController_SetShapeVelocityGain','YawController_StartShape','YawController_SetFeedforward']:
    s=drop_function(s,name)
s=re.sub(r'^.*yaw->(?:pid\.|integral|have_pid_feedback|speed_limited|shape\.|ff\.).*\n','',s,flags=re.M)
# Multiline experimental validation blocks have no remaining useful condition.
start=s.index('    if (!yaw_controller_finite_value(rate_dps)')
end=s.index('    age = now_ms - yaw->last_feedback_ms;',start)
s=s[:start]+'    if (!yaw_controller_finite_value(rate_dps)) { fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f; }\n'+s[end:]
s=s.replace('    if (rate_dps != 0.0f) { }\n','')
texts['yaw_controller.c']=s
s=headers['yaw_controller.h']
start=s.index('typedef struct { float kp, ki, kd; } YawPid;')
end=s.index('typedef struct {\n    YawPid pid;',start)
s=s[:start]+s[end:]
for line in ['    YawPid pid;\n','    YawFeedforward ff;\n','    YawShape shape;\n','    uint8_t speed_limited;\n']:
    s=s.replace(line,'')
s=s.replace('torque_nm, integral;','torque_nm;').replace('last_control_ms, pid_feedback_ms, pid_sequence, stationary_started_ms;','last_control_ms, stationary_started_ms;').replace('have_control_tick, have_pid_feedback, stationary_samples;','have_control_tick, stationary_samples;')
start=s.index('/* 历史单环接口')
end=s.index('void YawController_Stop',start)
s=s[:start]+'uint8_t YawController_SetTarget(YawController *yaw, float degrees);\n'+s[end:]
headers['yaw_controller.h']=s

s=texts['yaw_backend.c']
for name in ['YawBackend_SetPid','YawBackend_SetVelocityLimit','YawBackend_SetFeedforward','YawBackend_SetShape','YawBackend_SetShapeVelocityGain']:
    s=drop_function(s,name)
start=s.index('    if (YAW_CONTROLLER_CASCADE_ENABLED && parameter')
end=s.index("    } else if (parameter == 'T'",start)
s=s[:start]+"    if (parameter != 'T') { return 0U; }\n    taskENTER_CRITICAL();\n    if (parameter == 'T'"+s[end+len("    } else if (parameter == 'T'"):]
s=re.sub(r'^.*YawController_StartShape\([^\n]*\n','',s,flags=re.M)
# Telemetry retains the existing column positions with inactive legacy values.
for prefix in ['state.controller','yaw.controller']:
    for field,value in {'shape.active':'0U','shape.duration_ms':'0U','shape.velocity_gain':'0.0f','shape.position_deg':prefix+'.target_deg',
        'shape.velocity_dps':'0.0f','shape.acceleration_dps2':'0.0f','ff.enabled':'0U','ff.acceleration_gain':'0.0f','ff.limit_nm':'0.10f','ff.revision':'0U'}.items():
        s=s.replace(prefix+'.'+field,value)
s=s.replace('    if (0U) { taskEXIT_CRITICAL(); return 0U; }\n','')
texts['yaw_backend.c']=s

s=texts['yaw_console.c']
s=drop_function(s,'FeedforwardReport')
start=s.index('    if (strcmp(line, "YAW SHAPE") == 0) {')
end=s.index('    if (strcmp(line, "YAW PID") == 0)',start)
s=s[:start]+s[end:]
s=drop_if(s,'    if (strncmp(line, "YAW VMAX ", 9U) == 0) {')
s=s.replace('    if (strcmp(line, "YAW FF") == 0) { FeedforwardReport(response, capacity); return 1U; }\n','')
texts['yaw_console.c']=s
for name in ['YawBackend_SetPid','YawBackend_SetVelocityLimit','YawBackend_SetFeedforward','YawBackend_SetShape','YawBackend_SetShapeVelocityGain']:
    headers['yaw_backend.h']=re.sub(r'(?m)^[\w *]+\b'+name+r'\([^;]*;\n','',headers['yaw_backend.h'])
s=texts['yaw_cascade.c'];s=drop_function(s,'YawCascade_SetVelocityLimit');s=drop_function(s,'YawCascade_UpdateTracking');texts['yaw_cascade.c']=s
for name in ['YawCascade_SetVelocityLimit','YawCascade_UpdateTracking']:
    headers['yaw_cascade.h']=re.sub(r'(?m)^[\w *]+\b'+name+r'\([^;]*;\n','',headers['yaw_cascade.h'])

# Constants in removed Yaw state are frozen at their previous production
# values for the existing VOFA wire layout; no subsequent field is shifted.
for name,s in list(texts.items()):
    for prefix in ['yaw.controller','state.controller'] if name=='yaw_backend.c' else ['yaw.controller']:
        for field,value in {'shape.active':'0U','shape.duration_ms':'0U','shape.velocity_gain':'0.0f','shape.position_deg':prefix+'.target_deg',
            'shape.velocity_dps':'0.0f','shape.acceleration_dps2':'0.0f','ff.enabled':'0U','ff.acceleration_gain':'0.0f','ff.limit_nm':'0.10f','ff.revision':'0U'}.items():
            s=s.replace(prefix+'.'+field,value)
    texts[name]=s

families={
    'system':['app_fault','app_logic','gimbal_startup','app_tasks','remote_mapper','safety_manager','target_manager','gimbal_step_sync'],
    'drivers':['can_frame','gl60_ii','dm3507','can_bus','ibus','remote_input'],
    'control':['yaw_cascade','yaw_step','pitch_step'],
    'motors':['flywheel_speed','flywheel_jog','motor_backend','feeder_controller','feeder_backend','yaw_controller','yaw_backend','pitch_controller','pitch_backend'],
    'console':['app_console','yaw_console','pitch_console','feeder_console','console_parse','yaw_display_sim','pitch_display_sim','display_demo_jitter','display_step_watch'],
}
locations={name:group for group,names in families.items() for name in names}

# Keep former header documentation as a starting point; update navigation after
# semantic cleanup rather than repeat a generic multi-role file header.
descriptions={
    'app_fault':'应用致命故障处置','app_logic':'遥控、安全与目标管理的应用协调',
    'gimbal_startup':'双轴上电准备和参考建立','app_tasks':'RTOS 任务与运行调度',
    'remote_mapper':'现用 CH5～CH8 遥控映射','safety_manager':'运行许可和失联/故障撤权',
    'target_manager':'手动/阶跃目标与 50 ms 发射节拍','can_frame':'CAN 数据帧基本校验',
    'gl60_ii':'GL60 II 固定协议编解码','dm3507':'DM3507 固定协议编解码',
    'can_bus':'CAN 接收与各电机专用发送权限','ibus':'iBUS 帧解析与校验',
    'remote_input':'UART4 遥控接收与有效帧发布','yaw_cascade':'Yaw/Pitch 共用串级 PID 与观测器',
    'yaw_step':'两轴真实阶跃统计核心','pitch_step':'Pitch 真实阶跃判据适配',
    'flywheel_speed':'摩擦轮速度 PI 与现用速度斜坡','flywheel_jog':'摩擦轮运行状态与反馈保护',
    'motor_backend':'摩擦轮遥控执行与双轮停机确认','feeder_controller':'拨盘单发/连发位置速度控制',
    'feeder_backend':'拨盘准备、触发与 CAN 执行','yaw_controller':'Yaw 双环目标与反馈保护',
    'yaw_backend':'Yaw 会话、阶跃、归位与锁轴','pitch_controller':'Pitch 双环、重力补偿与制动',
    'pitch_backend':'Pitch 会话、阶跃、归位与锁轴','app_console':'串口接收、命令分发与 VOFA 调度',
    'yaw_console':'Yaw 串口命令和真实成绩','pitch_console':'Pitch 串口命令和真实成绩',
    'feeder_console':'拨盘串口命令、状态与结果',
}
for name,s in texts.items():
    stem=Path(name).stem
    functions=re.findall(r'(?m)^(?!static\b)[\w *]+\b(\w+)\s*\([^;{}]*\)\s*\{',masked(s))
    nav=', '.join(functions[:5])+(' 等' if len(functions)>5 else '')
    header=f'/**\n * @file {name}\n * @brief {descriptions[stem]}\n * 入口：{nav}。\n * 配置：app_config.h；与对应头文件配套，内部辅助函数不对外暴露。\n */\n'
    texts[name]=header+clean_blanks(s)

for name,s in list(texts.items())+list(headers.items()):
    if name=='app_config.h':continue
    # Retired build choices become explicit current behavior, not new knobs.
    for macro,value in FIXED.items():
        s=re.sub(r'\b'+macro+r'\b',str(value)+'U',s)
    if name in texts:texts[name]=clean_blanks(s)
    else:headers[name]=clean_blanks(s)

cfg=headers.pop('app_config.h')
for macro,value in FIXED.items():
    cfg=re.sub(r'(?m)^#define '+macro+r'\b(?:[^\n]*\\\n)*[^\n]*\n','',cfg)
    cfg=re.sub(r'\b'+macro+r'\b',str(value)+'U',cfg)
for macro in ['APP_DIAG_VESC_HOST_ID','APP_DIAG_TIMEOUT_MS','APP_DIAG_PASSIVE_SLOTS','VESC_DIAG_MAX_PAYLOAD',
              'VESC_FILL_RX_BUFFER','VESC_FILL_RX_BUFFER_LONG','VESC_PROCESS_RX_BUFFER','VESC_PROCESS_SHORT_BUFFER',
              'APP_YAW_LEGACY_KP','APP_YAW_LEGACY_KD','APP_YAW_FF_LIMIT_NM','APP_YAW_CASCADE_LEGACY_VELOCITY_DPS']:
    cfg=re.sub(r'(?m)^#define '+macro+r'\b[^\n]*\n','',cfg)
cfg=re.sub(r'(?m)^#define DM_DIAG_\w+[^\n]*\n','',cfg)
cfg=cfg.replace('APP_YAW_CASCADE_LEGACY_VELOCITY_DPS','0.0f')
headers['app_config.h']=clean_blanks(cfg)

# Do not touch any live source that changed after the approved review backup.
for p in APP.glob('*'):
    if p.is_file() and p.suffix in ('.c','.h'):
        assert p.read_bytes()==(BEFORE/p.name).read_bytes(),f'Concurrent source change: {p}'
paths={}
for name,s in {**texts,**headers}.items():
    dest=APP/name if name=='app_config.h' else APP/locations[Path(name).stem]/name
    assert not dest.exists() or dest.name=='app_config.h',dest
    dest.parent.mkdir(exist_ok=True)
    dest.write_text(s,encoding='utf-8',newline='\n');paths[name]=str(dest.relative_to(APP)).replace('\\','/')
for p in APP.glob('*'):
    if p.is_file() and p.suffix in ('.c','.h') and p.name!='app_config.h':p.unlink()
(REC/'source_locations.json').write_text(json.dumps(paths,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Reorganized',len(texts),'C files and',len(headers),'headers')
