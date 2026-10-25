"""One-shot mechanical reorganization; the preserved before/ tree is the input."""
from pathlib import Path
import collections
import hashlib
import json
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FW = ROOT / 'Output/Gimbal_Control'
APP = FW / 'App'
BEFORE = HERE / 'before/Output/Gimbal_Control'

# Keep the public headers and API names. Only implementation ownership changes.
GROUPS = {
    'app_tasks.c': ['app_fault.c', 'app_logic.c', 'gimbal_startup.c', 'app_tasks.c'],
    'app_console.c': ['app_console.c'],
    'motor_console.c': ['yaw_console.c', 'pitch_console.c', 'feeder_console.c'],
    'can_bus.c': ['can_frame.c', 'gl60_ii.c', 'dm3507.c', 'can_bus.c'],
    'app_diagnostics.c': ['dm_diag.c', 'vesc_diag.c', 'app_diagnostics.c'],
    'remote_input.c': ['ibus.c', 'remote_mapper.c', 'safety_manager.c', 'target_manager.c', 'remote_input.c'],
    'motor_backend.c': ['flywheel_speed.c', 'flywheel_jog.c', 'motor_backend.c'],
    'feeder_controller.c': ['feeder_controller.c', 'feeder_backend.c'],
    'yaw_controller.c': ['yaw_cascade.c', 'yaw_controller.c', 'yaw_step.c', 'yaw_backend.c'],
    'pitch_controller.c': ['pitch_controller.c', 'pitch_step.c', 'pitch_backend.c'],
}
INFO = {
    'app_tasks.c': ('应用调度与启动', '创建四个 RTOS 任务，组合安全许可，协调双轴准备并发布状态快照。',
        ['AppTasks_Create：初始化后端、队列及遥控/CAN/控制/控制台任务。',
         'AppLogic_Step：先执行安全判定，再更新遥控目标和执行器请求。',
         'GimbalStartup_Update：按安全 OFF、反馈及使能握手推进双轴准备。',
         'AppTasks_RequestStop / AppFault_Fatal：分别处理可恢复停机与致命故障。'],
        'main → AppTasks_Create；控制任务每 APP_CONTROL_PERIOD_MS 调用各后端 Apply。',
        '保护判定、许可及命令提交仍在原临界区；不得让过期快照重新开放输出。'),
    'app_console.c': ('串口命令入口与 VOFA 遥测', '接收串口命令，选择显示视图，发布实测状态及带标记的演示数据。',
        ['AppConsole_CreateQueue / Start：建立命令队列并启动 USART1 接收。',
         'AppConsole_Poll：解析完整命令行，处理回执、报告和定时遥测。',
         'HandleLine：分发系统、Yaw、Pitch、拨盘和摩擦轮命令。',
         'PublishYawTelemetry / PublishPitchTelemetry：维持既有组名及通道顺序。'],
        'USART1 DMA/中断 → 接收队列 → ConsoleTask → 命令处理或遥测输出。',
        '格式化和发送只在控制台任务执行；演示角度不能进入反馈、闭环或实测成绩。'),
    'motor_console.c': ('执行器串口命令', '实现 Yaw、Pitch、拨盘的命令解析、准备流程及状态报告。',
        ['YawConsole_Handle / Poll：处理 YAW 命令、准备和阶跃报告。',
         'PitchConsole_Handle / Poll：处理 PITCH 命令、坐标/重力查询及报告。',
         'FeederConsole_Handle：处理 FEEDER 配置、单发/连发和状态查询。'],
        'app_console 分发命令 → 本文件校验参数 → 对应 Backend 执行。',
        '三种设备的握手条件分别保留；查询不应隐式改变使能、零点或积分。'),
    'can_bus.c': ('CAN 传输与电机协议', '编码/解析电机报文，管理两条 CAN 的接收队列、发送邮箱和输出许可。',
        ['CanFrame_*：验证通用帧并构造协议数据。',
         'Gl60_* / Dm3507_*：转换 GL60 II、DM3507 的位置、速度和力矩字段。',
         'CanBus_Start / Receive：启动总线并向 CAN 任务交付时间戳反馈。',
         'CanBus_*Torque / *Disable：受许可约束地提交命令或停机帧。'],
        'CAN ISR → 队列 → CanTask → 各 Backend；控制任务经 CanBus 提交输出。',
        '邮箱所有权、故障代次和撤权顺序必须保留；停机不能被旧命令覆盖。'),
    'app_diagnostics.c': ('只读设备诊断', '构造参数读取请求、拼接回复，处理超时并向控制台交付诊断结果。',
        ['DmDiag_BuildRead / MatchesReply：白名单 RID 查询及回复核对。',
         'VescDiag_Begin / OnFrame：VESC 分片接收、长度和 CRC 校验。',
         'AppDiagnostics_Request / Poll：串行推进查询及超时处理。',
         'AppDiagnostics_TakeEvent / Cancel：交付结果或撤销未完成查询。'],
        '控制台请求 → CanTask 轮询 → CAN 回复 → 控制台逐段读取。',
        '本模块仅查询；取消代次隔离旧回复，不添加使能或写入设备参数的行为。'),
    'remote_input.c': ('遥控输入、安全判定与目标管理', '解析 iBUS，映射通道，形成安全许可及单发、连发、阶跃目标请求。',
        ['ibus_feed*：寻找帧头，校验长度、通道、校验和及字节超时。',
         'RemoteInput_Poll / GetState：消费接收队列并发布有效遥控快照。',
         'RemoteMapper_Map：将物理通道映射为模式、方向和开关状态。',
         'SafetyManager_Step / TargetManager_Step：先判保护，再生成边沿事件和目标。'],
        'UART4 → iBUS 队列/解析 → RemoteMapper → SafetyManager → TargetManager。',
        'CH7 为边沿事件，长按不重复；故障后的解锁依赖新的安全 OFF 帧。'),
    'motor_backend.c': ('双摩擦轮控制', '管理左右摩擦轮会话、反馈健康、速度环、受限输出及停机确认。',
        ['FlywheelSpeed_*：目标爬升、速度滤波、PI 和电流限幅。',
         'FlywheelJog_*：点动/持续运行、双轮反馈及故障处理。',
         'MotorBackend_Apply / OnFrame：对接遥控请求与 CAN 反馈。',
         'MotorBackend_GetHealth / Stop：提供健康状态并安排双轮停机。'],
        '控制任务 → MotorBackend → FlywheelJog/Speed → CanBus；反馈反向回流。',
        'ERPM 与机械 RPM 按极对数换算；左右电流方向和互锁沿用当前配置。'),
    'feeder_controller.c': ('拨盘控制与执行', '累计编码器角度，生成单发/连发目标，计算位置/速度控制及保护。',
        ['Feeder_Init / SetConfig：加载 app_config.h 默认参数或显式配置。',
         'Feeder_OnFrame / Update：累计位置，计算输出并检查堵转、超时。',
         'FeederBackend_Apply：执行单发/连发、摩擦轮互锁和停机流程。',
         'FeederBackend_GetState：向控制台提供目标角度及实际角度。'],
        'TargetManager 发射请求 → FeederBackend → Feeder 控制器 → CAN 电流命令。',
        '连发每 APP_FIRE_INTERVAL_MS 推进目标；PID 仍随高速控制任务更新。'),
    'yaw_controller.c': ('Yaw 轴及共用串级 PID', '实现共用串级 PID、Yaw 控制、阶跃实测和 GL60 II 会话后端。',
        ['YawCascade_*：两轴共用的速度观测、双环 PID 与抗积分饱和。',
         'YawController_Update：Yaw 位置目标、轨迹/前馈和力矩计算。',
         'YawStep_*：统计真实反馈的首次到位、稳定时间、超调及稳态误差。',
         'YawBackend_Apply / StartStep：处理许可、使能、锁轴及相对阶跃。'],
        '控制任务 → YawBackend → YawController/YawCascade → CanBus；Pitch 也调用 YawCascade。',
        '共用 PID 不是 Yaw 专属；实测结果不读取演示角度，限位和旧模式开关保留。'),
    'pitch_controller.c': ('Pitch 轴控制与执行', '实现 Pitch 串级 PID 接入、重力补偿、末段制动、阶跃实测及会话后端。',
        ['PitchController_Update：叠加 PID、重力和制动，输出限幅力矩。',
         'PitchController_StartMove / ReturnTo：普通运动与可中断归位分别处理。',
         'PitchStep_*：复用 YawStep 统计核心，保留 Pitch 判定参数。',
         'PitchBackend_Apply / SetViewLock：处理坐标、使能、锁轴与反馈保护。'],
        '控制任务 → PitchBackend → PitchController → 共用 YawCascade → CanBus。',
        '重力补偿依赖角度坐标和水平标定；锁轴保持锁定时目标，不随受扰反馈重取。'),
}

TOKEN = re.compile(r'/\*[\s\S]*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[A-Za-z_]\w*|->|\S')
def rename(text, mapping):
    previous = ''
    def replace(m):
        nonlocal previous
        word = m.group()
        value = mapping.get(word, word) if previous not in ('.', '->') else word
        if not word.startswith(('/*', '//')):
            previous = word
        return value
    return TOKEN.sub(replace, text)

def mask(text):
    return re.sub(r'/\*[\s\S]*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda m: re.sub(r'[^\n]', ' ', m.group()), text)

def static_names(text):
    found = set()
    for m in re.finditer(r'^static\s+([^;{]+)[;{]', mask(text), re.M):
        decl = m.group(1)
        if '(' in decl and ('=' not in decl or decl.index('(') < decl.index('=')):
            found.add(re.findall(r'\w+', decl.split('(')[0])[-1])
        else:
            for part in decl.split(','):
                found.add(re.findall(r'\w+', re.split(r'[=\[]', part)[0])[-1])
    return found

local_macros = {
    'yaw_controller.c': {'PI_RAD':'YAW_PI_RAD', 'TWO_PI_RAD':'YAW_TWO_PI_RAD',
        'RAD_TO_DEG':'YAW_RAD_TO_DEG', 'DEG_TO_RAD':'YAW_DEG_TO_RAD', 'RECOVERABLE_FAULTS':'YAW_RECOVERABLE_FAULTS'},
    'pitch_controller.c': {'RAD_TO_DEG':'PITCH_RAD_TO_DEG', 'RECOVERABLE_FAULTS':'PITCH_RECOVERABLE_FAULTS',
        'LANDING_TRACE_WINDOW_MS':'PITCH_LANDING_TRACE_WINDOW_MS'},
    'feeder_controller.c': {'COUNTS_TO_DEGREES':'FEEDER_COUNTS_TO_DEGREES'},
    'flywheel_jog.c': {'STATUS1_PACKET':'FLYWHEEL_STATUS1_PACKET', 'STATUS4_PACKET':'FLYWHEEL_STATUS4_PACKET',
        'BOTH_NODES':'FLYWHEEL_BOTH_NODES'},
    'yaw_console.c': {'READY_TIMEOUT_MS':'YAW_READY_TIMEOUT_MS'},
    'yaw_backend.c': {'OFF_MOTION_FAULTS':'YAW_OFF_MOTION_FAULTS'},
}

def extract_macros(text, guard=None):
    """Move definitions, preserving all #if/#else contexts and -D overrides."""
    lines = text.splitlines(keepends=True)
    stack, extracted, kept = [], [], []
    index = 0
    while index < len(lines):
        block = lines[index]; index += 1
        while block.rstrip('\r\n').endswith('\\'):
            block += lines[index]; index += 1
        d = re.match(r'\s*#\s*(\w+)\b(.*)', block)
        if not d:
            kept.append(block); continue
        kind, arg = d.group(1), d.group(2).strip()
        if kind in ('if','ifdef','ifndef'):
            stack.append([block, []])
        elif kind in ('elif','else'):
            stack[-1][1].append(block)
        elif kind == 'endif':
            stack.pop()
        if kind == 'define' and arg.split()[0] != guard:
            contexts = [f for f in stack if not (guard and re.match(r'\s*#\s*ifndef\s+'+guard+r'\b', f[0]))]
            extracted.append(''.join(f[0]+''.join(f[1]) for f in contexts) + block + '#endif\n'*len(contexts))
        else:
            kept.append(block)
    assert not stack
    cleaned = ''.join(kept)
    # Remove now-empty conditional wrappers; never remove #error checks.
    empty = re.compile(r'(?m)^\s*#(?:if|ifdef|ifndef)[^\n]*\n(?:(?:[ \t]*\n)|(?:[ \t]*#(?:else|elif)[^\n]*\n))*[ \t]*#endif[^\n]*\n')
    while True:
        newer = empty.sub('', cleaned)
        if newer == cleaned: break
        cleaned = newer
    return cleaned, extracted

texts = {p.name:p.read_text(encoding='utf-8-sig') for p in (BEFORE/'App').glob('*.c')}
assert set(texts) == {s for members in GROUPS.values() for s in members}
renames = {}
for group, members in GROUPS.items():
    counts = collections.Counter(n for f in members for n in static_names(texts[f]))
    for f in members:
        renames[f] = {n:f.removesuffix('.c')+'_'+n for n in static_names(texts[f]) if counts[n] > 1}
        local = {'HALF_TICK_RANGE':'APP_TICK_HALF_RANGE', **local_macros.get(f,{})}
        texts[f] = rename(texts[f], local)

config_sections = []
seen = {}
def collect(file, blocks):
    unique = []
    for block in blocks:
        definitions = re.findall(r'^\s*#define\s+(\w+)\s*(.*)', block, re.M)
        assert len(definitions) == 1, (file, block)
        name, value = definitions[0]
        if name in seen and seen[name] == block:
            continue
        if name in seen and not block.lstrip().startswith(('#if','#ifndef')):
            raise AssertionError(('conflicting central macro',name,file))
        seen[name] = block
        unique.append(block.rstrip())
    if unique:
        config_sections.append('/* 原 '+file+'：常量值和条件编译语义保持原样。 */\n'+'\n'.join(unique))

for p in sorted((BEFORE/'App').glob('*.h')):
    if p.name == 'app_config.h': continue
    original = p.read_text(encoding='utf-8-sig')
    guard = re.search(r'^#ifndef\s+(\w+)', original).group(1)
    cleaned, blocks = extract_macros(original, guard)
    collect(p.name, blocks)
    if '#include "app_config.h"' not in cleaned:
        cleaned = cleaned.replace('#define '+guard, '#define '+guard+'\n#include "app_config.h"',1)
    (APP/p.name).write_text(cleaned, encoding='utf-8', newline='\n')
for f in texts:
    texts[f], blocks = extract_macros(texts[f])
    collect(f, blocks)

config = (BEFORE/'App/app_config.h').read_text(encoding='utf-8-sig')
assert config.rstrip().endswith('#endif')
config = config.rstrip()[:-len('#endif')].rstrip()
config += '\n\n/* ==================== 集中常量：原头文件/实现文件 ====================\n * 此处以下的协议端点、故障位和时间回绕常量通常无需调节。\n * 所有宏只在本文件定义；各头文件仅保留防重复包含宏。\n */\n\n'
config += '\n\n'.join(config_sections)

# Previously hard-coded boot defaults are also user-editable in this one file.
defaults = {
    'APP_FEEDER_STEP_DEG':('40.0f','每发输出轴目标增量，deg'),
    'APP_FEEDER_GEAR_RATIO':('35.68f','转子角度 / 输出轴角度'),
    'APP_FEEDER_DIRECTION':('-1','转子发射方向，-1 或 +1'),
    'APP_FEEDER_CALIBRATED':('1U','既有上电校准标志'),
    'APP_FEEDER_POSITION_KP':('10.0f','位置环比例增益'),
    'APP_FEEDER_SPEED_KP':('2.0f','速度环比例增益'),
    'APP_FEEDER_SPEED_KI':('8.0f','速度环积分增益'),
    'APP_FEEDER_MAX_RPM':('3600.0f','单发目标转子转速上限，rpm'),
    'APP_FEEDER_MAX_FEEDBACK_RPM':('6000.0f','转子反馈保护上限，rpm'),
    'APP_FEEDER_CONTINUOUS_MAX_RPM':('5800.0f','连发目标转子转速上限，rpm'),
    'APP_FEEDER_CONTINUOUS_BRAKE_KP':('30.0f','连发边界制动增益'),
    'APP_FEEDER_CONTINUOUS_PHASE_MS':('0.0f','连发跟踪相位偏移，ms'),
    'APP_FEEDER_SINGLE_DECEL_RPM_S':('50000.0f','单发减速率，rpm/s'),
    'APP_FEEDER_CURRENT_LIMIT':('3000.0f','C610 协议电流原始值上限'),
    'APP_FEEDER_TOLERANCE_DEG':('1.5f','输出轴到位误差带，deg'),
    'APP_FEEDER_SETTLE_RPM':('30.0f','到位静止转速阈值，rpm'),
    'APP_FEEDER_SETTLE_MS':('10U','到位保持时间，ms'),
    'APP_FEEDER_MOTION_TIMEOUT_MS':('3000U','一次运动超时，ms'),
    'APP_FEEDER_JAM_RPM':('10.0f','堵转低速阈值，rpm'),
    'APP_FEEDER_JAM_CURRENT_RAW':('1600.0f','堵转电流原始值阈值'),
    'APP_FEEDER_JAM_MS':('300U','堵转持续时间，ms'),
    'APP_YAW_LEGACY_KP':('1.0f','保留单环对照默认 Kp'),
    'APP_YAW_LEGACY_KD':('0.015f','保留单环对照默认 Kd'),
    'APP_YAW_FF_LIMIT_NM':('0.10f','原 Yaw 前馈默认限幅，N*m'),
    'APP_YAW_CASCADE_LEGACY_VELOCITY_DPS':('180.0f','受限双环对照默认速度上限，deg/s'),
    'APP_CASCADE_OBSERVER_MAX_GAP_MS':('100U','共用 PID 反馈间隔上限，ms'),
    'APP_CONSOLE_LINE_BYTES':('64U','一条命令的缓冲长度，byte'),
    'APP_CONSOLE_TELEMETRY_BYTES':('1024U','一帧遥测的格式化缓冲，byte'),
    'APP_CONSOLE_TX_TIMEOUT_MS':('(APP_YAW_ENABLED ? 120U : 60U)','原串口阻塞发送超时，ms'),
    'APP_YAW_DISPLAY_DEFAULT_ENABLED':('1U','上电启用带标记的 Yaw 演示显示'),
    'APP_PITCH_DISPLAY_DEFAULT_ENABLED':('1U','上电启用带标记的 Pitch 演示显示'),
}
config += '\n\n/* ==================== 上电默认值（数值保持不变） ==================== */\n'
config += '\n'.join('#define '+name+' '+value+' /* '+note+' */' for name,(value,note) in defaults.items())
config += '\n\n#endif /* APP_CONFIG_H */\n'
(APP/'app_config.h').write_text(config, encoding='utf-8', newline='\n')

def change(file, before, after):
    assert before in texts[file], (file,before)
    texts[file] = texts[file].replace(before,after)
feeder_fields = ['step_deg','gear_ratio','direction','calibrated','pos_kp','speed_kp','speed_ki','max_rpm',
    'max_feedback_rpm','continuous_max_rpm','continuous_brake_kp','continuous_phase_ms','single_decel_rpm_s',
    'current_limit','tolerance_deg','settle_rpm','settle_ms','motion_timeout_ms','jam_rpm','jam_current_raw','jam_ms']
for field, macro in zip(feeder_fields,list(defaults)[:21]):
    value = defaults[macro][0]
    change('feeder_controller.c', 'c->config.'+field+' = '+value+';', 'c->config.'+field+' = '+macro+';')
change('yaw_controller.c','yaw->pid.kp = 1.0f;','yaw->pid.kp = APP_YAW_LEGACY_KP;')
change('yaw_controller.c','yaw->pid.kd = 0.015f;','yaw->pid.kd = APP_YAW_LEGACY_KD;')
change('yaw_controller.c','yaw->ff.limit_nm = 0.10f;','yaw->ff.limit_nm = APP_YAW_FF_LIMIT_NM;')
change('yaw_cascade.c','YAW_CASCADE_OUTPUT_LIMITS_ENABLED ? 180.0f : 0.0f','YAW_CASCADE_OUTPUT_LIMITS_ENABLED ? APP_YAW_CASCADE_LEGACY_VELOCITY_DPS : 0.0f')
change('yaw_cascade.c','gap_ms >= 100U','gap_ms >= APP_CASCADE_OBSERVER_MAX_GAP_MS')
change('yaw_cascade.c','gap < 100U','gap < APP_CASCADE_OBSERVER_MAX_GAP_MS')
change('app_console.c','static char line[64], telemetry[1024];','static char line[APP_CONSOLE_LINE_BYTES], telemetry[APP_CONSOLE_TELEMETRY_BYTES];')
change('app_console.c','APP_YAW_ENABLED ? 120U : 60U','APP_CONSOLE_TX_TIMEOUT_MS')
change('app_console.c','YawDisplaySim_Reset(&yaw_display, 1U);','YawDisplaySim_Reset(&yaw_display, APP_YAW_DISPLAY_DEFAULT_ENABLED);')
change('app_console.c','PitchDisplaySim_Reset(&pitch_display, 1U);','PitchDisplaySim_Reset(&pitch_display, APP_PITCH_DISPLAY_DEFAULT_ENABLED);')

def header(file):
    title,purpose,functions,flow,constraint = INFO[file]
    return ('/**\n * @file '+file+'\n * @brief '+title+'\n *\n * 职责：'+purpose+'\n * 主要函数：\n'+
        ''.join(' *   - '+f+'\n' for f in functions)+' * 调用关系：'+flow+'\n * 维护约束：'+constraint+
        '\n * 参数入口：app_config.h；对外声明仍在原头文件中。\n */\n\n')

for group,members in GROUPS.items():
    sections = []
    for f in members:
        body = rename(texts[f],renames[f]).strip()+'\n'
        # Includes remain at the original section boundary; this preserves feature guards.
        sections.append('/* APP_SECTION_BEGIN '+f+' */\n'+body+'/* APP_SECTION_END '+f+' */\n')
    (APP/group).write_text(header(group)+'\n'.join(sections), encoding='utf-8', newline='\n')
for old in sorted(set(texts)-set(GROUPS)):
    path = APP/old
    assert path.resolve().parent == APP.resolve()
    path.unlink()  # Exact snapshotted source file, never a recursive operation.

manifest = {'format':1,'description':'Production section ownership; tests select sections to keep their existing HAL mocks.',
            'modules':GROUPS,'private_renames':renames}
(FW/'Tests/app_source_map.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(HERE/'source_map.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

# Update original VS Code/EIDE and Keil source lists, not a second project.
path = FW/'MDK-ARM/.eide/eide.yml'
yml = path.read_text(encoding='utf-8-sig')
pattern = r'(    - name: App\n      files:\n)([\s\S]*?)(      folders: \[\])'
yml,n = re.subn(pattern,lambda m:m[1]+''.join('        - path: ../App/'+f+'\n' for f in GROUPS)+m[3],yml)
assert n == 1
path.write_text(yml,encoding='utf-8',newline='\n')
path = FW/'MDK-ARM/Gimbal_Control.uvprojx'
xml = path.read_text(encoding='utf-8-sig')
entries = ''.join('            <File>\n              <FileName>'+f+'</FileName>\n              <FileType>1</FileType>\n              <FilePath>../App/'+f+'</FilePath>\n            </File>\n' for f in GROUPS)
xml,n = re.subn(r'(<GroupName>App</GroupName>\s*<Files>\n)[\s\S]*?(          </Files>)',lambda m:m[1]+entries+m[2],xml)
assert n == 1
path.write_text(xml,encoding='utf-8',newline='\n')
path = FW/'MDK-ARM/build/Gimbal_Control/builder.params'
params = json.loads(path.read_text(encoding='utf-8-sig'))
old_sources = params['sourceList']
params['sourceList'] = ['../App/'+f for f in GROUPS]+[p for p in old_sources if '/App/' not in p.replace('\\','/')]
assert len(params['sourceList']) == len(old_sources)-len(texts)+len(GROUPS)
path.write_text(json.dumps(params,ensure_ascii=False,indent=4)+'\n',encoding='utf-8')
print(json.dumps({'before_c_files':len(texts),'after_c_files':len(GROUPS),'moved_macros':len(seen),'new_default_macros':len(defaults)},ensure_ascii=False))
