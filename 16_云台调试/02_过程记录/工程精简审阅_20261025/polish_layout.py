from pathlib import Path
import re,json
REC=Path(__file__).resolve().parent;FW=REC.parents[1]/'Output/Gimbal_Control';APP=FW/'App';TESTS=FW/'Tests'

# Group complete preprocessor units. All validation-only #if blocks are kept
# after definitions, preserving their checks and avoiding forward evaluation.
p=APP/'app_config.h';old=p.read_text(encoding='utf-8')
body=old.split('#define APP_CONFIG_H\n',1)[1].rsplit('#endif',1)[0]
body=re.sub(r'/\* 参数导航[\s\S]*?\*/','',body)
body=re.sub(r'/\* 原 [\s\S]*?\*/','',body)
body=re.sub(r'/\* =+[\s\S]*?\*/','',body)
lines=body.splitlines(keepends=True);i=0;pending=[];units=[]
while i<len(lines):
    if not lines[i].lstrip().startswith('#'):
        pending.append(lines[i]);i+=1;continue
    start=i
    if re.match(r'\s*#if',lines[i]):
        depth=1;i+=1
        while depth:
            if re.match(r'\s*#if',lines[i]):depth+=1
            elif re.match(r'\s*#endif',lines[i]):depth-=1
            i+=1
    else:
        while lines[i].rstrip().endswith('\\'):i+=1
        i+=1
    unit=''.join(lines[start:i]);comment=''.join(pending).strip();pending=[]
    names=re.findall(r'(?m)^#define (\w+)',unit)
    units.append((names,comment,unit))
groups={key:[] for key in ['模式与设备开关','任务与通信','遥控与业务节拍','Yaw 参数与边界','Pitch 参数与补偿','拨盘参数与保护','摩擦轮参数与保护','共用常量','配置合法性检查']}
feature={'APP_YAW_ENABLED','APP_PITCH_ENABLED','APP_FEEDER_ENABLED','APP_FLYWHEEL_JOG_ENABLED','APP_GIMBAL_REMOTE_STARTUP_ENABLED','APP_MOTOR_DRIVERS_IMPLEMENTED','APP_REMOTE_PROFILE_ASSESSMENT','APP_REMOTE_PROFILE_LEGACY','APP_FLYWHEEL_SPEED_ACTIVE','YAW_CONTROLLER_CASCADE_ENABLED','YAW_CASCADE_OUTPUT_LIMITS_ENABLED'}
for names,comment,unit in units:
    name=names[0] if names else ''
    if not names:key='配置合法性检查'
    elif name in feature:key='模式与设备开关'
    elif name.startswith(('RC_','APP_REMOTE_PROFILE_VERSION','APP_STEP_')):key='遥控与业务节拍'
    elif name.startswith(('APP_YAW_','YAW_','GL60_')):key='Yaw 参数与边界'
    elif name.startswith(('APP_PITCH_','PITCH_','DM3507_')):key='Pitch 参数与补偿'
    elif name.startswith(('APP_FEEDER_','FEEDER_','APP_FIRE_')):key='拨盘参数与保护'
    elif name.startswith(('APP_FLYWHEEL_','FLYWHEEL_','APP_FIRMWARE_VERSION')):key='摩擦轮参数与保护'
    elif name.startswith(('APP_CONTROL_','APP_REMOTE_','APP_CONSOLE_','APP_CAN_','APP_INPUT_','APP_TELEMETRY_','REMOTE_','IBUS_')):key='任务与通信'
    else:key='共用常量'
    if name in ('FLYWHEEL_OBSERVE_COAST_ERPM','FLYWHEEL_OBSERVE_RESUME_ERPM'):continue
    if 'VESC 查询的回包地址' in comment:comment='/* 双 CAN 为 1 Mbps；CAN2 同时连接 C610，按实际总线配置维护。 */'
    if name=='APP_FLYWHEEL_JOG_MAX_DURATION_MS':continue
    if name.startswith('APP_YAW_POSITION_') and '2026-10-20' in comment:
        comment='/* Yaw 上电双环参数；串口调参仅修改本次 RAM 会话。 */'
    groups[key].append((comment+'\n' if comment else '')+unit)
header='''/**
 * @file app_config.h
 * @brief App 唯一参数与常量入口
 * 分区：模式、任务通信、遥控、Yaw、Pitch、拨盘、摩擦轮、共用常量。
 * MS=毫秒，DEG=度，DPS=度/秒，NM=牛·米；ERPM 与机械 RPM 不同。
 * 宏在重新编译/烧录后生效；串口调参仍只修改运行中的 RAM。
 * 协议端点/故障位由对应协议维护；#ifndef 参数保留构建覆盖能力。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H
'''
text=header
for index,(key,items) in enumerate(groups.items(),1):
    if not items:continue
    text+=f'\n/* ==================== {index}. {key} ==================== */\n\n'+'\n'.join(items)
text+='\n#endif /* APP_CONFIG_H */\n'
text=re.sub(r'\n{3,}','\n\n',text)
p.write_text(text,encoding='utf-8',newline='\n')

# Backend inhibition remains a useful explicit caller input. It no longer
# denotes the retired serial diagnostic mode; argument order/behavior is same.
for p in APP.rglob('*'):
    if p.suffix not in ('.c','.h') or p.name=='app_config.h':continue
    s=p.read_text(encoding='utf-8')
    s=re.sub(r'\bdiagnostic\b','inhibited',s)
    s=s.replace('"DIAGNOSTIC"','"INHIBITED"')
    s=s.replace('诊断查询','其他会话').replace('Yaw或诊断','Yaw 或其他电机').replace('DIAG取消邮箱','撤权取消邮箱')
    s=s.replace('0=legacy axis view','0=axis angle view').replace('Explicit legacy preparation/stop','Explicit preparation/stop')
    s=s.replace('yaw_controller.c；支持既有单环对照和生产串级 PID，开关/限位见 app_config.h。','yaw_controller.c；只使用生产串级 PID，参数和限位见 app_config.h。')
    s=s.replace('定义位置基准、PID、轨迹/前馈和反馈连续性、机械边界保护。','定义位置基准、双环状态、反馈连续性和机械边界保护。')
    for obsolete in [r'/\* Yaw平滑阶跃专用：[\s\S]*?\*/\n',r'/\* 显式重新设零后保留参数、关闭加速度前馈[\s\S]*?\*/\n']:
        s=re.sub(obsolete,'',s)
    if p.suffix=='.h':
        names=re.findall(r'(?m)^(?!static\b)(?:[A-Za-z_]\w*[ *]+)+([A-Za-z_]\w*)\s*\([^;{}]*\)\s*;',s)
        names=[n for n in names if n not in ('if','while')]
        if names:s=re.sub(r' \* 主要接口：[^\n]*',' * 主要接口：'+', '.join(names[:5])+(' 等' if len(names)>5 else '')+'。',s)
        impl=p.with_suffix('.c')
        if impl.exists():
            s=re.sub(r' \* 实现/调用：[\s\S]*?\n(?= \* 参数：)',' * 实现：'+p.parent.name+'/'+impl.name+'；调用关系见实现文件与 App/README。\n',s,count=1)
    p.write_text(re.sub(r'\n{3,}','\n\n',s),encoding='utf-8',newline='\n')

p=TESTS/'test_yaw_telemetry.c';s=p.read_text(encoding='utf-8').replace('columns[48] == 1 && columns[49]','columns[48] == 0 && columns[49]');p.write_text(s,encoding='utf-8')
p=TESTS/'test_yaw_cascade_unlimited_console.c';s=p.read_text(encoding='utf-8').replace('"YAW VMAX 180", "Y|ERROR|OUTPUT_LIMITS_DISABLED"','"YAW VMAX 180", "Y|ERROR|UNKNOWN_COMMAND"');p.write_text(s,encoding='utf-8')
print('Grouped config without changing retained parameter values; updated file navigation')
