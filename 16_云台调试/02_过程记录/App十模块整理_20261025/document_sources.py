"""Improve module/parameter navigation without changing executable statements."""
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
APP = HERE.parents[1] / 'Output/Gimbal_Control/App'

HEADERS = {
 'app_config.h':('App 统一配置','集中功能开关、PID 默认值、调度周期、遥控映射、机械边界和协议常量。','所有 .c / .h 均从这里取得宏；不包含业务头文件，避免循环依赖。'),
 'app_console.h':('控制台入口','声明命令接收、遥测轮询、诊断模式及视图锁定状态接口。','app_console.c；由控制台任务调用，接收回调只负责入队和错误记录。'),
 'app_diagnostics.h':('诊断会话接口','定义只读查询种类、结果事件及被动 CAN 帧快照。','app_diagnostics.c；Request/Poll 推进查询，TakeEvent 分段读取结果。'),
 'app_fault.h':('致命故障接口','定义队列、任务、UART 初始化失败等不可继续运行的故障入口。','app_tasks.c；AppFault_Fatal 进入 Error_Handler，不依赖损坏的 RTOS 环境重试。'),
 'app_logic.h':('应用逻辑与保护记录','定义应用状态、轴健康输入和停机原因追踪；组合安全判定与目标请求。','app_tasks.c；AppLogic_Step 先处理保护，再推进 TargetManager。'),
 'app_tasks.h':('任务及应用快照','声明任务创建、统一停机、重新解锁屏障及只读 AppState 快照。','app_tasks.c；AppTasks_GetState 在临界区内复制一致快照。'),
 'can_bus.h':('双 CAN 总线接口','定义总线状态和故障来源，声明接收、输出许可、电机命令及邮箱撤销接口。','can_bus.c；所有输出都经过许可和故障代次检查。'),
 'can_frame.h':('通用 CAN 帧','定义含总线、接收时间和故障代次的帧结构，并声明帧合法性检查。','can_bus.c；协议模块和后端共享同一 CanFrame 类型。'),
 'console_parse.h':('共用命令数字解析','实现不依赖设备状态的浮点数及 PID 三参数解析。','头文件内 static 函数；Yaw/Pitch 命令共用，拒绝非有限值和尾随垃圾。'),
 'display_demo_jitter.h':('带标记演示的连续扰动','生成连续、有界的不规则显示扰动，用于演示角度。','头文件内纯显示函数；不修改真实反馈、闭环状态或考核成绩。'),
 'display_step_watch.h':('实测阶跃只读观察','从真实角度记录首次进入误差带和超调，供显示状态查询。','头文件内观察器；正式成绩仍由 YawStep / PitchStep 给出。'),
 'dm3507.h':('DM3507 协议接口','定义 Pitch 电机反馈，声明使能、失能、读取反馈和力矩报文编码。','can_bus.c；角度 rad、速度 rad/s、力矩 N*m，协议端点集中在 app_config.h。'),
 'dm_diag.h':('达妙只读 RID 查询','声明允许读取的 RID 白名单及请求/回复匹配接口。','app_diagnostics.c；只读取参数，不写入设备或保存设置。'),
 'feeder_backend.h':('拨盘执行后端','声明拨盘会话、配置、准备、单发/连发、停机及只读状态接口。','feeder_controller.c；Apply 连接遥控请求、Feeder 控制器和 CAN。'),
 'feeder_console.h':('拨盘串口命令','声明 FEEDER 命令处理、角度遥测和速度遥测格式化接口。','motor_console.c；只负责校验/报告，运动授权仍由后端检查。'),
 'feeder_controller.h':('拨盘控制器数据与接口','定义编码器累计、位置/速度环参数、单发/连发状态和保护位。','feeder_controller.c；转子角度与输出轴角度分开，初始化默认值见 app_config.h。'),
 'flywheel_jog.h':('双摩擦轮受限会话','定义左右轮反馈、点动/持续会话、过流过速等保护及输出请求。','motor_backend.c；保留速度软滑行、硬保护及停机反馈确认。'),
 'flywheel_speed.h':('摩擦轮速度 PI','定义目标爬升、滤波、积分和限幅状态，声明速度环计算接口。','motor_backend.c；速度用 ERPM，输出用 mA，机械 RPM 需按极对数换算。'),
 'gimbal_startup.h':('双轴准备状态机','定义安全 OFF 准备阶段、拒绝原因和对外查询/更新接口。','app_tasks.c；不绕过新 OFF 帧、反馈健康及原使能握手。'),
 'gimbal_step_sync.h':('双轴阶跃与视图同步','同步实测阶跃编号、最近测试轴和视图保持请求。','头文件内协调函数；调用时序保留在控制任务的各原同步位置。'),
 'gl60_ii.h':('GL60 II 协议接口','定义 Yaw 电机反馈及力矩、使能、失能、清错报文接口。','can_bus.c；协议量使用 rad、rad/s、N*m，正负方向保持既有约定。'),
 'ibus.h':('iBUS 解析接口','定义解析器、有效帧和统计信息，支持按字节或块输入。','remote_input.c；处理帧头、校验和、通道范围、字节超时与丢字节重同步。'),
 'motor_backend.h':('双摩擦轮后端','声明双轮初始化、反馈、遥控准备、健康判定和停机接口。','motor_backend.c；向应用层提供 flywheels_ready，供拨盘互锁。'),
 'pitch_backend.h':('Pitch 执行后端','声明坐标标定、准备/使能、目标、归位、锁轴及实测结果接口。','pitch_controller.c；StartStep 保留门禁，ReturnTo 允许中断但保留健康和许可检查。'),
 'pitch_console.h':('Pitch 串口命令','声明 PITCH 命令、准备状态轮询及取消接口。','motor_console.c；参数合法后仍由 PitchBackend 再验证当前会话与安全状态。'),
 'pitch_controller.h':('Pitch 控制器数据与接口','定义串级 PID 接入、重力补偿、制动、轨迹和首次到位追踪。','pitch_controller.c；主要单位为 deg、deg/s、N*m，内部共用 YawCascade。'),
 'pitch_display_sim.h':('Pitch 带标记演示显示','定义演示轮廓、连续扰动及跟随目标的显示状态。','头文件内显示函数；关闭或健康失效返回真实角度，禁止进入控制与实测判定。'),
 'pitch_step.h':('Pitch 实测阶跃接口','复用 YawStep 数据与统计核心，声明 Pitch 参数和结果接口。','pitch_controller.c；判据集中在 app_config.h，与显示演示阈值分别保留。'),
 'remote_input.h':('遥控传输与快照','声明 UART4 DMA/字节回调、接收轮询、错误统计和只读状态。','remote_input.c；错误先撤权，恢复接收后仍需新安全帧解锁。'),
 'remote_mapper.h':('遥控语义映射','定义 RemoteFrame、RemoteCommand、开关/发射/云台模式并声明映射函数。','remote_input.c；通道从零编号，CH1 对应下标 0，物理档位见 app_config.h。'),
 'safety_manager.h':('统一安全状态机','定义安全输入、锁定原因、输出许可及人工重解锁屏障。','remote_input.c；SafetyManager_Step 使用真实链路和设备健康状态。'),
 'target_manager.h':('运动与发射目标管理','定义执行器请求、阶跃边沿序号及单发/连发目标节拍。','remote_input.c；目标请求不代表最终输出许可，后端仍逐项校验。'),
 'vesc_diag.h':('VESC 只读诊断协议','定义参数接收缓存，声明查询、分片拼接和 CRC 结果接口。','app_diagnostics.c；长度及节点/命令白名单验证保持原逻辑。'),
 'yaw_backend.h':('Yaw 执行后端','声明基准、使能、参数、相对阶跃、锁轴、停机及实测结果接口。','yaw_controller.c；普通阶跃保留静止门禁，归位使用明确的目标接管路径。'),
 'yaw_cascade.h':('两轴共用串级 PID','定义位置/速度环参数、观测器、前馈和抗积分饱和状态。','yaw_controller.c；虽保留 YawCascade 名称，Pitch 同样依赖本接口。'),
 'yaw_console.h':('Yaw 串口命令','声明 YAW 命令处理、准备状态轮询和准备取消接口。','motor_console.c；查询、显式配置和运动请求分别沿用原权限规则。'),
 'yaw_controller.h':('Yaw 控制器数据与接口','定义位置基准、PID、轨迹/前馈和反馈连续性、机械边界保护。','yaw_controller.c；支持既有单环对照和生产串级 PID，开关/限位见 app_config.h。'),
 'yaw_display_sim.h':('Yaw 带标记演示显示','定义显示轮廓、分段上升和目标附近的连续有界扰动。','头文件内显示函数；输出附带实测角度及演示标记，不进入正式成绩。'),
 'yaw_step.h':('共用实测阶跃统计','定义阶跃阶段、结束原因、测量快照及首次到位/稳定/超调判定。','yaw_controller.c；Yaw/Pitch 保留独立实例，只采真实反馈与实际提交命令。'),
}

for name,(brief,purpose,owner) in HEADERS.items():
    path = APP/name
    text = path.read_text(encoding='utf-8')
    # Remove pre-typedef parameter comments that were orphaned by centralization.
    if name in ('yaw_controller.h','pitch_controller.h','flywheel_jog.h','yaw_cascade.h','feeder_controller.h'):
        start = text.find('typedef')
        if start >= 0:
            text = re.sub(r'/\*[\s\S]*?\*/','',text[:start])+text[start:]
    # Avoid blank blocks left by moved macros.
    text = re.sub(r'\n(?:[ \t]*\n){2,}','\n\n',text)
    apis = list(dict.fromkeys(re.findall(r'^(?:static\s+)?(?:const\s+)?[\w\s*]+\s+([A-Za-z_]\w*)\s*\(',text,re.M)))
    apis = [a for a in apis if a not in ('if','while','switch','return')]
    api_note = '主要接口：'+', '.join(apis[:5])+(' 等。' if len(apis)>5 else '。') if apis else '主要内容：数据类型、状态枚举或配置宏。'
    doc = ('/**\n * @file '+name+'\n * @brief '+brief+'\n * 职责：'+purpose+'\n * '+api_note+
           '\n * 实现/调用：'+owner+'\n * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。\n */\n')
    path.write_text(doc+text,encoding='utf-8',newline='\n')

# File navigation: logical sections remain distinct inside each implementation.
section_notes = {
 'app_fault.c':'致命故障入口：交由 Error_Handler，避免在异常环境依赖 RTOS。',
 'app_logic.c':'应用纯逻辑：组合轴健康、安全状态和目标请求。',
 'gimbal_startup.c':'双轴启动协调：安全 OFF、建立基准和原使能握手。',
 'app_tasks.c':'RTOS 调度：队列、任务、全局停机和控制周期。',
 'app_console.c':'控制台传输、命令分发和遥测调度。',
 'yaw_console.c':'YAW 命令：独立准备流程、配置查询和实测报告。',
 'pitch_console.c':'PITCH 命令：坐标/重力、独立准备流程和实测报告。',
 'feeder_console.c':'FEEDER 命令：单发、连发、配置及角度/速度遥测。',
 'can_frame.c':'通用帧检查：与具体设备状态无关。',
 'gl60_ii.c':'GL60 II 协议编解码。', 'dm3507.c':'DM3507 协议编解码。',
 'can_bus.c':'CAN 传输：输出许可、邮箱所有权、故障代次及中断回调。',
 'dm_diag.c':'达妙 RID 白名单与只读查询编码。','vesc_diag.c':'VESC 只读查询、分片重组和 CRC。',
 'app_diagnostics.c':'诊断会话调度、取消、超时与结果交付。',
 'ibus.c':'iBUS 帧解析与错误后重同步。','remote_mapper.c':'遥控通道到业务语义的纯映射。',
 'safety_manager.c':'安全许可状态机与新 OFF 帧解锁屏障。','target_manager.c':'目标节拍、CH7 边沿与最近测试轴记忆。',
 'remote_input.c':'UART4 接收、队列消费及有效帧快照。',
 'flywheel_speed.c':'共用轮速 PI：目标爬升、观测、限幅及抗饱和。',
 'flywheel_jog.c':'双轮会话：反馈健康、方向、软滑行与停机确认。',
 'motor_backend.c':'双摩擦轮后端：遥控准备和应用层接口。',
 'feeder_controller.c':'拨盘控制数学：累计角度、单发减速、连续跟踪及保护。',
 'feeder_backend.c':'拨盘会话：发射互锁、单发/连发请求与 CAN 输出。',
 'yaw_cascade.c':'两轴共用 PID 数学：观测、双环、前馈与抗饱和。',
 'yaw_controller.c':'Yaw 控制数学：软件零点、目标、轨迹与保护。',
 'yaw_step.c':'共用真实反馈阶跃统计；不依赖显示演示。',
 'yaw_backend.c':'Yaw 会话：许可、握手、相对阶跃、归位与锁轴。',
 'pitch_controller.c':'Pitch 控制数学：坐标、重力补偿、制动和轨迹。',
 'pitch_step.c':'Pitch 实测统计适配：使用独立判定参数与实例。',
 'pitch_backend.c':'Pitch 会话：使能、水平标定、锁轴和可中断归位。',
}
for path in APP.glob('*.c'):
    text = path.read_text(encoding='utf-8')
    for name,note in section_notes.items():
        marker = '/* APP_SECTION_BEGIN '+name+' */\n'
        text = text.replace(marker,marker+'/* '+note+' */\n')
    path.write_text(text,encoding='utf-8',newline='\n')

# Correct stale column-count documentation and explain the physical signal path.
path = APP/'app_console.c'
text = path.read_text(encoding='utf-8').replace(
    '/* Pitch原I0~I57保持，I58/I59为末段附加力矩/active；I60~I64保留旧轨迹对照。',
    '/* Pitch：实测组65列，带标记的演示组68列；I58/I59为制动力矩/active。\n     * I60~I64为轨迹状态；演示组追加I65实测角度、I66演示active、I67 armed。')
path.write_text(text,encoding='utf-8',newline='\n')

path = APP/'pitch_controller.c'
text = path.read_text(encoding='utf-8').replace(
    '    pitch->gravity_torque_nm = gravity_at(pitch, pitch->gravity_kg_nm);',
    '    /* 重力项和末段制动在力矩层叠加；串级 PID 内统一限幅并处理抗饱和。\n     * 此处角度来自电机反馈，绝不使用控制台的演示角度。 */\n    pitch->gravity_torque_nm = gravity_at(pitch, pitch->gravity_kg_nm);')
text = text.replace('/* 用户确认写入的上电默认；不改变Yaw默认增益。 */',
                    '/* 共用 PID 状态初始化后，加载 Pitch 自身的默认增益。 */')
path.write_text(text,encoding='utf-8',newline='\n')

# Short parameter index at the single edit point; keep the original values.
path = APP/'app_config.h'
text = path.read_text(encoding='utf-8')
needle = '#define APP_CONFIG_H\n'
index = '''
/* 参数导航（Ctrl+F 搜索前缀即可）：
 *   APP_YAW_POSITION_ / APP_YAW_SPEED_     Yaw 双环 PID
 *   APP_PITCH_POSITION_ / APP_PITCH_SPEED_ Pitch 双环 PID
 *   APP_PITCH_GRAVITY_ / *_BRAKE_*         重力与制动
 *   APP_FEEDER_*                          拨盘上电默认值、单发/连发
 *   APP_FLYWHEEL_*                        摩擦轮速度、电流、极对数
 *   *_CONTROLLER_*LIMIT* / *_TIMEOUT_*     机械边界与反馈保护
 *   *_STEP_*                              真实阶跃考核判据
 *   APP_*PERIOD_MS / APP_*STACK_WORDS      任务周期和 RTOS 栈
 *   APP_CONSOLE_* / APP_TELEMETRY_*        串口、缓冲和遥测
 *   RC_* / IBUS_*                         通道映射及解析
 *   *_DISPLAY_*                           带标记的演示显示，独立于实测判定
 *
 * 单位后缀：MS=毫秒，DEG=度，DPS=度/秒，NM=牛·米，MA=毫安；
 * ERPM=电气转速，RPM=机械转速，C610 CURRENT_RAW=协议原始值。
 * 上电默认值在重新编译/烧录后生效；运行时串口设置只改变对应 RAM 状态。
 * 协议帧长度、编码端点、故障位和时间回绕常量应按协议维护。
 * #ifndef 包装的开关仍允许构建参数 -D 覆盖，历史回归配置继续有效。
 */
'''
text = text.replace(needle,needle+index,1)
text = text.replace('/* 独立视图紧凑FireWater/200Hz：Yaw70/Pitch58/拨盘39列；UART4仍115200。 */',
    '/* 当前视图以 FireWater 200Hz 输出；UART4 遥控接收保持 115200。 */')
text = text.replace(' * 所有宏只在本文件定义；各头文件仅保留防重复包含宏。',
    ' * 所有参数/常量宏只在本文件定义；各头文件仅保留防重复包含宏。')
path.write_text(text,encoding='utf-8',newline='\n')
print('Documented',len(HEADERS),'headers and 10 implementation files.')
