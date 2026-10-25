"""Add reviewed function descriptions and file inventories, without code edits."""
from pathlib import Path
import re,json,hashlib,textwrap
REC=Path(__file__).resolve().parent;APP=REC.parents[1]/'Output/Gimbal_Control/App'
inventory=json.loads((REC/'function_inventory.json').read_text(encoding='utf-8'))
globals_by_file=json.loads((REC/'global_inventory.json').read_text(encoding='utf-8'))
descriptions={}
def entries(text):
    for line in text.strip().splitlines():
        name,desc=line.split('|',1);descriptions[name.strip()]=desc.strip()
entries('''
FormatYawDisplayStatus|格式化 Yaw 显示演示的模式、模板和真实角度通道说明。
FormatPitchDisplayStatus|格式化 Pitch 显示演示的模式、模板和真实角度通道说明。
TransmitFailed|累计串口发送故障代次，并请求统一停机以撤销旧输出。
SendText|在控制台任务中发送文本；发送失败时记录故障并撤销运行请求。
PitchViewBlock|读取 Pitch 保持会话及全局许可，返回当前不能锁轴的具体原因。
PublishViewHold|发布视图锁轴状态、固定锁点、当前误差及阻塞原因。
PublishRemoteRx|发布遥控接收计数和故障证据，可只报告新发生的故障。
PublishWheelStatus|发布两侧摩擦轮实测状态、当前选择、动力请求和停机状态。
PublishOneSpeed|格式化一侧摩擦轮的目标、滤波转速和速度环运行统计。
PublishWheelSpeed|发布两侧摩擦轮速度控制状态及当前选中轮的统计信息。
SelectWheel|处理摩擦轮串口停止请求；当前正式遥控配置不接受串口选轮启动。
PublishStartup|按查询或新事件发布双轴准备阶段、参考有效性和阻塞原因。
StartupRequest|把当前遥控快照提交给双轴重新准备流程，并建立新的停机屏障。
HandleLine|分发一条完整命令，处理视图切换和锁轴后交给对应电机控制台。
AppConsole_CreateQueue|建立命令接收队列并初始化显示状态和控制台运行标记。
StartConsoleRx|按配置启动 USART1 的 DMA 接收或单字节中断接收。
AppConsole_Start|启动控制台接收，失败时按原故障路径记录并请求重新接收。
AppConsole_GetFaultEpoch|返回控制台累计故障代次，供控制任务识别旧命令和撤权。
AppConsole_GetViewHoldMask|返回当前视图要求保持的另一轴掩码。
HAL_UART_RxCpltCallback|分流 UART 接收完成事件，将控制台字节入队或交给遥控接收模块。
HAL_UARTEx_RxEventCallback|分流串口 DMA 接收事件，将本批有效字节按接收时间入队。
HAL_UART_ErrorCallback|分流串口错误，记录故障并通知相应接收模块恢复链路。
PublishPitchTelemetry|采集 Pitch 实测快照，按既有 VOFA 列顺序发布角度、控制量和成绩。
PublishWheelTelemetry|发布两侧摩擦轮实测及目标转速，保留既有速度视图通道。
PublishYawTelemetry|采集 Yaw 实测快照，按既有 VOFA 列顺序发布角度、控制量和成绩。
PublishTelemetry|按输出周期和当前视图选择对应遥测，控制发送频率。
AppConsole_Poll|恢复接收故障、组装完整命令，并轮询准备回执和定时遥测。
ConsoleParse_Number|严格解析一个有限浮点数，拒绝空输入和数值后的多余字符。
ConsoleParse_PidNumbers|严格解析一组 Kp、Ki、Kd，供串口参数范围校验使用。
DisplayDemoJitter_Node|由节点编号和种子生成确定的显示扰动节点值。
DisplayDemoJitter_Band|在相邻扰动节点之间插值，生成连续的显示波动。
DisplayDemoJitter_Offset|合成并限制演示显示的角度扰动，结果不进入真实控制。
DisplayStepWatch_Update|只读跟踪实测目标变化，记录首次入带时间和实际超调。
PitchConsole_Init|初始化 Pitch 控制台的准备流程及已报告故障编号。
PitchConsole_CancelPreparation|取消尚未完成的 Pitch 查询或准备操作，不启动新的动作。
RemoteReason|核对遥控有效性、解锁和回中条件，返回当前操作的拒绝原因。
StopName|把 Pitch 停止原因编号转换为可读名称。
GuardReport|格式化全局撤权证据、遥控代次和控制周期等运行信息。
Summary|汇总 Pitch 会话、协议反馈、坐标、PID 与重力标定状态。
GravityReport|格式化重力补偿、PID 各分量、制动和实际总输出的只读快照。
SafeOffReason|核对安全 OFF、新鲜反馈和遥控回中条件，返回准备操作的阻塞原因。
SafeOff|判断当前是否满足 Yaw 准备操作要求的安全 OFF 条件。
YawConsole_CancelPreparation|取消尚未完成的 Yaw 准备流程，避免旧操作继续接管会话。
PreparationTick|按反馈和超时推进 Yaw 停止、监测、参考捕获及请求准备流程。
SafeStep|核对当前遥控、解锁与回中条件是否允许串口阶跃。
StepBlockReason|把 Yaw 阶跃门禁位转换为优先显示的拒绝原因。
ClearReport|格式化 Yaw 清错条件、停止确认和最近一次清错结果。
BusReport|发布双 CAN 收发计数、邮箱占用及首次错误证据，不改变输出许可。
PitchDisplaySim_Rise|按选定模板插值显示进度，形成起步抬升、中段爬升和末端收敛。
winds_up|判断本次积分增量是否会继续加重已饱和的输出。
YawCascade_Stop|清除双环控制历史和积分，保留参数及已有速度观测。
YawCascade_ResetObserver|清除速度、加速度观测及对应控制历史。
YawCascade_Init|载入默认双环参数并初始化独立的控制与观测状态。
YawCascade_Observe|利用有效位置增量和接收时间间隔更新速度、加速度滤波值。
YawCascade_SetPid|校验并更新指定环的 PID；参数改变时重置控制历史并递增版本。
YawCascade_Update|以直接位置误差执行双环控制，不额外添加前馈力矩。
update|执行位置环和速度环计算，按新反馈推进积分并处理总力矩抗饱和。
YawCascade_UpdateWithFeedforward|执行双环控制，把指定前馈力矩纳入求和与抗积分饱和。
YawCascade_UpdateReference|执行带参考速度的双环控制，并按调用要求冻结位置积分。
PendingOwned|汇总指定 CAN 总线上属于某个执行者的待发邮箱掩码。
CancelOwned|只取消指定执行者拥有的待发邮箱，保持其他电机发送归属。
SubmitOwned|提交专用 CAN 帧并记录邮箱所有者，供停止和完成中断核对。
HasTwoFreeMailboxes|检查同一 CAN 控制器是否能完整提交摩擦轮双零帧。
RememberFirstError|保存当前总线首次错误的来源、时间和硬件错误码。
RecordError|累计总线错误和故障代次，并保存首次错误证据。
AbortOutputs|撤销双 CAN 已提交的输出，配合故障处理取消旧动力命令。
HasCanFault|汇总双 CAN 的软件及硬件故障状态。
AbortFlywheelOutputs|仅撤销摩擦轮拥有的输出，保留云台和拨盘的独立发送归属。
BusIndex|把 CAN 句柄映射为内部总线下标，拒绝不属于本工程的句柄。
CanBus_CreateQueue|创建 CAN 接收帧队列，供接收中断与 CAN 任务交接。
CanBus_Start|配置双 CAN 过滤器、启动接收和通知，并记录启动失败。
CanBus_Receive|从接收队列取出一帧，可按指定 RTOS 等待时间阻塞。
CanBus_GetState|复制 CAN 启动状态、帧计数、丢帧和故障代次。
CanBus_GetDiagnostics|复制 CAN 运行计数、硬件错误和邮箱等只读诊断快照。
CanBus_SetOutputPermission|更新摩擦轮输出许可；独立拨盘模式下保持轮动力关闭。
CanBus_OutputAllowed|查询摩擦轮输出许可是否仍满足启动及总线健康条件。
CanBus_SetYawPermission|更新 Yaw 专用发送许可，撤权时取消该轴旧输出。
CanBus_YawAllowed|查询 Yaw 专用发送许可及总线健康状态。
SendYawFrame|通过 Yaw 专属发送路径提交固定协议 CAN 帧。
YawSendResult|统一处理 Yaw 发送结果；真正的传输错误进入总线故障路径。
CanBus_YawEnable|在许可有效时提交 Yaw 固定使能帧，不开放任意协议报文。
CanBus_YawTorque|校验和限幅 Yaw 力矩，生成纯力矩帧并受专属许可约束提交。
CanBus_YawDisable|撤销 Yaw 许可并按需取消旧帧，提交固定失能帧。
CanBus_YawTxPending|读取当前归属于 Yaw 的待发邮箱掩码。
CanBus_YawClearComms|在原清错门禁允许时提交 Yaw 固定通信错误清除帧。
CanBus_SetPitchPermission|更新 Pitch 专用发送许可，撤权时取消该轴旧输出。
CanBus_PitchAllowed|查询 Pitch 专用发送许可及总线健康状态。
SendPitchFrame|通过 Pitch 专属发送路径提交固定协议 CAN 帧。
PitchSendResult|统一处理 Pitch 发送结果；真正的传输错误进入总线故障路径。
CanBus_PitchEnable|在许可有效时提交 Pitch 固定使能帧。
CanBus_PitchTorque|校验和限幅 Pitch 力矩，按固定协议编码后受专属许可约束提交。
CanBus_PitchDisable|撤销 Pitch 许可并按需取消旧帧，提交固定失能帧。
CanBus_PitchReadFeedback|提交 Pitch 固定状态查询帧，供未使能时监测及准备流程使用。
CanBus_PitchTxPending|读取当前归属于 Pitch 的待发邮箱掩码。
CanBus_CancelPitchTx|只取消 Pitch 归属的待发帧，返回邮箱是否已经清空。
CanBus_SetFeederPermission|更新拨盘专用电流许可，撤权时取消拨盘旧输出。
CanBus_FeederAllowed|查询拨盘专用许可及 CAN 健康状态。
SendFeederCurrent|把拨盘电流原始值编码到 C610 固定控制槽并提交。
CanBus_FeederCurrent|检查拨盘许可和电流边界后提交 C610 电流，不占用其他设备槽位。
CanBus_StopFeeder|撤销拨盘许可，取消该设备旧输出并提交零电流。
SendJogCurrent|按 VESC 固定电流协议编码指定摩擦轮的 mA 请求。
RecordJogTxFailure|记录摩擦轮传输错误并使原输出进入故障撤权路径。
CanBus_JogVescCurrent|校验摩擦轮节点、电流及许可后，发送固定 VESC 电流命令。
CanBus_StopFlywheels|取消摩擦轮旧输出，并在邮箱足够时完整提交两侧零电流。
CanBus_KeepFlywheelsStopped|维持两侧摩擦轮零电流，避免半组零帧或影响拨盘归属。
HAL_CAN_RxFifo0MsgPendingCallback|读取硬件接收 FIFO，附加接收时间和故障代次后入队。
HAL_CAN_ErrorCallback|记录 CAN 硬件错误并撤销对应旧输出。
TxComplete|核对硬件待发位后处理邮箱完成事件，保留已被复用的邮箱归属。
TxAbortComplete|处理邮箱取消完成事件，防止迟到中断清除新帧的归属。
CanBus_Send|始终拒绝任意 CAN 帧请求；该兼容接口没有动力发送实现。
CanFrame_IsValid|检查总线编号、标准/扩展标志、帧 ID 和数据长度是否有效。
encode|把物理量限幅并按给定端点量化为协议无符号整数。
decode|按协议端点把无符号整数还原为有符号物理量。
init_frame|初始化 DM3507 专用 CAN 帧头与负载。
Dm3507_Decode|识别 DM3507 有效反馈帧，解出位置、速度、力矩、状态和温度。
make_special|构造 DM3507 固定特殊命令帧，不提供任意设备写入入口。
Dm3507_MakeEnable|构造 DM3507 固定使能帧，发送许可由上层负责。
Dm3507_MakeDisable|构造 DM3507 固定失能帧。
Dm3507_MakeReadFeedback|构造 DM3507 固定反馈查询帧。
Dm3507_MakeTorque|按 DM3507 协议生成纯力矩帧，内部位置/速度增益保持为零。
EncodeSymmetric|对称限幅物理量并按给定量程编码为协议整数。
DecodeSymmetric|把协议整数按对称量程转换为物理量。
DecodeTemperature|把 GL60 温度字节转换为有符号摄氏温度。
InitCommand|初始化 GL60 II 专用 CAN 命令帧头。
Gl60_Decode|识别 GL60 II 有效反馈，解出实际位置、速度、力矩和状态信息。
Gl60_BuildTorque|生成 GL60 II 纯力矩控制帧，内部位置/速度增益保持为零。
BuildSpecial|构造 GL60 II 固定特殊命令负载。
Gl60_BuildEnable|构造 GL60 II 固定使能帧。
Gl60_BuildDisable|构造 GL60 II 固定失能帧。
Gl60_BuildClearError|构造 GL60 II 固定清错帧，实际清错门禁由后端检查。
ibus_drop_first|丢弃候选缓冲的首字节，继续在剩余数据中寻找有效帧。
ibus_find_header|在缓冲中定位 iBUS 帧头并保留可继续拼接的候选数据。
ibus_init|初始化 iBUS 解析状态、默认通道校验范围和接收统计。
ibus_set_channel_validation|校验并设置需要检查的通道数和通道数值范围。
ibus_poll_timeout|检查候选帧的字节间隔，超时后放弃未完成帧。
ibus_feed_byte|处理一个带时间戳的字节，完成帧头、通道和校验和核对后输出有效帧。
ibus_feed|依次把一批字节送入同一解析器，返回有效帧数量。
StartRemoteRx|按配置启动 UART4 的 DMA 或单字节中断接收。
RemoteInput_CreateQueue|创建遥控字节队列并初始化解析器与接收状态。
RemoteInput_Start|启动遥控接收，失败时记录错误并安排恢复。
RemoteInput_RxCplt|处理遥控单字节接收完成事件，附加接收信息后入队。
RemoteInput_RxEvent|把一批遥控 DMA 字节附加时间和故障代次后入队。
RemoteInput_Error|记录遥控串口错误，撤销旧数据可信度并安排重新接收。
RemoteInput_SyncFault|同步接收故障代次，使旧队列字节不能重新形成有效遥控输入。
RemoteInput_Poll|恢复遥控接收并解析队列字节，只发布通过校验的新鲜遥控帧。
RemoteInput_GetState|复制当前遥控通道、有效帧时间和接收故障统计。
Fresh|检查拨盘反馈是否新鲜，并与当前 CAN 故障代次一致。
RemoteGuard|按操作类型核对拨盘所需的遥控 OFF/解锁、回中和输入时效条件。
positions|由连续编码器计数和传动配置换算拨盘实际及目标角度。
reset_output|清除拨盘当前控制输出、速度参考及对应积分历史。
fresh|判断拨盘控制器最近一帧反馈是否仍在有效时间范围内。
Feeder_ConfigValid|检查拨盘传动、方向、PID、速度、电流、到位和堵转参数的合法范围。
Feeder_Init|载入拨盘默认配置，并初始化编码器、动作和故障状态。
Feeder_Stop|结束拨盘当前动作并清除控制输出，保留原配置和后续恢复所需状态。
Feeder_SetConfig|在控制器允许修改时应用合法配置，并同步必要的控制状态。
Feeder_OnFrame|解析 C610 反馈，进行解环、时效、跳变、速度和电流检查。
Feeder_Clear|在反馈健康且允许恢复时清除可恢复的拨盘故障。
Feeder_Zero|在原静止门禁允许时建立拨盘软件零点。
consume_event|按事件序号识别新单次请求，避免重复执行同一个发射事件。
Feeder_ContinuousRpm|按每发角度、传动比和连发间隔计算所需转子转速。
completed_step|记录一发完成后的用时、误差、超调及累计完成计数。
stream_crossings|依据实测连续角度统计连发过程中已经跨过的发射边界。
finish_stream|在释放连发时收回多余前瞻目标，保留已接受的尾步目标。
single_speed_limit|依据剩余角度和减速参数计算单发接近目标时的速度上限。
Feeder_Run|执行拨盘请求接收、单发/连发目标推进、位置速度控制和运行保护。
Feeder_Step|以非连续模式调用拨盘统一控制入口，保留单步调用兼容性。
age_ok|检查反馈年龄是否小于允许超时，并处理计时回绕。
newer|比较两个毫秒时间戳，判断是否属于更新的数据。
read_i32|按摩擦轮反馈协议读取有符号 32 位整数。
read_i16|按摩擦轮反馈协议读取有符号 16 位整数。
magnitude_greater|在不依赖数值符号的情况下判断幅值是否超过限制。
feedback_faults|从摩擦轮速度、电流和温度反馈中汇总运行故障位。
FlywheelJog_Init|初始化左右轮反馈、速度环及会话和停机状态。
FlywheelJog_Stop|结束当前摩擦轮运行请求并将两侧电流请求置零。
FlywheelJog_Fault|锁存摩擦轮故障位，并沿原停止路径清除动力请求。
synchronize_epoch|同步 CAN 故障代次，防止旧反馈继续支撑当前轮子会话。
damaged|对损坏或不符合预期的摩擦轮反馈锁存帧错误。
FlywheelJog_OnFrame|识别两侧 VESC 状态广播，更新反馈并立即检查运行故障。
FlywheelJog_GetHealth|根据两侧反馈时效、故障和转速汇总健康与停转状态。
FlywheelJog_Select|核对停止、健康和重新触发条件后选择运行的摩擦轮。
FlywheelJog_Step|按实时许可和输入推进摩擦轮速度控制、停止及待发电流请求。
FlywheelJog_GetRequestedCurrent|读取当前选中轮的有符号电流请求，用于状态发布。
FlywheelJog_GetNodeCurrent|按具体 VESC 节点读取对应左右轮的有符号电流请求。
FlywheelJog_TakeCommand|取出一条待发的摩擦轮电流请求，并消耗对应待发标记。
FlywheelJog_ClearFault|仅在关闭、回中且反馈健康停转时清除可恢复故障。
magnitude|计算转速或电流的无符号幅值，用于运行统计和边界比较。
add_saturated|累加统计计数，达到整数上限后保持饱和，避免回绕。
FlywheelSpeed_Init|载入现用摩擦轮速度环参数并初始化滤波、积分和统计状态。
FlywheelSpeed_Start|以当前反馈建立一次速度运行会话，初始化目标斜坡和计时。
FlywheelSpeed_Stop|清除当前速度控制请求与积分，并保留需要读取的运行统计。
FlywheelSpeed_RecordFeedback|记录新反馈对应的达速、带内保持、峰值和限流统计。
FlywheelSpeed_Step|推进现用速度斜坡和滤波 PI，结合软滑行与电流限制返回 mA 请求。
MotorBackend_Init|初始化摩擦轮执行状态、速度会话和双轮稳速互锁计时。
MotorBackend_GetJogState|复制两侧轮子反馈、控制请求、运行统计和停止状态。
MotorBackend_GetHealth|根据当前双轮会话和真实反馈生成系统所需的健康与稳速就绪状态。
MotorBackend_OnFrame|把有效轮子反馈交给会话控制器，并同步双轮稳速和故障事件。
MotorBackend_Stop|撤销摩擦轮运行请求，并管理双零提交及后续停止确认。
MotorBackend_PrepareRemote|依据新鲜安全 OFF 帧准备双轮会话；独立拨盘模式不开放轮动力。
MotorBackend_SelectJog|受原门禁约束更新轮子选择；独立拨盘模式只接受停止。
MotorBackend_TakeRearmRequest|取出并清除摩擦轮要求重新解锁的单次标记。
MotorBackend_Apply|把遥控轮子请求转为受许可约束的速度控制和 CAN 电流输出。
Online|核对本轴反馈时间与当前 CAN 故障代次，判断设备是否在线。
StopWithReason|记录 Pitch 停止原因并撤销该轴会话、许可和旧输出。
SetTarget|按目标接管类型检查 Pitch 门禁，接受目标并同步当前阶跃统计。
Notice|记录一条 Pitch 遥控动作接收或拒绝事件，供控制台一次性读取。
StationarySupport|判断原目标附近是否已形成可用于保持的稳定支撑状态。
TransferPositionSupport|在满足静态条件时转移原位置积分支撑，减少进入保持时的输出突变。
normalize_degrees|把角度归一到连续等价的正负 180° 范围。
observe_pitch|按 Pitch 独立滤波周期，由位置差分更新速度和加速度观测。
gravity_at|按实际角度及水平偏置计算余弦形式的重力补偿力矩，单位 N·m。
reset_shape|结束 Pitch 全程轨迹，使参考位置回到最终目标并清零参考速度。
reset_brake|清除当前运动的制动跟踪状态和附加力矩。
clear_brake_statistics|清除制动峰值和生效帧数，供下一次动作独立统计。
start_landing_trace|为一次 Pitch 运动建立首次到位与回退跟踪快照。
sample_landing_trace|根据新实测反馈记录落点速度、支撑积分、输出及最大回退。
terminal_brake|依据接近方向、实际速度和回退条件计算限幅且平滑变化的制动力矩。
update_shape|按已接受的运动时间推进 Pitch 五次轨迹的位置、速度和加速度参考。
PitchController_InvalidateGravity|清除水平标定有效性，避免未经确认的重力相位参与使能。
stationary_safe|检查 Pitch 的反馈、状态、温度和静止条件是否允许建立参考。
PitchController_SetFeedbackDivisor|仅接受合法的反馈轴侧换算，修改后使旧软件零点失效。
PitchController_InvalidateGravity|使 Pitch 水平重力标定失效，后续使能仍需满足原标定条件。
PitchController_SetShape|校验并设置 Pitch 全程轨迹时间和参考速度权重，不直接启动动作。
StartMove|统一接受 Pitch 离散目标，按普通运动或抢占归位建立轨迹、制动及落点跟踪。
PitchController_StartMove|以普通离散运动规则接受 Pitch 目标，保留原静止和轨迹门禁。
PitchController_ReturnTo|以可抢占方式接管 Pitch 归位目标，不额外重置原 PID 参数。
PitchController_SetBrakeEnabled|切换 Pitch 附加制动；关闭立即撤去附加力矩，开启由后续动作触发。
PitchController_SetGravityOffset|校验并设置水平角偏置，更新重力标定和必要的支撑状态。
PitchController_CaptureHorizontal|以当前已确认软件坐标建立水平重力基准。
PitchController_SetGravityGain|校验并更新重力补偿系数，同时按原规则处理支撑力矩衔接。
PitchController_UpdateViewHold|使用原双环、重力补偿和保护执行固定目标保持，不放大 PID 增益。
StepBlocks|汇总 Yaw 正式阶跃所需的会话、位置、静止、限位和视图门禁。
CaptureForStart|在安全静止时捕获启动参考，并尽可能保留人工零点以保持当前位置。
RemoteNotice|记录 Yaw 遥控动作的接收结果和门禁信息，供控制台报告。
sample_safe|核对 Yaw 最近反馈的状态、温度和静止速度是否适合建立参考。
AppFault_Fatal|处理不可恢复的应用初始化故障，使系统进入原致命故障路径。
AppLogic_CombineActuatorHealth|组合已请求云台轴与必要轮子的健康状态，形成统一运行许可输入。
AppLogic_Init|初始化应用、安全管理和目标节拍状态。
AppLogic_RecordStop|保存本次解锁后的首次撤权来源、输入和故障证据。
AppLogic_Step|映射有效遥控，推进安全状态，再生成当前手动、阶跃或发射请求。
AppTasks_RequireRearm|递增停机屏障序号，要求后续通过新的安全输入重新解锁。
AppTasks_RequestStop|统一撤销动力许可并通知四类电机后端停止，阻断旧请求继续输出。
AppTasks_GetState|在临界区内复制已发布的应用、遥控及安全状态。
RemoteTask|按固定周期轮询遥控接收和解析，不在本任务执行电机控制。
CanTask|接收有效 CAN 帧，按故障代次过滤后分发给对应电机后端。
ControlTask|按 2 ms 节拍统一采样、判定安全、同步锁轴并提交各电机控制输出。
ConsoleTask|按控制台周期处理串口命令、异步回执和 VOFA 输出。
AppTasks_Create|初始化各后端和接收队列，创建遥控、CAN、控制及控制台任务。
GimbalStartup_Init|初始化双轴上电准备阶段、参考有效性及重新解锁屏障。
SafeOff|检查启动流程所需的新鲜安全 OFF 遥控条件。
Barrier|更新时间屏障，要求后续使用更新的安全 OFF 帧重新准备。
CancelInitial|撤销尚未完成的双轴初始准备及对应会话请求。
GimbalStartup_ManualOverride|把准备流程控制权交给显式人工操作，避免自动流程覆盖人工请求。
GimbalStartup_Request|核对安全条件后提出双轴重新准备请求，等待原流程逐步完成。
GimbalStartup_GetState|复制双轴准备阶段、原因和参考状态，供只读查询。
GimbalStartup_TakeEvent|取出并清除一条新的双轴准备状态事件。
GimbalStartup_OutputReady|检查双轴参考和准备阶段是否允许后续运行解锁。
GimbalStartup_Update|按原安全门禁推进双轴停止确认、参考捕获和准备就绪状态。
GimbalStepSync_ApplyViewHold|把当前视图保持掩码分别提交给 Yaw 和 Pitch 后端。
GimbalStepSync_FilterViewRequest|屏蔽被锁轴的手动速度和阶跃事件，保留可见轴动作。
GimbalStepSync_Update|识别新接受的真实阶跃编号，更新最近测试轴并按需锁住另一轴。
SwitchPosition|把遥控通道值映射为上、中、下三个业务档位。
MapRate|按死区和速度标尺把摇杆通道转换为连续角速度请求。
RemoteMapper_Map|校验现用遥控通道，并映射 CH5 模式、CH6 发射、CH7 阶跃和 CH8 安全扳机。
Lock|撤销当前运行许可并建立安全屏障，要求新的完整解锁条件。
SafetyManager_Init|初始化操作员解锁、输出许可和故障代次观察状态。
SafetyManager_Step|综合遥控时效、故障和新帧边沿，推进解锁、撤权与重新解锁屏障。
TargetManager_Init|初始化目标管理器、发射节拍和最近一次测试轴。
TargetManager_RecordAcceptedTestAxis|只在轴后端接受阶跃后更新最近测试轴，供 CH7 回退使用。
Cancel|清除当前运动与发射请求及触发锁存，保留已经发布的事件序号。
RepeatShot|发布一条连发事件并推进 50 ms 节拍，迟到时不追补成突发请求。
AssessmentStep|按现用遥控模式生成手动、单次阶跃、单发或连发请求及边沿锁存。
TargetManager_Step|校验输入指针并推进现用遥控目标和发射节拍状态机。
''')

numeric={
    'absolute':'返回数值的绝对值，用于误差或速度幅值比较。',
    'abs_double':'返回双精度数值的绝对值，保留连续编码器计算精度。',
    'clamp':'把数值限制在指定上下界内，避免控制请求越界。',
    'finite_value':'检查浮点数是否有限，拒绝 NaN 和无穷值进入控制。',
    'within':'检查数值是否有限且处于指定闭区间。',
}
backend={
    'Init':'初始化{axis}控制、会话、动作和故障状态。',
    'GetState':'在同步保护下复制{axis}实测反馈、会话和控制状态。',
    'Stop':'撤销{axis}当前动作与发送许可，并管理原停止确认流程。',
    'Request':'更新{axis}会话请求；实际使能仍由实时许可、反馈及原握手条件决定。',
    'OnFrame':'接收{axis}有效新反馈，更新控制器、真实成绩和停止/故障状态。',
    'CaptureReference':'在关闭且满足原静止门禁时建立{axis}软件位置参考。',
    'SetLoopPid':'校验并更新{axis}指定环 PID，参数变更时同步控制历史和真实阶跃状态。',
    'SetLoopParameter':'读取{axis}指定环参数，仅修改选中的 Kp、Ki 或 Kd 后走统一校验入口。',
    'SetParameter':'处理 Yaw 直接目标参数；双环 PID 使用专用位置环/速度环接口。',
    'SetTarget':'在原门禁允许时接受 Pitch 位置目标，并同步真实动作统计。',
    'ReturnTo':'立即接管 Pitch 归位目标，允许中断当前规划而不额外改写 PID 参数。',
    'SetGravityGain':'在原状态门禁允许时修改 Pitch 重力补偿系数，并同步相关动作状态。',
    'SetGravityOffset':'在原状态门禁允许时更新 Pitch 水平偏置或捕获水平基准。',
    'SetCoordinate':'在关闭状态确认 Pitch 反馈坐标侧，并使旧参考在必要时失效。',
    'QueryFeedback':'请求一帧 Pitch 固定反馈，供关闭状态监测和准备使用。',
    'PrepareStop':'建立 Yaw 人工准备所需的停止监测和请求屏障，不直接使能电机。',
    'ReleasePreparation':'释放 Yaw 准备流程门禁，仍需后续的新鲜遥控解锁。',
    'StartStep':'检查{axis}阶跃门禁，接受目标并建立基于真实反馈的成绩记录。',
    'StepBlocks':'检查相对角度后返回 Yaw 阶跃所有阻塞条件的组合。',
    'GetStepId':'读取{axis}最近接受的真实阶跃编号，用于双轴同步。',
    'GetStep':'复制{axis}真实阶跃统计快照，不改变运动状态。',
    'TakeStepReport':'取出{axis}待报告的真实阶跃记录，避免重复发布同一结果。',
    'TakeRemoteStepEvent':'取出并清除一条{axis}遥控动作接收或拒绝事件。',
    'TakeRearmRequest':'取出并清除{axis}后端要求重新解锁的单次标记。',
    'HoldCurrent':'满足原保持条件后锁定{axis}当前实测位置，保留原 PID 和观测器。',
    'SetViewLock':'更新{axis}持续视图保持请求，实际锁点在健康且已解锁后应用。',
    'PrepareViewHold':'为 Pitch 视图保持恢复准备需求，固定锁点仍在满足原门禁后捕获。',
    'SetShape':'按原门禁配置 Pitch 可选全程轨迹时间和参考速度权重。',
    'SetBrakeEnabled':'按原动作门禁更新 Pitch 制动开关，不修改 PID 参数。',
    'Apply':'将{axis}目标请求与实时许可、反馈、使能及停止状态结合，计算并提交专用 CAN 输出。',
    'GetClearCheck':'复制 Yaw 清错门禁与最近一次清错尝试的只读证据。',
    'ClearCommsFault':'核对 Yaw 原清错门禁，允许时提交固定清错帧并跟踪恢复状态。',
}
feeder_backend={
    'Init':'初始化拨盘控制器、发送时序、准备流程和单次触发锁存。',
    'Stop':'撤销拨盘动力许可和待执行请求，启动原零电流停止流程。',
    'ManualOff':'处理拨盘人工关闭，防止自动准备立即覆盖显式停止意图。',
    'OnFrame':'接收拨盘有效反馈，并同步零点、故障和已完成动作信息。',
    'GetState':'复制拨盘控制器、反馈、许可和动作状态快照。',
    'TakeRearmRequest':'取出并清除拨盘要求重新解锁的单次标记。',
    'SetConfig':'在安全关闭条件下校验并设置拨盘运行配置。',
    'Clear':'在原安全门禁允许时清除拨盘可恢复故障。',
    'Zero':'在原静止且安全关闭条件下建立拨盘软件零点。',
    'Arm':'核对准备条件后请求拨盘待命，实际输出仍受实时许可约束。',
    'Ready':'按原安全关闭流程完成拨盘清错、软件设零和待命准备。',
    'PrepareRemote':'在新鲜安全 OFF 输入及静止反馈下自动准备拨盘，尊重人工关闭意图。',
    'Single':'核对单发门禁并锁存一次串口触发，不排队补发。',
    'Apply':'根据真实反馈、运行许可和请求序号执行拨盘单发/连发，并提交受限电流。',
}
step={
    'Init':'初始化{axis}真实阶跃统计及未到位标记。',
    'Start':'建立{axis}新阶跃的起点、最终目标、计时和判据快照。',
    'Command':'记录{axis}本轮实际提交的控制量、命令次数及首次输出。',
    'Abort':'用指定原因结束{axis}当前阶跃统计，保留中断及故障证据。',
    'Passed':'依据时间、超调和稳态误差判据判断真实阶跃是否合格。',
    'Sample':'接收{axis}真实反馈样本，更新首次入带、稳定时间、超调及稳态误差。',
}
controller={
    'Init':'载入{axis}默认双环参数，初始化参考、观测和故障状态。',
    'Stop':'停止{axis}当前控制并清理对应历史，保留原参数及安全参考规则。',
    'OnFeedback':'检查{axis}反馈有效性与连续性，更新实际角度、观测和运行故障。',
    'CaptureReference':'在原健康静止条件下建立{axis}软件零点，保留不可自动清除的故障。',
    'SetPid':'校验并更新 Pitch 指定环 PID，按原规则处理控制历史。',
    'SetTarget':'核对{axis}目标范围和参考有效性后，设置最终位置目标。',
    'Update':'在实时许可和反馈有效时执行{axis}控制，返回协议力矩；CAN 发送由后端负责。',
}
display={
    'Reset':'初始化{axis}显示演示状态及开关，保持与真实控制状态分离。',
    'Cancel':'取消{axis}当前显示过渡，真实反馈与真实成绩不被改写。',
    'Begin':'为{axis}新接受目标建立显示过渡起点和选定模板。',
    'Angle':'依据{axis}当前演示阶段生成显示角度；停止、失效或关闭时返回实测值。',
}
console={
    'Handle':'解析{axis}命令，核对格式和操作条件后查询状态或调用对应后端。',
    'Poll':'推进{axis}准备操作和异步事件，按需生成一条回执或真实结果。',
    'BriefStatus':'格式化{axis}精简状态，供高频查询读取且不改变运行状态。',
    'StepReport':'格式化{axis}真实阶跃成绩、控制参数和终止原因。',
}

specific={
    'console/feeder_console.c':{
        'Status':'格式化拨盘在线、准备、动作、反馈和控制配置状态。',
        'Result':'格式化拨盘真实单发/连发完成用时、误差、超调和累计计数。',
        'Check':'格式化拨盘故障及首次异常反馈证据，帮助定位跳变或堵转。',
        'FeederConsole_Telemetry':'按既有 VOFA 列顺序输出拨盘真实/目标角度、转速和控制状态。',
        'FeederConsole_SpeedTelemetry':'格式化拨盘转子/输出轴实际与目标转速，保留原速度视图通道。',
    },
    'system/gimbal_startup.c':{'Status':'更新双轴准备阶段、原因和就绪标记，状态变化时产生待报告事件。'},
    'console/yaw_console.c':{'SafeOff':'判断 Yaw 当前是否满足准备流程要求的安全 OFF 条件。'},
    'motors/feeder_controller.c':{'fault':'锁存拨盘运行故障，并清除当前控制输出和动作请求。'},
    'motors/pitch_controller.c':{'fault':'锁存 Pitch 控制故障并沿原停止路径清理当前输出。'},
    'motors/yaw_controller.c':{'fault':'锁存 Yaw 控制故障并沿原停止路径清理当前输出。'},
}

def description(file,name,occurrence):
    if file=='motors/motor_backend.c' and occurrence>1:
        return '提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。'
    if name in specific.get(file,{}):return specific[file][name]
    if name in descriptions:return descriptions[name]
    for key,desc in numeric.items():
        if name==key or name.endswith('_'+key):return desc
    if name.startswith('HAL_CAN_TxMailbox'):
        m=re.fullmatch(r'HAL_CAN_TxMailbox([012])(Complete|Abort)Callback',name)
        if m:return f'把 CAN 邮箱 {m[1]} 的'+('发送' if m[2]=='Complete' else '取消')+'完成中断交给归属核对逻辑。'
    if name.startswith('FeederBackend_'):return feeder_backend[name.split('_',1)[1]]
    for prefix,axis in [('YawBackend_','Yaw'),('PitchBackend_','Pitch')]:
        if name.startswith(prefix):return backend[name[len(prefix):]].format(axis=axis)
    for prefix,axis in [('YawStep_','Yaw/共用'),('PitchStep_','Pitch')]:
        if name.startswith(prefix):return step[name[len(prefix):]].format(axis=axis)
    for prefix,axis in [('YawController_','Yaw'),('PitchController_','Pitch')]:
        if name.startswith(prefix):return controller[name[len(prefix):]].format(axis=axis)
    for prefix,axis in [('YawDisplaySim_','Yaw'),('PitchDisplaySim_','Pitch')]:
        if name.startswith(prefix):return display[name[len(prefix):]].format(axis=axis)
    for prefix,axis in [('YawConsole_','Yaw'),('PitchConsole_','Pitch'),('FeederConsole_','拨盘')]:
        if name.startswith(prefix):return console[name[len(prefix):]].format(axis=axis)
    for prefix,axis in [('yaw_console_','Yaw'),('pitch_console_','Pitch')]:
        if name.startswith(prefix):return console[name[len(prefix):]].format(axis=axis)
    raise KeyError((file,name))

variable_notes={}
def variables(text):
    for line in text.strip().splitlines():
        names,desc=line.split('|',1)
        for name in names.split(','):variable_notes[name.strip()]=desc.strip()
variables('''
rx_queue|接收队列，连接中断生产者与任务消费者。
fault_epoch|累计接收/发送故障代次，用于隔离旧数据。
needs_rearm|接收需要重新启动的标记。
rx_dma,rx_byte|DMA 批量缓冲或单字节接收缓冲，按构建选择。
rx_bytes_seen,rx_bytes|累计接收字节数。
rx_bursts|累计 DMA 接收批次数。
rx_uart_errors,rx_errors|累计串口接收错误数。
rx_queue_drops,rx_drops|接收队列丢字节计数。
tx_errors|控制台发送失败计数。
last_uart_error|最近一次串口硬件错误码。
line|正在组装的完整命令行缓冲。
telemetry|控制台回执与 VOFA 格式化发送缓冲。
observed_epoch|任务已处理的故障代次。
last_byte_ms,last_rx_byte|最近一个接收字节的时间戳。
telemetry_ms|最近一次定时遥测的调度时间。
line_length|当前命令行已接受的字符数。
discard_line|丢弃当前不完整或受损命令直到换行的标记。
telemetry_enabled|当前是否发布定时波形。
speed_view|当前速度视图选择：拨盘或两侧摩擦轮。
view_hold_mask|当前视图请求保持的轴掩码。
feeder_view,pitch_view|当前角度视图选择标记。
reported_remote_epoch|最近已经向控制台报告的遥控故障代次。
yaw_display,pitch_display|独立的显示演示状态，不参与物理闭环或真实成绩。
pending_operation|待完成的 Pitch 查询/准备操作类型。
operation_ms|本次异步操作开始时间。
query_ms,query_rx|最近查询时间与已观察接收计数。
reported_guard_id,reported_stop_id|已经报告的全局撤权及 Pitch 停止事件编号。
ready_phase|Yaw 人工准备流程当前阶段。
ready_clear_sent|本次准备是否已经发送过固定清错帧。
ready_started_ms|Yaw 本次准备流程开始时间。
state|本模块维护并向外发布的设备或流程状态快照。
output_permission,yaw_permission,pitch_permission,feeder_permission|对应动力路径的独立 CAN 发送许可。
tx_owner|各 CAN 邮箱对应的执行者归属表。
parser|iBUS 持续解析器及有效帧统计状态。
rearm_errors|重新启动遥控接收失败的计数。
owned|本后端是否已经拥有过输出，供停止重试使用。
rearm,rearm_request,rearm_requested|需要操作员重新解锁的待消费标记。
was_allowed|上一控制周期的拨盘许可状态。
single_pending|待执行的一次串口单发请求。
have_completed|是否已有可用于后续门禁的完成动作。
last_tx_ms|最近一次控制命令的提交时间。
last_success_ms|最近一次成功提交拨盘输出的时间。
event_sequence|最近已处理的拨盘事件序号。
single_ms|串口单发请求接受时间。
completed_ms|最近一发动作完成时间。
auto_quiet_since|自动准备所需静止观察的起始时间。
auto_quiet|自动准备是否正在进行静止观察。
reference_captured|本次自动准备是否已经捕获软件参考。
manual_hold|显式人工关闭对自动准备的抑制标记。
jog|两侧摩擦轮反馈、速度环及运行会话状态。
initialized|后端初始化完成标记。
stop_pending|是否仍待完成原停止提交或确认。
stop_attempted|是否已经尝试提交摩擦轮双零。
last_stop_ms|最近一次停止重试时间。
wheel_stable_since|两侧轮子连续稳速观察的起始时间。
wheel_sample_ms|两侧最近一帧用于稳速检查的时间。
wheel_stable_mask|已经满足真实稳速条件的轮子掩码。
remote_primed|当前遥控周期是否已经准备好轮子会话。
step|当前动作基于真实反馈的阶跃成绩记录。
previous_step|新动作开始时保留的上一条未读或中断成绩。
remote_event,remote_step_event|待向控制台交付的遥控动作事件快照。
last_step_sequence|最近已经处理的遥控阶跃事件序号。
enable_ms,disable_ms|最近使能/失能握手的起始时间。
last_query_ms|最近一次 Pitch 固定反馈查询时间。
remote_pending,remote_event_pending|是否存在尚未读取的遥控动作事件。
cancel_pending|是否需要取消本轴尚未发送的旧命令。
disable_submitted|Pitch 固定失能帧是否已经提交。
actuation_started|本会话是否尝试过使能或观察到实际使能状态。
last_mode|上一控制周期的遥控云台模式。
submitted_torque,last_submitted_torque_nm|最近成功提交的实际控制力矩，单位 N·m。
submitted_limited,last_submitted_limited|最近提交力矩对应的饱和状态。
step_from_remote|当前阶跃保持是否来源于遥控动作。
monitoring|Yaw 当前是否允许进行关闭状态反馈监测。
sent|本会话是否已经提交过原使能命令。
preparation_hold|人工准备流程对正常运行的保持门禁。
clear_waiting_disable|清错前是否仍在等待可靠失能确认。
clear_stationary_samples|清错前连续静止反馈样本数。
clear_stationary_since|清错静止观察开始时间。
last_clear_blocks|最近一次清错尝试的阻塞条件。
last_clear_result,last_disable_result|最近清错/失能提交的 HAL 返回结果。
off_motion_faults|已经辨明发生在关闭状态下的运动故障位。
published|控制任务向其他任务发布的完整应用快照。
stop_sequence|全局停止与重新解锁屏障序号。
remote_sequence|启动流程最近已观察的有效遥控帧序号。
guard_stop,guard_uart,guard_can|启动流程已同步的停止、串口和 CAN 故障代次。
barrier_ms|启动流程的新鲜输入时间屏障。
off_since_ms|本次安全 OFF 连续保持的起始时间。
started_ms|当前双轴准备流程开始时间。
have_sequence,have_guard|是否已经建立遥控序号和故障代次观察基准。
off_seen|本次流程是否已经观察到有效安全 OFF 输入。
pending_event|准备流程是否存在新的待报告事件。
begun|初始双轴准备是否已经开始。
yaw_zero_done,pitch_zero_done|本次准备是否已完成对应轴的参考捕获。
clear_sent|本次双轴准备是否已经尝试过允许的 Yaw 固定清错。
manual_rezero|当前流程是否为人工请求的重新建立零点。
''')

def tokens(s):
    pattern=r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
    clean=re.sub(pattern,lambda m:m[0] if m[0].startswith(('"',"'")) else '',s,flags=re.S)
    return ''.join(re.findall(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|\S',clean))
def mask(s):
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda m:''.join('\n' if c=='\n' else ' ' for c in m[0]),s,flags=re.S)
PAT=re.compile(r'(?m)^(?:static\s+)?(?:inline\s+)?(?:[A-Za-z_]\w*[*\t ]+)+([A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{')
prepared={};audit=[];all_docs={}
for file,names in inventory.items():
    p=APP/file;s=p.read_text(encoding='utf-8');code=mask(s)
    matches=[m for m in PAT.finditer(code) if m[1] not in ('if','while','for','switch')]
    assert [m[1] for m in matches]==names,file
    counts={};edits=[];public=[];private=[]
    for m in matches:
        name=m[1];counts[name]=counts.get(name,0)+1
        desc=description(file,name,counts[name]);all_docs[file+'::'+name+':'+str(counts[name])]=desc
        (private if code[m.start():m.end()].lstrip().startswith('static ') else public).append(name)
        brace=m.end()-1;line_start=s.rfind('\n',0,brace)+1
        if not s[line_start:brace].strip():
            indent=s[line_start:brace];edits.append((line_start,brace,indent+'/* 功能：'+desc+' */\n'+indent))
        else:
            pos=brace
            while pos>0 and s[pos-1] in ' \t':pos-=1
            edits.append((pos,brace,'\n/* 功能：'+desc+' */\n'))
    for a,b,value in reversed(edits):s=s[:a]+value+s[b:]
    if p.suffix=='.c':
        old_header=re.match(r'/\*\*[\s\S]*?\*/\n',s);assert old_header,file
        brief=re.search(r'\* @brief ([^\n]*)',old_header[0])[1]
        header=['/**',f' * @file {p.name}',f' * @brief {brief}',' * 配置入口：App/app_config.h；接口声明见同目录对应头文件。']
        for title,fs in [('对外函数',list(dict.fromkeys(public))),('内部函数',list(dict.fromkeys(private)))]:
            header.append(f' * {title}（{len(fs)}）：')
            if not fs:header.append(' *   无。')
            else:
                for line in textwrap.wrap(' / '.join(fs),width=96,break_long_words=False,break_on_hyphens=False):header.append(' *   '+line)
        variables=[v for v in globals_by_file.get(file,[]) if v not in ('byte','ConsoleByte','RemoteByte')]
        header.append(f' * 文件级变量/常量（{len(variables)}；局部变量见对应函数）：')
        for name in variables:
            assert name in variable_notes,(file,name)
            header.append(' *   '+name+'：'+variable_notes[name])
        if not variables:header.append(' *   无；状态由调用方传入的结构体保存。')
        if file=='console/app_console.c':header.append(' * 文件级类型：ConsoleByte，携带命令字节的接收时间和故障代次。')
        if file=='drivers/remote_input.c':header.append(' * 文件级类型：RemoteByte，携带遥控字节的接收时间和故障代次。')
        if file=='drivers/can_bus.c':header.append(' * 文件级类型：TxOwner，区分各电机在发送邮箱中的归属。')
        if len(names)!=len(set(names)):header.append(' * 构建说明：模块关闭时，同名后端函数提供停用兼容实现。')
        header+=[' * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。',' */']
        s='\n'.join(header)+'\n'+s[old_header.end():]
    assert tokens(p.read_text(encoding='utf-8'))==tokens(s),'Executable text changed: '+file
    prepared[p]=s
    audit.append({'file':file,'function_definitions':len(matches),'executable_tokens_unchanged':True})
for p,s in prepared.items():p.write_text(s,encoding='utf-8',newline='\n')
(REC/'function_descriptions.json').write_text(json.dumps(all_docs,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(REC/'annotation_verification.json').write_text(json.dumps({'function_definitions':len(all_docs),'files':audit,'all_executable_tokens_unchanged':True},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Annotated',len(all_docs),'function definitions in',len(prepared),'files; executable tokens unchanged')
