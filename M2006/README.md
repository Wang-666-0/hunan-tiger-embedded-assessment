# M2006 + C610：Keil Debug 与 VOFA 入门

## 本次已完成的实测

- 板卡：Cherrycoast 4.8.1，原理图为 STM32F405RGT6、25 MHz 外部晶振。
- ST-Link 已识别，目标电压读数 3.29 V。
- 新工程编译通过：0 Error(s), 0 Warning(s)，烧录后校验成功。
- UART3 对应本机 **COM12，115200，8N1**。
- CAN1 已收到 **C610 ID1（标准帧 0x201）** 的持续反馈；静止转速为 0。
- 用户已通过 Keil Watch 触发首次点动，并确认电机实际转动。CAN 通信和短时驱动链路已通过实物初步验证；方向、实际速度、跟随误差与重复性尚待波形评估。
- 原程序的完整 1 MB Flash 备份：`backup/original_flash_20261025.bin`。
  SHA256：`8DAEDE391A2CC44A0779450E443570C3338B6074241F672F527857659E808F8A`。

工程入口：`MDK-ARM/M2006_Debug.uvprojx`。控制与打印：`firmware/main.c`。
变量定义及注释：`firmware/motor_demo.h`。观察列表：`MDK-ARM/watch_setup.ini`。

## 1. 先理解两个观察通道

ST-Link/SWD → Keil Debug → 直接查看或修改 MCU 内存变量。

UART3/USB转串口 → COM12 → VOFA → 查看固件主动发送的时序数据。

两条通道可以同时使用。Keil 中的 UART/Serial Window 不能自动代替电脑上的物理 COM12。
串口同一时刻只由一个程序打开；本次探测脚本已关闭端口，可以由 VOFA 接管。

原理图核对：CAN1 RX=PA11、TX=PA12；UART3 TX=PB10、RX=PB11。
UART3 接口 J6：1=TX，2=RX，3=GND，4=5V_USART。信号按 TX↔RX 交叉连接。
CAN1 板上有 120Ω 终端电阻；当前通信已通，不需要为此改接线。

## 2. Keil：先只看数据

1. 双击 `MDK-ARM/M2006_Debug.uvprojx`。
2. 按 **F7** 编译。工程使用 ARM Compiler 5.06；优化为 Level 0，保留调试信息。
3. `Options for Target → Debug` 选择 **ST-Link Debugger**；Settings 中选择 **SW、1 MHz**。
   不要同时用 CubeProgrammer 占用 ST-Link。工程已随附本机 F405 的 ST-Link 配置。
4. 如需重新下载，退出 Debug 后点击 Download。当前新固件已下载过。
5. **Ctrl+F5** 进入 Debug，通常会停在 `main()` 开头。此时外设还没有全部初始化。
6. **F5** 运行。打开 **View → Watch Windows → Watch 1**。
7. 勾选 **View → Periodic Window Update**，运行时也刷新变量。
8. 在 Watch 空行逐项输入 `g_control`、`g_debug`、`g_motor[0]`，点结构体左侧 `+` 展开。
   可选快捷方式：在 Debug 的 **Command** 输入框输入 `INCLUDE watch_setup.ini` 并回车，
   然后手动打开 Watch 1 / Watch 2。若相对路径找不到，输入该 ini 的完整路径并加双引号。

先看这些值：

| Watch 表达式 | 含义 | 当前正常表现 |
|---|---|---|
| `g_debug.selected_id` | 当前选择的电调 ID | 1 |
| `g_debug.online_mask` | 在线 ID 位图，bit0 对应 ID1 | 1 |
| `g_debug.rx_total` | 收到的有效反馈总数 | 持续增加 |
| `g_debug.feedback_age_ms` | 距最近反馈的时间 | 通常 0～数 ms |
| `g_debug.measured_rpm` | 电机转子的转速 | 静止时接近 0 |
| `g_motor[0].angle_raw` | ID1 转子的单圈编码器值 | 0～8191 |
| `g_debug.active` | 是否正在执行点动 | 0 |
| `g_debug.current_raw` | 当前发送的电流请求 | 0 |
| `g_debug.fault` | 锁存的保护故障位 | 0 |
| `g_debug.can_error` | 累计记录的 HAL CAN 错误 | 通信健康时为 0 |

`g_motor[0]` 对应 ID1，`g_motor[7]` 对应 ID8。数组索引比 ID 小 1。
Watch 是周期采样，1 ms 内发生的变化可能看不到；动作过程用 VOFA 看更清楚。
数值显示成十六进制时，在值上右键切换显示方式；输入十进制 360 即可，不要把 0x360 当成 360。

## 3. VOFA：接收并绘图

1. 数据接口选 **Serial/串口**，端口选 **COM12**。
2. 波特率 **115200**，数据位 **8**，停止位 **1**，无校验、无硬件流控。
3. 协议引擎选 **FireWater**，打开串口。固件每 20 ms 发一行，即 50 Hz。
4. 角度版固件接收文本形如 `motor:0,0,0,19,4329,1,1,0,0,1,190.24,0.00`，共 12 路。
5. 从控件栏拖入波形图，右键 → 绑定 Y 轴，先绑定 **通道 0 和 1**。
   另外一张图绑定 **通道 2**；编码器角度最好单独一张图，避免量程把转速曲线压扁。
6. 点 Auto 调整 Y 轴，时间滚动条拖到最右，显示最新数据。

通道从 0 开始编号；若界面显示第1路而不是 I0，按下表顺序对应。

| 通道 | 对应变量 | 单位/含义 |
|---|---|---|
| 0 / I0 | `g_debug.target_rpm` | 目标转子 rpm，空闲时为 0 |
| 1 / I1 | `g_debug.measured_rpm` | 实际转子 rpm |
| 2 / I2 | `g_debug.current_raw` | 电流请求 raw；1000 raw 对应 1 A 请求 |
| 3 / I3 | `g_debug.torque_raw` | 电调反馈转矩 raw，不作为标定后的实测电流 |
| 4 / I4 | `g_debug.angle_raw` | 转子单圈角度 0～8191 |
| 5 / I5 | `g_debug.selected_id` | 电调 ID |
| 6 / I6 | `g_debug.online_mask` | 在线位图 |
| 7 / I7 | `g_debug.active` | 点动中为 1 |
| 8 / I8 | `g_debug.fault` | 故障位 |
| 9 / I9 | `g_debug.feedback_age_ms` | 反馈年龄 ms |
| 10 / I10 | `g_debug.rotor_angle_deg` | 转子单圈角度，单位 °，0～小于360；过一圈回到0 |
| 11 / I11 | `g_debug.output_angle_deg` | 输出轴相对累计角度，单位 °，按标称36:1换算，可为负数或超过360 |

**观察电机输出轴转了多少角度，优先绑定 I11。** 另建一个波形图，右键 → 绑定 Y 轴 → I11，
点 Auto 调整量程；I4 保留原始编码器计数，I10 适合观察转子单圈回绕。
I11 从本次启动后的首次反馈起累计，初始为0，表示相对位移，不是输出轴绝对位置。
复位会重新起算；断开反馈或调试暂停期间可能漏计位移，不能据此保证断联前后的累计位置准确。
它仅用于调试观察，尚未用作位置闭环；减速比采用标称值36，未做输出轴实测标定。

角度版需重新下载才生效：确认电机已静止，Ctrl+F5 退出 Debug → F7 编译 → Download 下载，
再 Ctrl+F5 进入 Debug、F5 运行，并打开 Periodic Window Update。
Watch 中 `g_debug.version=0x20060102` 表示已运行角度版；若 VOFA 仍只有10路，先核对该版本号。

打印采用 `snprintf + HAL_UART_Transmit_IT`，发送时不阻塞控制循环。
改打印内容后需重新编译、下载；Watch 中改参数只修改 RAM，复位会恢复默认值。

## 4. 第一次由你在 Watch 触发点动

保持电机固定、空载、输出轴周围无干涉。先让程序保持 F5 运行，观察 ID1、反馈更新、fault=0。
展开 `g_control`，在 **Value** 栏双击修改：

| 字段 | 首次使用值 | 作用 |
|---|---|---|
| `motor_id` | 1 | 选择刚识别出的 C610 |
| `target_rpm` | 360 | 目标转子转速；按标称 36:1，约为输出轴 10 rpm |
| `duration_ms` | 1000 | 点动 1 秒 |
| `current_limit_raw` | 500 | 最高请求 0.5 A |
| `kp` | 0.8 | 速度 PI 的比例系数 |
| `ki` | 2.0 | 速度 PI 的积分系数 |
| `command` | **最后改为 1** | 开始一次点动 |

参数设置完成后才写 `command=1`。程序会把 command 消费并恢复为 0，这是正常现象。
`command_result=0` 表示命令被接受；`active` 应先变 1，1 秒后回到 0。
结束时发送零电流请求，电机会自由减速，不做位置保持，也不承诺立即机械停止。

在 VOFA 中，目标转速会出现 360 的平台，实际转速应跟随上升，结束后逐渐回到 0。
这组 PI 是首测参数，尚未完成实物调参；速度跟不上时，先看波形与故障，不直接提高限流。
待完全静止后，可用 `target_rpm=-360` 再点动一次观察反向。

立即停止请求：**程序运行时将 `g_control.command` 改为 2**，或在 VOFA 发送 `STOP` 加换行，
或单个字符 `!`。两种方式都会撤掉电流请求；机械紧急情况应直接切断电机供电。

串口也支持 `RUN 1 360 1000` 加换行，与 Watch 点动作用相同，本次学习优先练 Watch。

## 5. 断点、单步怎么练

先确认 `active=0`、`current_raw=0` 且电机静止，再暂停程序或设置断点。
**不要在电机出力时把 Keil 的 Stop/断点当作电机停机按钮。**
CPU 停下后软件限时、闭环和零电流发送也会停下，电调的动作不能靠 CPU 暂停来保证。

停机后，可在 `HAL_CAN_RxFifo0MsgPendingCallback()` 里设置断点，观察：

1. `h.StdId` 为 `0x201`，表示来自 ID1。
2. `d[0]` / `d[1]` 拼成 `angle_raw`。
3. `d[2]` / `d[3]` 拼成有符号转子 rpm。
4. **F10 Step Over** 跨过函数调用；**F11 Step Into** 进入函数；**F5** 恢复连续运行。

回调中的 `h`、`d` 是局部变量，停在函数内时在 **Call Stack + Locals** 看更方便。
离开该函数后局部变量可能显示 not in scope；全局 `g_motor` / `g_debug` 适合长期 Watch。
CAN 反馈很密集，断点会反复命中并可能引起接收 FIFO 溢出。练完先移除断点，
再复位并 F5 运行，重新确认通信正常后才点动。

## 6. 故障与命令返回值

`g_debug.fault` 是按位组合：1=反馈超时，2=转速过高，4=CAN 错误，8=控制循环空窗，
16=发送邮箱不可用，32=在线节点数量/所选节点变化，128=疑似堵转。
反馈超过 20 ms 未更新、转子超过 1800 rpm 等会撤掉点动请求并锁存故障。
故障原因排除、静止后写 `g_control.command=3` 清除；不要直接把 `g_debug.fault` 改成 0。

`g_debug.command_result`：0=已接受，1=正在执行、拒绝重复开始，2=参数非法，3=未准备好。
允许的点动目标为 ±720 转子 rpm，时长 100～2000 ms，电流请求上限 500 raw。
修改 `g_control` 的参数在下一次点动生效；当前动作使用开始时的参数快照。

## 资料与复现

- [本机原理图](D:/Project/27电软正式批考核/16_云台调试/03_参考资料/Cherrycoast4.8.1.pdf)，第1～2页。
- [DJI C610 官方手册](https://rm-static.djicdn.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf)，CAN 协议为 1 Mbps，反馈转速为转子 rpm。
- [Keil Watch 官方说明](https://www.keil.com/support/man/docs/uv4cl/uv4cl_db_dbg_watchwin.htm)。
- [VOFA FireWater 快速开始](https://www.vofa.plus/docs/learning/start/quick_start/)。

HAL/CMSIS 来自本机 STM32Cube F4 V1.28.3，保留原源码授权说明。
日常直接在 Keil F7 编译即可。`tools/prepare_project.py` 用于初次生成工程及复制驱动，
无需每次构建运行；它会重建 uvprojx，因此自行修改工程文件后不应随意重跑。
