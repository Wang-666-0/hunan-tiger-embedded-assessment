# Gimbal_Control：云台调试参数采集版

本工程沿用上一份考核的平铺 App、配对 .c/.h、Core、Drivers、Middlewares、MDK-ARM 和 Tests 格式。当前支持遥控保护观察及USART1电机参数采集：读取DM3507和VESC配置、记录CAN反馈。完整操作见[USART1电机参数采集步骤](../../01_学习笔记/USART1电机参数采集步骤.md)。闭环控制和实机性能验收仍在下一阶段。

`App/app_config.h` 中 `APP_MOTOR_DRIVERS_IMPLEMENTED = 0`，`motor_backend` 返回未就绪，动力发送入口拒绝所有控制报文。独立白名单读接口只允许DM的0x33读操作，以及VESC命令0/4/14/17/91；不提供任意CAN发送入口。CAN为正常模式，会对合法接收帧产生硬件ACK，关闭自动重传。没有使能、设零、清错、写参数、检测电机或运动命令。

## 工程入口

- Keil：`MDK-ARM/Gimbal_Control.uvprojx`。
- EIDE：`MDK-ARM/Gimbal_Control.code-workspace`，目标 Gimbal_Control，ARM Compiler 5。
- CubeMX：`Gimbal_Control.ioc`；重新生成时保留 USER CODE，并检查 App 源文件分组。
- 模块说明：[App/README.md](App/README.md)。上板步骤：[基础版上板与VOFA观察](../../01_学习笔记/基础版上板与VOFA观察.md)。

在考核仓库根目录只编译、不下载：

```powershell
& '.\16_云台调试\Output\Gimbal_Control\Scripts\build.ps1'
```

构建脚本从 Keil 工程读取完整源文件列表，调用本机 EIDE 构建器和 ARM Compiler 5。工具路径不同可通过 `-ToolchainRoot`、`-BuilderPath` 指定。默认产物：`MDK-ARM/build/Gimbal_Control/Gimbal_Control.hex`。Keil 自身构建的目录为 `MDK-ARM/Gimbal_Control/`。

如果EIDE已在修改前加载本工程，其旧内存配置会自动保存并覆盖新增分组。首次在当前EIDE工程树App中右键“添加文件”，加入 `app_diagnostics.c`、`dm_diag.c`、`vesc_diag.c` 并保存，再执行“eide: 重新构建”。也可接受EIDE自身“项目文件已更改，重新加载项目”的提示；普通VSCode重载窗口可能先保存旧配置。已生成HEX、Keil源文件清单和 `Scripts/build.ps1` 不依赖该旧缓存，可以直接使用。

本机EIDE 3.27.2的“选项字节配置 / Option Bytes Config”是打开INI文件的按钮，没有None下拉选项；None是“外部加载算法”的值。本次无需操作选项字节配置。当前引用的 `.eide/gimbal_control.st.option.bytes.ini` 不存在，下载器不会附加选项字节写入命令；若之后点击该按钮创建了文件，应检查是否含有效键值，避免无意改变芯片配置。

## 当前外设和任务

| 项目 | 配置 |
| --- | --- |
| 主控 | Cherrycoast4.8.1，STM32F405RGT6，HSE 25 MHz、SYSCLK 168 MHz |
| UART4 | PA1 RX，新板P2上层pin4，115200 / 8N1，富斯i-BUS，单字节中断与带时间戳队列 |
| USART1 | PA9 TX、PA10 RX，115200 / 8N1，VOFA FireWater、STOP、只读采集命令 |
| CAN1 | PA11 RX、PA12 TX，正常模式，1 Mbps；GL60Ⅱ ID1、DM3507 ID2，实际配置待读取 |
| CAN2 | PB12 RX、PB13 TX，正常模式，1 Mbps；VESC 节点70/71、C610 ID5 |
| 输入任务 | 2 ms，优先级4；只解析遥控 |
| CAN接收任务 | 优先级4；最大2ms等待接收队列，处理只读查询、回复和超时 |
| 控制任务 | 2 ms，优先级3；连续间隔超过10 ms锁定故障 |
| 控制台任务 | 5 ms，优先级2；50 ms发送一次遥测 |

当前未接入IMU。4.8.1原理图是MPU6050加外置IMU FPC；须核对实际装配后选驱动，不能再默认板载BMI088。后续依据实际反馈频率确定控制周期、滤波和前馈，不沿用旧GM6020 PID参数。

## 遥控映射

当前默认 `APP_REMOTE_PROFILE_LEGACY = 1`，沿用第15项的遥控设置和接线：CH1输入、CH5总解锁、CH6失联许可。只校验这三路，CH1回中、CH6高位，再做CH5新的OFF→ON即可观察逻辑解锁；其他辅助通道不参与动作或回中检查。原Failsafe仍为CH1=1500、CH5=1000、CH6=1000，采集时CH5保持OFF即可。无需开启十路或重新绑定开关。

下面是保留给后续完整考核的可选映射。仅在明确设置 `APP_REMOTE_PROFILE_LEGACY = 0` 并核对遥控通道后使用；当前默认不会生成Pitch、摩擦轮、发射或阶跃请求。

| 通道 | 建议控件 | 行为 |
| --- | --- | --- |
| CH1 / CH2 | 自回中摇杆轴 | Yaw / Pitch角速度请求，最大±60 / ±45 °/s，中心死区±30通道值 |
| CH3 | 剩余摇杆轴 | 发射扳机；低位释放、高位触发，中位取消 |
| CH5 | SWA | 总解锁：低位OFF、高位ON；中位锁定 |
| CH6 | SWB | 独立Failsafe许可；运行高位、接收机失联输出低位 |
| CH7 | SWC | 低SAFE、中SINGLE、高CONTINUOUS；切换模式本身不发射 |
| CH8 | SWD | 摩擦轮请求：低关、高开；中位也关闭 |
| CH9 | VRA | 低手动、中Yaw阶跃、高Pitch阶跃 |
| CH10 | VRB | 低−30°、中零偏移、高+30°；每次新触发必须先回中 |

阈值：≤1300为低位，>1700为高位，其余中位。完整映射要求CH1..10在800..2200内；旧配置只要求CH1/5/6在此范围。

完整映射解锁前CH1/2回中，CH3/7/8/9低位、CH10中位、CH6高位，然后CH5先OFF再ON。上电已处于ON无法解锁；故障后收到的新OFF帧必须晚于故障时间，再收到新ON帧才可解锁。

阶跃模式禁止摩擦轮和拨盘；CH10触发只产生±30°请求事件。后续 `step_test` 须保存入模式时的参考角，并直接跳变目标。当前没有实测角度或阶跃成绩。

## VOFA 和 STOP

USART1输出 `gimbal:...\r\n`，VOFA选择 FireWater。下标从0开始，共29个通道：

| 下标 | 含义 |
| --- | --- |
| 0..9 | CH1..10原始通道值 |
| 10 | 合法i-BUS帧新鲜度；不能独立证明RF链路正常 |
| 11 / 12 | 遥控逻辑解锁 / 动力输出许可；基础版12始终为0 |
| 13 | SAFE=0、READY=1、ARMED=2、FAULT=3 |
| 14 / 15 | 当前保护原因 / 最后合法帧年龄ms（从未收到帧时年龄显示0） |
| 16 / 17 | Yaw / Pitch角速度请求°/s |
| 18 / 19 / 20 | 摩擦轮请求 / 发射模式0/1/2 / 发射事件累计数 |
| 21 / 22 / 23 | 云台模式0/1/2 / 阶跃偏移° / 阶跃事件累计数 |
| 24 / 25 | CAN1 / CAN2合法接收帧数，不能替代电机在线判定 |
| 26 | 电机后端就绪；基础版为0 |
| 27 / 28 | 遥控UART故障代次 / UART4接收字节累计数 |

原因码：0无、1上电、2帧失效/超时、3Failsafe、4UART、5CAN、6控制迟到、7STOP、8未回安全操纵位置、9解锁开关中位、10关键反馈失效、11执行器故障。10/11要等正式后端提供真实状态才能实机验证。

电脑发送大写 `STOP` 并带回车或换行，立即撤销CAN发送许可，在下一控制周期清除请求并锁定。没有电脑启动命令。恢复后仍须新的OFF→ON；发射扳机也须先释放。基础版的发射事件数始终为0，因为没有真实摩擦轮就绪反馈；50 ms逻辑已通过离线测试。

## 下一阶段补全位置

参数采集启动器为 `Scripts/capture.ps1`，电脑解码及保存放在 `Tools`。采集会暂停FireWater输出，结束后恢复；关闭VOFA串口后再运行。固件标识为`GIMBAL_DIAG_1`，每笔查询1200ms超时，电脑逐项等待并保存原始报文，完整VESC配置只在已知签名和长度匹配时解码。GL60Ⅱ驱动配置、机械方向/零点/限位、拨盘每发步距不能由本工具完整获取。

在 App 下增加独立的 GL60Ⅱ、DM3507、VESC、C610 驱动与控制模块，再通过 `motor_backend` 聚合。需补齐反馈在线、单位/方向/零点、软限位、拨盘传动参数、摩擦轮极对数、阶跃记录和停机命令。当前 `CanBus_Send` 在锁定时拒绝所有发送；后续必须设计只允许已验证的禁能/零给定报文的停机接口，并结合驱动通信超时验收，不能把“停止发送”当成机械已停止。

软件检查见 [Tests说明](Tests/README.md) 和 [软件验证记录](../../02_过程记录/软件验证记录.md)。软件通过后仍需按[实机验收记录](../../02_过程记录/实机验收记录.md)完成遥控保护专项，再开始动力调试。
