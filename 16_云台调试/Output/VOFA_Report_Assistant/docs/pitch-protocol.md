# 当前 Pitch 串口协议核对

核对日期：2026-10-22。依据当前 `Output/Gimbal_Control/App/pitch_console.c`、`app_console.c` 和 `app_config.h`，未修改固件。版本字符串为 `DM3507_FAST_OBSERVER_V2F`。

## 分帧与遥测

串口收到的是字节流，不能把一次接收回调当成一条报告。先缓存不完整行，按换行结束的完整行解析；保留接收顺序和原文。`CRLF` 可以跨接收块，一块也可能包含多行。

当前 FireWater 数值组为 `pitch:`、`gimbal:`、`feeder:`。仅“已知组名 + 完整合法数值字段”的行被隐藏于报告正文，仍交给 VOFA 波形处理。未知文字、格式损坏行、含非数值说明的同名行均保留，不因频繁出现被丢弃。Pitch 当前 65 列，周期 5 ms；插件不更改组名、列索引、波形通道或固件发送周期。

状态行和回复可能任意穿插。等待报告的判定只看该命令的已知中间/终结消息，不能用“发送后的下一行”、固定行数或“串口一段时间没有消息”代替。周期遥测也不能无限延长等待。

## 查询与动作的回复

| 命令 | 中间回复 | 终结回复与注意事项 |
| --- | --- | --- |
| `PITCH` | 无 | `P\|STATE\|…` |
| `PITCH STATUS` / `PITCH INFO` | 无 | `P\|STATUS\|…\|QUERY\|SNAPSHOT_ONLY\|…`，状态快照 |
| `PITCH PID` | 无 | `P\|PID\|POS\|…\|VEL\|…`，RAM 参数 |
| `PITCH HELP` / `PITCH ?` | 无 | `P\|HELP\|…` |
| `PITCH CHECK` | `P\|BUSY\|CHECK\|QUERY\|CC\|…` | `P\|CHECK\|…\|QUERY\|NEW_FEEDBACK/TIMEOUT\|…`，或 `P\|ERROR\|CHECK\|…`。板端准备超时 1000 ms；`TIMEOUT` 是查询失败，不能显示为检查成功 |
| `PITCH GUARD` | 无 | `P\|GUARD\|…`；保护变化也主动发同格式，归属待确认 |
| `PITCH RESULT` | 无 | `P\|STEP\|ID\|n\|STATE\|…`；异步成绩同格式，归属待确认。查询时 `STATE` 可以是 `IDLE/RUNNING/DONE/ABORTED` |
| `PITCH FF` | 无 | `P\|FF\|KG\|…` |
| `PITCH SHAPE` | 无 | `P\|SHAPE\|MS\|…` |
| `PITCH SHAPEV` | 无 | `P\|SHAPEV\|…` |
| `PITCH BRAKE` | 无 | `P\|BRAKE\|MODE\|…` |
| `PITCH READY` | `P\|BUSY\|READY\|QUERY\|CC\|…` | `P\|OK\|READY\|…` / `P\|OK\|ALREADY_READY\|…` / `P\|ERROR\|READY\|…`；会准备参考与请求状态，非只读 |
| `PITCH STEP [目标角]` | `P\|OK\|STEP_STARTED\|TARGET\|…\|WINDOW_MS\|5000\|…` | 5 秒观测完成后 `P\|STEP\|ID\|n\|STATE\|DONE/ABORTED\|…`；ACK 没有 ID，不能声称严格关联。范围/准备错误以 `P\|ERROR\|STEP_…` 结束。`RUNNING` 不是本次阶跃终结 |
| `PITCH HOME` | 无 | `P\|OK\|HOME\|TARGET\|0` 或 `P\|ERROR\|HOME_…`；ACK 只表示目标被接受，不表示回零已完成 |
| `PITCH TARGET x` | 无 | `P\|OK\|TARGET\|…` / `P\|ERROR\|TARGET_…`，只代表接受目标 |
| `PITCH POS kp ki kd` / `PITCH VEL kp ki kd` | 无 | `P\|OK\|POS/VEL` / `P\|ERROR\|PID_RANGE_OR_FORMAT` |
| `PITCH PKP/PKI/PKD/VKP/VKI/VKD value` | 无 | `P\|OK\|PARAMETER` / `P\|ERROR\|PID_RANGE_OR_FORMAT`；ACK 不包含参数名或新值 |
| `PITCH KG x` / `PITCH FF OFF` | 无 | `P\|OK\|KG_SET\|…` / `P\|ERROR\|KG_…` |
| `PITCH GOFF x` / `PITCH GHORIZ` | 无 | `P\|OK\|HORIZONTAL_SET\|…` / `P\|ERROR\|GOFF_…` |
| `PITCH ZERO` | 无 | `P\|OK\|SOFTWARE_ZERO\|…` / `P\|ERROR\|ZERO_…` |
| `PITCH COORD 1/7` | 无 | `P\|OK\|COORD\|…` / `P\|ERROR\|COORD_…` |
| `PITCH OFF` | 无 | `P\|OK\|OFF_REQUESTED\|…`；卸载负载需现场承托，ACK 不等于机械刹车 |
| `PITCH VIEW` / `TELEM PITCH` | 无 | `P\|VIEW\|PITCH\|GROUP\|pitch\|COLS\|65\|…`；会改变波形数据组，不自动发送 |
| `TELEM OFF/ON` | 无 | `C\|OK\|TELEM\|OFF/ON`；改变数值遥测发送，不属于默认快捷按钮 |

准备操作未完成时，其他写命令可能返回 `P|BUSY|CHECK/READY|USE_PITCH_OFF_TO_CANCEL`，不能把它误认成目标命令开始。无效 Pitch 命令返回 `P|ERROR|UNKNOWN_COMMAND|USE|PITCH HELP`。

## 主动事件与不可消除的歧义

- `P|GUARD|…`：保护追踪/停止信息主动上报，也会由 `PITCH GUARD` 查询产生。
- `P|REMOTE_STEP|SEQ|…|RESULT|STARTED/RETURN_ZERO/START_GUARD|…`：遥控触发事件，不是任意正在等待命令的 ACK。
- `P|STEP|ID|…`：串口阶跃或遥控阶跃终结主动上报，也会由 `PITCH RESULT` 查询产生。
- `R|RX|…`、`C|UART_FAULT|…`、其他轴 `Y|…/F|…` 及启动 `G|…` 必须保留为主动消息。错误/故障突出显示。

当前协议没有请求编号。一次只等待一条命令可以减少歧义，但无法解决延迟旧回复、其他发送入口、遥控动作和相同格式主动上报的来源冲突。插件必须显示“归属待确认”，保留原始行；后到回复也不能悄悄并入下一份报告。超时/结束等待都不代表 MCU 动作已取消。

## 必需验证用例

1. `PITCH CHECK` 的 BUSY 和 CHECK 之间插入多条 `pitch:`、`R|RX|…`、`P|GUARD|…`，报告仍完整。
2. 每个字节切片、跨 `CRLF` 切片、多个完整行一次到达，结果相同。
3. `PITCH STEP` ACK 后穿插遥测，等待到 DONE/ABORTED，不在 ACK/RUNNING 结束。
4. GUARD、STEP 与查询同格式时显示归属不确定。
5. 健康状态中 `FAULT|0` 不因出现单词 FAULT 就被显示为故障；非零故障和 ERROR 需醒目。
6. 读取旧报告时新事件只增加未读数；内存和持久化均有上限。
7. 接收能力未通过 VOFA 运行探针前，不能把模拟测试通过宣传为实机可用。
