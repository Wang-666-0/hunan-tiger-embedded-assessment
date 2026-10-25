# VIEW窗口锁轴操作

本轮直接修改原 VS Code/EIDE 工程。功能标识为 `VIEW_AXIS_HOLD_V5`；拨盘仍保留 `C610_SINGLE_BRAKE_V1J` 的单发优化。烧录文件仍为当前 MDK-ARM 下的 `build/Gimbal_Control/Gimbal_Control.hex`。

## 使用

| 发送命令/点击原按钮 | 效果 |
| --- | --- |
| `PITCH VIEW` / `TELEM PITCH` | 显示pitch，锁住yaw当时实测位置，pitch沿用原控制和测试流程 |
| `YAW VIEW` / `TELEM YAW` | 显示yaw，锁住pitch当时实测位置，yaw沿用原控制和测试流程 |
| `GIMBAL VIEW LOCK` | 查询当前VIEW锁定请求 |
| `GIMBAL VIEW FREE` | 解除VIEW互锁，保留当前显示，返回原遥控/测试控制规则 |
| `FEEDER VIEW`、`FEEDER ANGLE VIEW`、`FEEDER SPEED VIEW`、`WHEEL VIEW`及其TELEM别名 | 切换对应曲线并解除VIEW互锁 |

**VOFA单独点击标签页不会向下位机发送命令。** 选择pitch/yaw标签页后，再点击该页原来的 `PITCH VIEW` / `YAW VIEW` 按钮，或在串口发送框发送相同命令并带换行；无需重做页面和通道绑定。

在原准备、解锁和反馈健康条件已满足时，锁定请求由原2ms控制任务执行。串口会在视图回执后再输出一行，例如：

```text
G|VIEW_LOCK|VIEW_AXIS_HOLD_V5|YAW|1|PITCH|0|MODE|HOLD_CURRENT|APPLY_MS|2
```

这里 `YAW|1` 表示请求锁yaw；`PITCH|1` 表示请求锁pitch。V2还附带pitch实际状态，发送 `GIMBAL VIEW LOCK` 可以检查：

- `PITCH_APPLIED|1`：pitch在线、活动且固定锁点已经捕获。
- `PITCH_PENDING|1`：等待满足原准备/解锁条件后捕获锁点。
- `PITCH_HOLD_DEG`：独立保存的锁点；`PITCH_ACTUAL_DEG`：真实反馈角度；`PITCH_TARGET_DEG`：控制器目标；`PITCH_ERROR_DEG`：锁点减真实角度。
- `PITCH_OUT_NM`：实际计算的控制输出；`PITCH_POSITION_GAIN|1` / `PITCH_SPEED_GAIN|1`：保持直接使用原PID，没有临时P/D放大。

VIEW命令首次回执时控制任务可能还未处理新请求，过一会查询才能区分“请求锁定”和“已应用”。该行不是新的通道数据，原角度曲线绑定保持不变。未解锁、故障或未准备时保持停止。V4在YAW VIEW持续选中、系统已正常解锁且全局动力许可有效时，为健康且已准备的pitch恢复保持会话；走原Request/FC/状态1反馈握手，之后捕获当前实测姿态，使用中不会直接追旧目标。

## V5：撤销临时增益放大并检查积分切换

实际启用保持后出现严重抖动，本轮先撤销加入的保持增益放大。V2到V4同时把位置P/D和速度P/D放大2倍，静态位置P纠偏斜率因此约4倍；现在直接调用原PID计算，不改用户配置的任何一个PID值。

同时取消无条件转移位置积分。运动中积累的位置积分不能被当成静止负载补偿保留下来；只有原静止条件与近目标条件满足才转移，减少切入保持时的力矩突变。V4的跨READY/重新解锁会话修复和独立固定锁点保留。

固件已离线检查原PID与保持输出等价、运动积分不转移、静态支撑仍保留、撤权/故障和原握手门禁。不能仅凭离线检查保证实机不抖。

重新烧录后先关闭演示显示以观察实测：

```text
YAW SIM OFF
PITCH SIM OFF
YAW VIEW
```

再查询`GIMBAL VIEW LOCK`，版本应为`VIEW_AXIS_HOLD_V5`，倍率均为1；完成保持仍以`PITCH_APPLIED|1`为准。

## V4：窗口保持需求跨准备/重新解锁保留

19:24回执里`ARMED=1`、`OUTPUT_ALLOWED=1`、`ONLINE=1`、`FAULT=0`，但`REQUESTED=0`、`PREPARE=0`，与“选YAW VIEW→执行READY/全局停止→重新解锁Yaw”的离线复现一致：V3的一次性准备请求被停止流程清除，视图没有切换就不会再设置。

V4让控制任务持续核对已选窗口的保持需求。全局STOP仍立即撤权、停止两轴；输出许可恢复前不会使能。用户按原流程正常OFF→ON重新解锁后，如果YAW VIEW仍选中，就自动通过原准备/使能握手恢复pitch保持，无需再次点击VIEW，也不因先VIEW后READY而丢失锁定请求。

若只通过后端单独停止pitch、同时全局许可仍有效，则保持需求标记为`PITCH_SUSPENDED=1`，不会立刻重新启动；新的YAW VIEW或正常的全局撤权/重新解锁周期才允许恢复。查询`GIMBAL VIEW LOCK`一直只读。

`PITCH_BLOCK|HOLD_SUSPENDED`表示这种单轴暂停；它区别于全局未解锁、故障、反馈超时和使能握手等待。已选视图的固定锁点在活动保持期间不反复采样，正常重新使能后捕获新鲜实测位置。

## 原V3路径及基础门禁

V2只设置VIEW锁定标志。如果此前用单轴READY/ON流程或单轴OFF使pitch.requested变成0，即使yaw还在正常工作，pitch也可能长期停在PENDING、OUT=0。

V3在原控制任务中消费新的VIEW准备请求：先通过原CC只读查询保持反馈新鲜；已有全局解锁/动力许可，且坐标、软件零点、重力标定、反馈、状态和故障门禁均满足时，才通过原使能握手恢复pitch。成功后捕获当时的真实角度，旧的−18°目标不沿用。重复点击YAW VIEW也可以发起这次准备；已经保持时重复点击不重新取锁点。

STOP立即停止并撤销动力许可。选中的VIEW需求在V4继续保留，但要等正常全局重新解锁；VIEW不能绕过故障、坐标准备或全局许可。普通安全OFF、冷启动时VIEW不会发出FC或力矩。

`GIMBAL VIEW LOCK`新增`PITCH_BLOCK`直接解释当前阻碍：

| 值 | 含义 |
| --- | --- |
| APPLIED | 已完成保持 |
| WAIT_CONTROL | 新视图请求尚未被控制任务处理 |
| WAIT_VIEW_PREPARE | 等待恢复pitch保持会话 |
| WAIT_ENABLE_FEEDBACK | 已提交使能，等待新的状态1反馈 |
| WAIT_CAPTURE | 等待捕获固定锁点 |
| REMOTE_NOT_ARMED / GLOBAL_OUTPUT_BLOCKED | 尚未解锁或全局动力许可被撤销 |
| REFERENCE_NOT_READY / GRAVITY_NOT_CALIBRATED | 坐标、零点或重力标定尚未准备 |
| FEEDBACK_NOT_FRESH | 没有新鲜反馈，正在保持只读查询 |
| WAIT_DISABLED_FEEDBACK | 等待旧失能动作及状态0确认 |
| PITCH_FAULT / DIAGNOSTIC_MODE | pitch故障或诊断模式正在阻止动力 |
| SESSION_NOT_REQUESTED | 会话未请求，等待控制任务核对当前视图需求 |
| HOLD_SUSPENDED | 单独停止pitch后仍处于全局许可会话，等待新的VIEW或全局重新解锁 |
| PITCH_NOT_ACTIVE | 后端/电机未进入活动状态 |
| NOT_SELECTED | 当前不是锁pitch的视图请求 |

`PITCH_HOLD_VALID|0`时，旧的`PITCH_HOLD_DEG`和`PITCH_ERROR_DEG`不表示已经锁定。回执同时包含requested、active、enabling、online、status、fault、stop reason以及全局armed/output allowed，避免仅凭PENDING猜原因。

## 锁定规则

- 锁定轴捕获一次当时实测角度作为固定目标，继续运行原位置/速度闭环。重复发送同一VIEW或之后发起另一轴STEP时，不会跟随扰动重新取锁点。
- 锁定期间，该轴的摇杆速度、遥控STEP/归位及串口STEP/HOME/TARGET不会改变目标。误选锁定轴也不会打断正在显示和测试的另一轴；拒绝的动作不排队。
- 切换到另一VIEW后，释放新选中的轴、捕获并锁定另一轴。旧轴尚未完成的动作会取消，相应五秒测量如实报告中止。
- pitch保持期间保存独立的固定锁点，受外力偏移后仍纠偏回该锁点，重复VIEW和另一轴阶跃不重新取点。
- V5已撤销V2引入的临时P/D放大。YAW VIEW下pitch固定保持与普通控制使用相同PID计算，原12个yaw/pitch增益数值均保留，没有隐藏倍率，也没有新增输出滤波或延迟来改变闭环。
- 仅在旧目标误差不超过0.2°、原始/滤波速度均不超过原6°/s静止门限且满足原20ms静止观察时，将位置积分对应的支撑力矩转移到速度积分。若还在运动或追赶旧目标，只清旧位置积分，不把运动积分转成长期力矩偏置；既有速度积分和重力补偿保留。切入/解除VIEW保持均执行这一判断。原力矩限幅、限位、撤权和故障保护继续生效。
- 电机闭环保持不是机械刹车；足够大的外力仍可能推开，目标是松手后恢复固定姿态，不能承诺物理上完全推不动。保持能力和是否振荡仍需实机验证。
- CH5进入测试时两轴先保持的上一版行为保留。要测pitch先发送 `PITCH VIEW`，要测yaw先发送 `YAW VIEW`；CH7确认/归位规则和近零静止门禁仍沿用原设置。
- `TELEM OFF`只暂停发送，不解除锁轴。STOP、失联、故障和原模式退出停机规则仍优先；解除锁定或恢复通信不代替重新解锁。

重新烧录后即可使用。本轮完成原工程编译和离线回归，未操作VOFA界面、连接电机或执行烧录；实际保持误差需实机检查。
