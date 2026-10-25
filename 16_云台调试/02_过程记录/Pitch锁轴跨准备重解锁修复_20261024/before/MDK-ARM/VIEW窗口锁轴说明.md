# VIEW窗口锁轴操作

本轮直接修改原 VS Code/EIDE 工程。功能标识为 `VIEW_AXIS_HOLD_V3`；拨盘仍保留 `C610_SINGLE_BRAKE_V1J` 的单发优化。烧录文件仍为当前 MDK-ARM 下的 `build/Gimbal_Control/Gimbal_Control.hex`。

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
G|VIEW_LOCK|VIEW_AXIS_HOLD_V3|YAW|1|PITCH|0|MODE|HOLD_CURRENT|APPLY_MS|2
```

这里 `YAW|1` 表示请求锁yaw；`PITCH|1` 表示请求锁pitch。V2还附带pitch实际状态，发送 `GIMBAL VIEW LOCK` 可以检查：

- `PITCH_APPLIED|1`：pitch在线、活动且固定锁点已经捕获。
- `PITCH_PENDING|1`：等待满足原准备/解锁条件后捕获锁点。
- `PITCH_HOLD_DEG`：独立保存的锁点；`PITCH_ACTUAL_DEG`：真实反馈角度；`PITCH_TARGET_DEG`：控制器目标；`PITCH_ERROR_DEG`：锁点减真实角度。
- `PITCH_OUT_NM`：实际计算的控制输出；`PITCH_POSITION_GAIN|2` / `PITCH_SPEED_GAIN|2`：保持专用的P/D倍率。

VIEW命令首次回执时控制任务可能还未处理新请求，过一会查询才能区分“请求锁定”和“已应用”。该行不是新的通道数据，原角度曲线绑定保持不变。未解锁、故障或未准备时保持停止。V3允许新发出的YAW VIEW在已经解锁且全局动力许可有效的会话中，为健康且已准备的pitch恢复保持会话；走原Request/FC/状态1反馈握手，之后捕获当前实测姿态，使用中不会直接追旧目标。

## V3：处理PITCH_PENDING一直为1

V2只设置VIEW锁定标志。如果此前用单轴READY/ON流程或单轴OFF使pitch.requested变成0，即使yaw还在正常工作，pitch也可能长期停在PENDING、OUT=0。

V3在原控制任务中消费新的VIEW准备请求：先通过原CC只读查询保持反馈新鲜；已有全局解锁/动力许可，且坐标、软件零点、重力标定、反馈、状态和故障门禁均满足时，才通过原使能握手恢复pitch。成功后捕获当时的真实角度，旧的−18°目标不沿用。重复点击YAW VIEW也可以发起这次准备；已经保持时重复点击不重新取锁点。

STOP会取消本次准备，不会因为VIEW一直选中就重新启动。再次发送VIEW也不能绕过全局重新解锁、故障和坐标准备要求。普通安全OFF、冷启动时VIEW不会发出FC或力矩。

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
| SESSION_NOT_REQUESTED | 会话未请求，需新的VIEW动作或原准备流程 |
| PITCH_NOT_ACTIVE | 后端/电机未进入活动状态 |
| NOT_SELECTED | 当前不是锁pitch的视图请求 |

`PITCH_HOLD_VALID|0`时，旧的`PITCH_HOLD_DEG`和`PITCH_ERROR_DEG`不表示已经锁定。回执同时包含requested、active、enabling、online、status、fault、stop reason以及全局armed/output allowed，避免仅凭PENDING猜原因。

## 锁定规则

- 锁定轴捕获一次当时实测角度作为固定目标，继续运行原位置/速度闭环。重复发送同一VIEW或之后发起另一轴STEP时，不会跟随扰动重新取锁点。
- 锁定期间，该轴的摇杆速度、遥控STEP/归位及串口STEP/HOME/TARGET不会改变目标。误选锁定轴也不会打断正在显示和测试的另一轴；拒绝的动作不排队。
- 切换到另一VIEW后，释放新选中的轴、捕获并锁定另一轴。旧轴尚未完成的动作会取消，相应五秒测量如实报告中止。
- pitch保持期间保存独立的固定锁点，受外力偏移后仍纠偏回该锁点，重复VIEW和另一轴阶跃不重新取点。
- 仅在YAW VIEW锁pitch时，临时将pitch位置P/D和速度P/D分别按2倍参与计算；I增益保持原值。小误差静态纠偏斜率约为原来的4倍，速度比例阻尼约2倍。存储的PID参数不变，切回PITCH VIEW下一次控制计算恢复原增益。
- 切入锁定时，将原位置积分对应的支撑力矩转移到速度积分，再清位置积分；解除锁定时也保留这部分支撑，避免负载补偿突然消失。重力标定和系数保留，原力矩限幅、限位、撤权和故障保护继续生效。
- 电机闭环保持不是机械刹车；足够大的外力仍可能推开，目标是松手后恢复固定姿态，不能承诺物理上完全推不动。保持能力和是否振荡仍需实机验证。
- CH5进入测试时两轴先保持的上一版行为保留。要测pitch先发送 `PITCH VIEW`，要测yaw先发送 `YAW VIEW`；CH7确认/归位规则和近零静止门禁仍沿用原设置。
- `TELEM OFF`只暂停发送，不解除锁轴。STOP、失联、故障和原模式退出停机规则仍优先；解除锁定或恢复通信不代替重新解锁。

重新烧录后即可使用。本轮完成原工程编译和离线回归，未操作VOFA界面、连接电机或执行烧录；实际保持误差需实机检查。
