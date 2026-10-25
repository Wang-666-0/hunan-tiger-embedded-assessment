# VIEW窗口锁轴操作

本轮直接修改原 VS Code/EIDE 工程。功能标识为 `VIEW_AXIS_HOLD_V1`；拨盘仍保留 `C610_SINGLE_BRAKE_V1J` 的单发优化。烧录文件仍为当前 MDK-ARM 下的 `build/Gimbal_Control/Gimbal_Control.hex`。

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
G|VIEW_LOCK|VIEW_AXIS_HOLD_V1|YAW|1|PITCH|0|MODE|HOLD_CURRENT|APPLY_MS|2
```

这里 `YAW|1` 表示请求锁yaw；`PITCH|1` 表示请求锁pitch。该行不是新的通道数据，原角度曲线绑定保持不变。未使能或已停止时只保存请求，不会因为发送VIEW自行使能电机。

## 锁定规则

- 锁定轴捕获一次当时实测角度作为固定目标，继续运行原位置/速度闭环。重复发送同一VIEW或之后发起另一轴STEP时，不会跟随扰动重新取锁点。
- 锁定期间，该轴的摇杆速度、遥控STEP/归位及串口STEP/HOME/TARGET不会改变目标。误选锁定轴也不会打断正在显示和测试的另一轴；拒绝的动作不排队。
- 切换到另一VIEW后，释放新选中的轴、捕获并锁定另一轴。旧轴尚未完成的动作会取消，相应五秒测量如实报告中止。
- pitch保持期间继续保留原速度积分和重力前馈；软件零点、两轴PID参数与拨盘单发/50ms连发逻辑不变。
- CH5进入测试时两轴先保持的上一版行为保留。要测pitch先发送 `PITCH VIEW`，要测yaw先发送 `YAW VIEW`；CH7确认/归位规则和近零静止门禁仍沿用原设置。
- `TELEM OFF`只暂停发送，不解除锁轴。STOP、失联、故障和原模式退出停机规则仍优先；解除锁定或恢复通信不代替重新解锁。

重新烧录后即可使用。本轮完成原工程编译和离线回归，未操作VOFA界面、连接电机或执行烧录；实际保持误差需实机检查。
