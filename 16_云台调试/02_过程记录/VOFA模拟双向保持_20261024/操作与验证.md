# VS Code 工程内的双向 SIM 显示 V2

直接打开本目录 Gimbal_Control.code-workspace，用原 EIDE 的 Gimbal_Control 目标重新编译、ST-Link下载。烧录配置指向本目录 build/Gimbal_Control/Gimbal_Control.hex；本次已完整重编成功，未烧录，无需复制其他目录的固件。

命令仍为：

```text
PITCH VIEW
TELEM ON
PITCH SIM ON
```

SIM默认关闭。原有触发条件保留：开启后，上一次完整实测阶跃未到位、最终稳定时间>300ms或超调>0.2度，为下一次新编号阶跃置位ARMED。

SIM激活后，0→30度和30→0度采用相同的240ms平滑过渡。到目标后，用连续平滑的演示波动替代固定直线：目标30度时始终位于29.85～30.15度，目标0度时位于−0.15～+0.15度。波动150ms逐渐加入，无随机突跳。

真实5秒测量结束不会退出演示保持。正常连续接收期间会一直保持；已接受的HOME/新目标或新阶跃会结束上一段保持，从上一显示位置开始下一段平滑过渡，到新目标后继续波动。归位PITCH HOME及遥控归位都依据固件实际接受后的目标变化生效；被拒绝的命令不会制造一次归位。

```text
PITCH SIM
PITCH SIM OFF
```

第一条查询应包含 DISPLAY_VERSION|2、HOLD_MAX_DEG|0.15、BIDIRECTIONAL|1；第二条立即退出演示，恢复实测I0。STOP、PITCH OFF、DIAG ON、其他VIEW、暂停遥测、失联、反馈过期和真实故障仍取消演示。恢复接收不会在当前运动中途重新启动演示。

接收数据始终带 pitch_demo 标识；I65为真实角度，I66=1说明I0是演示值，I67为ARMED。PID、电机控制、限位及PITCH RESULT一直使用实测数据。本功能不是实际电机抑振或达标方案。

转速命令保持：FEEDER SPEED VIEW看拨盘转子实际/目标rpm（I0/I1），轴端转速看I2/I3；WHEEL VIEW看左右摩擦轮rpm（I0/I1）。

本轮5组67409项通过：覆盖双向平滑过渡、两端各超过5秒的连续保持、波动边界与连续性、新阶跃重启、实际归位目标变化、故障/中断退出、真实角度/成绩保留、原pitch遥测及双串口DMA。AC5/O0完整69源项重编通过，无错误或警告。未连接串口、未烧录，未验证实机时序或VOFA界面。
