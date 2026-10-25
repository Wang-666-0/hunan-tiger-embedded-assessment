# 当前功能离线回归

`runtime_groups.json` 列出当前功能与实际 C 源文件。`run_selftests.py` 用 ARM GCC 的 `-Wall -Wextra -Werror` 编译，在 Unicorn 中运行 HAL/RTOS 替身；不连接串口、CAN 或烧录器。

- 正式源文件直接编译，无分区抽取、源码生成或另一套实现。
- `--groups 名称...` 选择相关组；不传参数运行全部现用组。
- 报告记录源文件哈希、编译配置、返回状态、断言数和首次失败位置；编译失败明确记录。
- 旧遥控、初期点动/观察、Yaw 单环/实验规划和设备初期诊断用例已随获批功能归档，保留整理前完整基线。
- 一部分 test_*.c 是其他用例引用的 HAL/状态夹具；独立旧 test_run 已移除，未放宽编译警告。
- mocks/bus_contention.h 只给 CAN 夹具制造无关流量和邮箱压力，代替已移除的设备查询。它不进入固件。
- Yaw 旧实验列仍检查固定值和原通道位置。Pitch 的真实轨迹、重力、刹车、归位与锁轴回归保留。

2026-10-25 整理前完整基线为 87 组，已有六组失败：yaw_performance、yaw_console、yaw_console_backend、yaw_console_backend_guarded、pitch_shape、pitch_manual90。其中旧单环三组随功能归档；现用的三个失败组继续保留并与基线对照，不通过改 PID 或放宽断言处理。

源码完整备份、旧用例及本轮结果见工程根目录 `02_过程记录/工程精简审阅_20261025/`。离线通过不能代替硬件阶跃和时序实测。
