# 参数采集版离线自检

`run_selftests.py` 使用 ARM GCC 编译正式C源码，再用 Unicorn 执行 Cortex-M4 指令。HAL、FreeRTOS接口使用测试替身；不连接串口、CAN或烧录器。它验证软件逻辑，不代表板上保护或电机性能已经验收。

| 分组 | 使用的正式模块 | 检查范围 |
| --- | --- | --- |
| ibus | ibus.c | 正常帧、校验错误、噪声、帧间超时、输入恢复和时间回卷 |
| logic | remote_mapper、safety_manager、target_manager、app_logic、motor_backend、can_frame | 通道范围、死区、解锁新帧、故障隔离、100 ms边界、单发/连发互锁、阶跃沿和发送后端拒绝 |
| legacy_remote | remote_mapper.c，实际默认配置 | 只依赖CH1/5/6，未配置辅助通道不影响有效性，发射与阶跃保持安全 |
| dm_diag | dm_diag.c、can_frame.c | 全RID白名单、固定只读帧、主从ID/操作码/RID匹配及查询回显拒绝 |
| vesc_diag | vesc_diag.c | 节点/命令白名单、短包/长包、连续偏移、CRC16、超限与错误响应 |
| can_bus | can_bus.c、can_frame.c、诊断构造器 | 双CAN过滤器、队列与错误代次，动力发送拒绝，读接口只生成白名单帧，取消挂起发送 |
| diagnostics | app_diagnostics.c与实际协议模块 | 单请求互斥、超时、tick回绕、旧帧/故障隔离、MasterID核对、分块结果与取消 |
| console | app_console.c与实际协议模块 | UART字节队列、全部命令白名单、输出格式、STOP、残行/故障恢复及禁止字节注入 |

`logic`分组以 `APP_REMOTE_PROFILE_LEGACY=0` 保留后续完整考核逻辑验证；`legacy_remote`验证当前固件实际默认映射。`test_motor_capture.py`另用Python unittest/fake serial检查PC流程、未知配置格式、超时、分块损坏与运动命令拒绝。

在仓库根目录执行：

```powershell
python '.\16_云台调试\Output\Gimbal_Control\Tests\run_selftests.py'
python '.\16_云台调试\Output\Gimbal_Control\Tests\test_motor_capture.py'
```

需 Python 3.8+、`arm-none-eabi-gcc` 和 `unicorn`。默认 GCC 位于用户目录 `.eide/tools/gcc_arm/bin`，Unicorn 优先使用本目录 `.deps`，否则复用第15项 Tests/.deps。可通过 `--gcc-dir` 和 `--unicorn-dir` 指定。每组以 `-Wall -Wextra -Werror` 编译；报告保留源文件SHA256、断言数、失败行和实际返回状态，输出到 `selftest_results.json`。

本轮用HAL/RTOS替身验证了USART1完整命令解析，但未模拟真实UART/CAN电气时序与FreeRTOS调度负载。最终固件另用ARM Compiler 5完整构建，最新证据见[参数采集版验证记录](../../../02_过程记录/参数采集版验证记录.md)，原基础版历史结果见[软件验证记录](../../../02_过程记录/软件验证记录.md)。
