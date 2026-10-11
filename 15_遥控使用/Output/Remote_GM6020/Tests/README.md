# 软件自测

本目录把真实应用 C 文件编译为 Cortex-M4 ELF，再使用 Unicorn ARM 模拟器执行。编译启用 `-Wall -Wextra -Werror`。测试不连接串口、ST-Link、CAN 或电机；不替代接线、接收器 failsafe 和实机停止验证。

| 组 | 检查范围 |
| --- | --- |
| `ibus` | 已知固定帧、14 个小端通道、校验失败不覆盖旧帧、全部 32 个位置丢字节、31 个帧内位置插噪声、错位进入流、所有分块边界、粘包、超时、毫秒回卷、800/2200/FFFF 边界 |
| `remote_control` | 上电 ARM 高保持停止、OFF→ON 中位许可、非中位许可拒绝、旧帧不能解锁、CH6 guard、100ms 遥控超时、实际 GM6020 19/20ms 在线边界、79/80°C 温度、UART/CAN_TX/控制迟到、STOP 后重新解锁、坏帧不能续期、时间和有效帧序号回卷 |
| `remote_integration` | 实际 `motor_console.c` 的 UART 回调路由、`remote_input.c` 和 `remote_app.c`；UART4 正常接收、错误统计与立即撤销、RxCplt先于ErrorCallback时的错误码保留和坏字节丢弃、最近原始字节记录、Abort/Rearm、旧故障代次字节丢弃、续接收失败、队列丢字节、USART1 正常字节不计入遥控/错误单独统计、STOP 前及同毫秒排队 OFF 不得解锁、过期队列数据、电机已报告的迟到/CAN_TX 故障 |

集成组中的 HAL、FreeRTOS 队列、CAN 反馈快照和电机提交接口使用 mock；HAL mock模拟重新接收清除ErrorCode的行为，FE/NE顺序测试还模拟HAL在错误回调返回后清码。遥控解析、UART 回调路由、故障代次、许可状态机和应用连接代码均为正式源文件。它没有验证真实中断优先级、RTOS 调度时延、物理信号电平、CAN 邮箱或电机运动。

遥控接收路径已迁移为 UART4：P1 上层 pin4 / PA1 / AF8，115200 8N1，RX-only、NOPULL。集成测试以 `huart4` 模拟正式接收句柄，并检查 USART1 的字节不会计入遥控统计；引脚、电平和物理接线仍需实机验证。

## 运行

本机已有 ARM GCC 位于 `C:/Users/Administrator/.eide/tools/gcc_arm/bin`。测试依赖 Unicorn 2.1.4 安装在本目录 `.deps/`，该目录与 `.pip-cache/` 已由 `.gitignore` 排除，不修改永久 Python 环境。

在考核仓库根目录使用 PowerShell：

```powershell
& 'D:/Codex/Cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' '15_遥控使用/Output/Remote_GM6020/Tests/run_selftests.py'
```

可以加 `--group ibus`、`--group remote_control` 或 `--group remote_integration` 单独运行。换机器时使用 `--gcc-dir`、`--unicorn-dir` 指定已有工具路径；也可以把测试依赖单独安装到 `.deps/`：

```powershell
python -m pip install --target '15_遥控使用/Output/Remote_GM6020/Tests/.deps' 'unicorn==2.1.4'
```

完整运行输出 `selftest_results.json`；单组输出 `selftest_<组名>_results.json`。报告记录完成标志、模拟器返回、检查数、第一处失败行与实际被测源码 SHA256。脚本在编译前后核对源码哈希，若测试过程中有文件变化则拒绝生成成功报告。测试 ELF 保存在系统临时目录。
