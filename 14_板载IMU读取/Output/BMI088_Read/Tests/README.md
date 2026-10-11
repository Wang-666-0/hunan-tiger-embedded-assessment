# 软件模拟自测

测试真实的 App C 文件，不连接串口、ST-Link 或板卡。测试文件不加入正式 EIDE/Keil 源文件组。

使用已经安装的 EIDE ARM GCC，以及先前任务在系统临时目录 `gm6020-unicorn-tests` 安装的 Unicorn：

```powershell
python .\run_selftests.py
```

如工具位置不同，通过 `--gcc-dir` 和 `--unicorn-dir` 指定。脚本只在新建的系统临时目录生成 ELF，检查结果和源码 SHA-256 打印到终端；不替换固件。

| 分组 | 内容 |
| --- | --- |
| mahony | 初始倾斜、Yaw 积分、实际 dt、重力收敛、归一化、非有限数据与失败保护 |
| bmi088_driver | SPI dummy、软复位等待、配置读回与只读位、SI 单位、温度、ID/HAL 失败不覆盖样本 |
| calibration_telemetry | 500 次静止样本、运动与方差拒绝、三通道小端帧和固定帧尾、UART 半帧与重同步失败恢复 |

2026-10-07 实际运行：Mahony 2150 项、BMI088 驱动 104 项、校准/协议 2062 项，共 4316 项检查，0 失败。真实结果与构建记录见任务的 `02_过程记录/软件验证记录.md`。

SPI/GPIO/DWT 的实机时序、真实 RTOS 调度和栈、姿态坐标方向、200 Hz 持续串口发送仍需板上验收。
