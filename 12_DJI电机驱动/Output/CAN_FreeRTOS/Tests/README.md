# 软件模拟自测

selftest.c 与 mocks/ 替代 HAL、FreeRTOS 队列和串口，编译实际 App 的 C 实现，然后在 ARM 指令模拟器中执行。
它不加入正式固件，不连接 ST-Link/串口，不会驱动真实电机。

运行工具为已有 EIDE arm-none-eabi-gcc 与 Python Unicorn。
run_selftests.py 支持 --gcc-dir 和 --unicorn-dir。此次 Unicorn 仅装在系统临时目录 gm6020-unicorn-tests，不修改永久 Python 环境。

例如：
```powershell
python run_selftests.py --unicorn-dir "$env:TEMP/gm6020-unicorn-tests"
```

默认测试 ID1/电压模式、ID5/电流模式，涉及全部七个 ID 的槽位。输出软件自测结果.json 到任务过程记录目录。
测试 ELF 放入新建系统临时目录，不替换工程 HEX/AXF，也不会执行烧录。

模拟只验证逻辑和协议；板上中断、真实 RTOS 栈、收发器和机械行为仍需实物验收。
