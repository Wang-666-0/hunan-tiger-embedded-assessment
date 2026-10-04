# 工程与验收文件

本次从任务 05 的 UART_DMA 复制出独立工程 VOFA_Demo，当前保存的是主动发送 FireWater 双通道数据的版本.

- [Keil 工程](VOFA_Demo/MDK-ARM/UART_DMA.uvprojx)
- [CubeMX 配置](VOFA_Demo/UART_DMA.ioc)
- [发送代码](VOFA_Demo/Core/Src/main.c)

RawData 的 Hello VOFA+ 代码保留为注释，演示该模式时启用文字代码、停用数值发送代码，重新编译烧录. 工程文件名仍为 UART_DMA，不影响文件夹名 VOFA_Demo 的使用.
