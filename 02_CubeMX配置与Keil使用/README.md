# 02 · CubeMX 配置与 Keil 使用

本任务使用 STM32F103C8T6 核心板，通过 CubeMX 配置时钟、GPIO 和 SWD，生成 Keil MDK-ARM 工程，完成编译、下载和板载 LED 点亮验证。

## 文件入口

- [学习笔记：CubeMX 配置、Keil 编译下载与 LED 点亮](01_学习笔记/CubeMX与Keil点灯学习笔记.md)
- [CubeMX/Keil 工程：LED_Blink](Output/LED_Blink)
- [CubeMX 配置截图](02_过程记录/01_CubeMX_RCC与SWD配置.png)
- [Keil 编译与下载器配置截图](02_过程记录/02_Keil编译与下载器配置.png)
- [LED 点亮实物记录](02_过程记录/03_LED点亮实物.jpg)

## 当前状态

- CubeMX 工程型号、时钟、PC13 和 SWD 配置已核对。
- Keil 构建日志为 `0 Error(s), 0 Warning(s)`。
- 用户已完成程序下载并确认 LED 点亮。
- 现有 Keil 截图同时保留了早期误选 ST-LINK 造成下载失败的记录；学习笔记已说明处理过程。
- 为完整覆盖考核中的 Debug 要求，建议再补一张断点和单步调试截图。

本目录目前仅在本地整理，确认内容后再提交到 GitHub。
