# VOFA+ 学习和使用笔记

姓名：王世伟　创建日期：2026-10-04

本笔记根据本人考核操作和截图整理，由 Codex 辅助编写.

## 1. 考核目标

- 学习 VOFA+ 的基本使用方法和常用数据协议.
- 使用 VOFA+ 显示 STM32 通过串口发送给电脑的数据.

## 2. 实践环境与工程位置

| 项目 | 本次使用 |
| --- | --- |
| 单片机 | STM32F103C8T6 |
| 开发工具 | Keil µVision、STM32CubeMX |
| 上位机 | VOFA+ 1.3.10，界面经过汉化 |
| 串口模块 | CH340，USB 转 TTL |
| 串口 | USART1，115200，8 数据位、无校验、1 停止位、无流控 |
| 电脑端口 | 本次 COM10，换插口或电脑后重新确认 |
| 本次工程 | `07_VOFA学习和使用/Output/VOFA_Demo` |

从任务 05 的 UART_DMA 文件夹复制工程到本任务 Output 目录，并将文件夹改名为 VOFA_Demo. 工程文件名仍为 UART_DMA，不影响使用.

打开 [Keil 工程](../Output/VOFA_Demo/MDK-ARM/UART_DMA.uvprojx)，在 [main.c](../Output/VOFA_Demo/Core/Src/main.c) 修改程序. [CubeMX 配置](../Output/VOFA_Demo/UART_DMA.ioc) 和 [串口初始化](../Output/VOFA_Demo/Core/Src/usart.c) 保留原设置.

接线：CH340 RXD 接 PA9（STM32 TX），需要向单片机发送时，CH340 TXD 接 PA10（STM32 RX），GND 接开发板 GND. 使用 3.3 V 逻辑电平，开发板单独供电时不用连接模块 VCC.

## 3. 先了解数据协议怎么选

协议引擎决定 VOFA+ 如何解释串口收到的字节，不会替单片机生成数据.

| 协议 | 发送形式与用途 | 本次结果 |
| --- | --- | --- |
| RawData | 原始字节显示，适合文字打印和串口调试，不解析采样通道 | 已验证 Hello VOFA+ |
| FireWater | 逗号分隔数值、换行结束的文本，容易编写和检查，适合少通道、较低发送频率 | 已验证双通道曲线 |
| JustFloat | 小端 float 数组加固定帧尾，节省传输带宽，适合更多通道或更高频率 | 只了解，本次未实测 |

FireWater 的格式为：

```text
数据说明:数值1,数值2,数值3\n
```

说明和冒号可以省略，例如本次的 `10,90\n`. `\n` 在 C 字符串中表示真正的换行字节，必须发送；手动输入时发送字面字符“反斜杠和 n”不等于换行. 一帧两个数值对应 I0、I1 两个通道，不是两条串口.

JustFloat 的 float 数据后使用 `00 00 80 7F` 作为帧尾，不能把同一份文本发送代码直接切换成 JustFloat 来解析. 本次考核以已操作的 RawData 和 FireWater 为重点.

## 4. RawData：让单片机主动打印文字

原工程是电脑发送数据、STM32 DMA 接收并原样回传. 本次改为 STM32 主动发送，先停用 DMA 接收启动和主循环里的回传逻辑，保留 GPIO、DMA、USART 初始化. 程序中的 DMA 状态变量和回调暂时保留，但本次发送使用 HAL_UART_Transmit，不据此宣称采用 DMA 发送.

在 `USER CODE BEGIN PV` 区域定义：

```c
uint8_t vofa_text[] = "Hello VOFA+\r\n";
```

主循环演示代码：

```c
while (1)
{
    HAL_UART_Transmit(&huart1, vofa_text,
                      sizeof(vofa_text) - 1, 100);
    HAL_Delay(1000);
}
```

`huart1` 对应 USART1；`sizeof(...) - 1` 不发送字符串尾部的空字符；100 是发送等待超时，单位 ms；1000 是两次发送之间的延时，约每秒打印一行. `\r\n` 用于换行.

保存 → Keil 编译 → 下载到开发板. VOFA+ 选择 RawData、串口、CH340 对应端口、115200、8N1、无流控，再打开连接. 检查接收区持续出现 Hello VOFA+，不要只看软件发送区.

图 01 · RawData 文本显示与烧录结果：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/07_VOFA%E5%AD%A6%E4%B9%A0%E5%92%8C%E4%BD%BF%E7%94%A8/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/01_RawData%E6%96%87%E6%9C%AC%E6%98%BE%E7%A4%BA.png).


截图显示 Programming Done、Verify OK、Application running，以及持续接收 Hello VOFA+，证明程序下载和单片机到电脑的通信正常.

## 5. FireWater：发送并解析两个数值通道

把文字发送切换为下面两个数组：

```c
uint8_t vofa_data_1[] = "10,90\n";
uint8_t vofa_data_2[] = "90,10\n";
```

主循环：

```c
while (1)
{
    HAL_UART_Transmit(&huart1, vofa_data_1,
                      sizeof(vofa_data_1) - 1, 100);
    HAL_Delay(500);

    HAL_UART_Transmit(&huart1, vofa_data_2,
                      sizeof(vofa_data_2) - 1, 100);
    HAL_Delay(500);
}
```

保存、编译、烧录. VOFA+ 关闭连接，将数据引擎改为 FireWater，保持端口和串口参数不变，再打开连接.

接收区交替出现 10,90 和 90,10；右侧出现 I0、I1，数值交替变化，说明协议已经解析成功. 此时即使中间画布没有曲线，也不需要重新改串口代码.

图 02 · FireWater 双通道解析：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/07_VOFA%E5%AD%A6%E4%B9%A0%E5%92%8C%E4%BD%BF%E7%94%A8/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/02_FireWater%E6%95%B0%E6%8D%AE%E8%A7%A3%E6%9E%90.png).


## 6. 添加波形控件与绑定数据

1. 点击左侧蓝紫色重叠方框图标，打开控件栏.
2. 找到波形图，按住并拖到中间画布.
3. 在波形图内右键，X 轴选时间 T，Y 轴勾选 I0、I1，或选 All.
4. 使用波形右键菜单中的 Auto 调整轴范围.
5. 绿色位置滑块移到最右侧，显示最新数据.

页面顶部 new tab 旁的 `+` 只新建标签页. 添加波形控件要从控件栏拖入；拖入后也必须绑定数据通道，才会显示曲线.

## 7. 如何调整曲线比例

本次两帧之间使用 HAL_Delay(500)，实际间隔约 500 ms，包含少量发送和程序执行时间. VOFA+ 的 Δt 应填 500 ms，它是每帧间隔，不是完整交替周期. 一个交替周期约 1 秒.

推荐顺序：Δt 改为 500 ms → 双击控件边框放大 → 波形右键 Auto → 鼠标在横轴上滚动，将一屏调到约 5～10 秒. 纵轴覆盖 10～90 并留少量边距即可.

| 控件或操作 | 使用方法 |
| --- | --- |
| Δt | 数据帧间隔，用于换算时间和频率；填错不影响数值解析，但横轴时间会错 |
| 缓冲区上限 50000 /ch | 每通道保存的点数上限，初学保留默认，不是波特率 |
| Auto 点数对齐 100 | 自动调整可视点数的对齐设置，先保留默认，不用它修改时间单位 |
| 缓冲区旁 Auto | 自动调整可视数据范围，与波形右键的轴范围 Auto 区分 |
| 缓冲区长条和彩色滑块 | 选择查看哪段数据，看实时数据将绿色位置移到最右 |
| 垃圾桶 | 清空采样历史，重新观察 |
| 可视点数 | 当前画面显示多少采样点，点数越多能观察更长的变化过程 |
| X-div | 横轴每大格代表的时间，依赖 Δt 和显示点数 |
| 波形右键 Auto | 自动调整显示轴范围，避免 10 和 90 超出画面 |
| 鼠标在横轴上滚动 | 单独缩放时间范围 |
| 鼠标在纵轴上滚动 | 单独缩放数值范围 |
| 鼠标在绘图区滚动 | 同时缩放横轴与纵轴 |
| 双击控件边框 | 填满画布，再次操作可退出 |
| 双击横轴 | 添加测量游标 |
| 页面锁图标 | 锁定布局，避免误移动控件 |

本次两通道仅交替发送 10 和 90，波形图用直线连接相邻点，因此看起来像三角形. 这是离散数值的连线显示，不代表单片机发送了连续三角波. 想显示平顶方波，可使用支持的阶梯显示方式，或在高、低数值保持期间发送更多重复点.

图 03 · FireWater 双通道曲线：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/07_VOFA%E5%AD%A6%E4%B9%A0%E5%92%8C%E4%BD%BF%E7%94%A8/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/03_FireWater%E5%8F%8C%E9%80%9A%E9%81%93%E6%9B%B2%E7%BA%BF.png).


**截图核对：图 03 中 Δt 仍为 1 ms，曲线已显示，但横轴尚未按约 500 ms 的发送间隔换算. 验收前改成 500 ms，截图中的毫秒刻度不能用来证明真实周期或频率.**

## 8. 本次问题与处理

| 现象 | 处理 |
| --- | --- |
| 不知道从哪里开始建工程 | 复制任务 05 的 UART_DMA 到本任务 Output/VOFA_Demo，再打开 MDK-ARM/UART_DMA.uvprojx |
| 没有曲线但右侧有 I0、I1 | 协议已解析，添加波形控件并绑定通道 |
| 点击顶部 + 没有控件 | 它新建标签页，应从左侧控件栏拖入波形图 |
| 波形显示很小 | 双击控件边框，让波形填满画布 |
| 数值峰值被截断 | 波形右键 Auto，或在纵轴滚动调整 |
| 横轴是毫秒但程序约半秒一帧 | Δt 改成 500 ms，它不会改变程序发送速度 |
| 两点连成斜线 | 正常的线性连点显示，不是串口数据出错 |
| FireWater 没有解析出通道 | 检查数据引擎、数值分隔与帧尾换行 |
| 串口不能打开 | 检查正确 COM 号，关闭占用同一端口的其他串口软件 |

## 9. 验收演示与完成情况

1. 说明接线和串口参数，指出数据由 STM32 主动发送.
2. 如需演示文字，启用 RawData 文字代码并重新烧录，选择 RawData，观察 Hello VOFA+.
3. 启用 FireWater 数值代码并重新烧录，切换 FireWater，观察 I0、I1 数值和曲线.
4. 说明逗号分隔通道、换行结束一帧，以及 Δt 应与约 500 ms 的发送间隔一致.

两个模式切换涉及 MCU 发送内容和 VOFA+ 数据引擎，不能只切换上位机引擎就认为完成程序切换. 当前保存的工程为 FireWater 双通道版，RawData 发送代码保留为注释.

现有截图验证了文本接收、数值解析、双通道绘图及烧录运行. 本任务不需要进一步展开 FFT、直方图或控件开发. 时间轴配置还需按第 7 节修正.

## 10. 官方操作参考

- [RawData 协议](https://www.vofa.plus/docs/learning/dataengines/rawdata/)
- [FireWater 协议](https://www.vofa.plus/docs/learning/dataengines/firewater/)
- [JustFloat 协议](https://www.vofa.plus/docs/learning/dataengines/justfloat/)
- [绘图快速开始](https://www.vofa.plus/docs/learning/start/quick_start/)
- [波形图与缩放操作](https://www.vofa.plus/docs/learning/widgets/wave/)
