# 基于 PWM 实现呼吸灯

姓名：王世伟　创建日期：2026-10-01

## 1. 任务与原理

使用 STM32F103C8T6 开发板，通过 CubeMX 配置 TIM1，在 Keil 中编写代码，实现 PA8 外接 LED 周期性渐亮、渐暗。

PWM 通过快速切换高、低电平控制 LED 的平均亮度。频率决定每秒重复的次数，占空比决定一个周期中高电平所占的比例。本实验保持 PWM 频率不变，逐渐改变占空比形成呼吸效果。

LED 接在 PA8（TIM1_CH1），需串联限流电阻。若接法为 PA8 → 电阻 → LED 正极，LED 负极 → GND，占空比越大越亮；若 LED 从 3.3 V 接向 PA8，亮度变化方向相反。板载 PC13 LED 没有定时器 PWM 输出功能，因此本实验使用外接 LED。

## 2. CubeMX 配置

1. 新建 STM32F103C8Tx 工程，在 SYS 中选择 Serial Wire，保留 SWD 下载调试接口。
2. RCC 的 HSE 选择 Crystal/Ceramic Resonator。外部晶振 8 MHz，经 PLL ×9 得到系统时钟 72 MHz；AHB ÷1、APB1 ÷2、APB2 ÷1，因此 TIM1 时钟为 72 MHz。
3. TIM1 的 Clock Source 选择 Internal Clock，Channel1 选择 PWM Generation CH1；PA8 分配为 TIM1_CH1，GPIO 为复用推挽输出。

| 参数 | 设置 | 作用 |
| --- | --- | --- |
| Prescaler（PSC） | 71 | 时钟除以 PSC＋1，计数频率为 1 MHz |
| Counter Mode | Up | 从 0 向上计数 |
| Counter Period（ARR） | 999 | 每周期计数 1000 次 |
| Repetition Counter | 0 | 每个计数周期可产生更新事件 |
| PWM Mode | PWM mode 1 | CNT < CCR1 时输出有效电平 |
| Pulse（CCR1） | 500 | 初始占空比为 50% |
| CH Polarity | High | 有效电平为高电平 |
| Output compare preload | Enable | CCR 新值在更新事件时生效 |

Clock Division 保持 No Division，Fast Mode、Break 保持关闭，Dead Time 为 0。CKD 主要影响滤波与死区相关时钟，不作为本实验计数器分频。

```text
PWM频率 = 72 MHz / [(71 + 1) × (999 + 1)] = 1 kHz
高电平占空比 = CCR1 / (ARR + 1) = CCR1 / 1000
```

TIM1 计数参数图片链接：________（原临时截图已失效，待补图；参数已按工程核对）

PWM 通道配置图片链接：________（对应 02_PWM通道配置.png）

时钟树图片链接：________（对应 03_时钟树.png）

在 Project Manager 中将工程命名为 PWM_Breath，工具链选择 MDK-ARM，开启 Keep User Code when re-generating，再生成工程并用 Keil 打开。

## 3. Keil 编程与下载

在 main.c 中，MX_TIM1_Init() 之后的 USER CODE BEGIN 2 区域启动 PWM：

```c
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
```

先验证初始 50% 占空比能使 LED 持续发光，再在 while (1) 内加入呼吸循环。当前工程实际使用步长 5、每步延时 5 ms：

```c
for (uint32_t pulse = 0; pulse <= 1000; pulse += 5)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
    HAL_Delay(5);
}

for (uint32_t pulse = 1000; pulse > 0; pulse -= 5)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
    HAL_Delay(5);
}
```

该宏修改 CCR1，使高电平占空比在 0%～100% 间变化。每个方向约 1 秒，完整呼吸周期约 2 秒。PWM 波形由定时器持续输出，HAL_Delay 只控制亮度更新间隔，不改变 1 kHz 的 PWM 频率。该写法会阻塞主循环，适合当前单功能实验。

Keil 使用 CMSIS-DAP/SWD 下载。现有工程构建日志显示 Arm Compiler 5.06 update 7，0 Error(s)、0 Warning(s)；最终代码截图显示 Programming Done、Verify OK 和 Application running，确认下载、校验成功。本人已确认呼吸效果。

最终代码与烧录成功图片链接：________（对应 05_最终代码与烧录成功.png）

## 4. 验证与理解

最终效果视频链接：________（对应 04_PWM呼吸灯效果.mp4）

最终效果视频保存在本任务的 02_过程记录 文件夹。本笔记按本人反馈记录功能完成；PWM 频率与呼吸周期由配置和代码计算，未使用示波器实测。

- PSC、ARR 决定 PWM 频率，CCR 决定占空比。
- 增大每步延时，呼吸变慢；减小步长，变化更细且周期变长。
- 占空比与人的视觉亮度并非严格线性，因此线性改变 CCR 不等于视觉上完全匀速变亮。
- 自写代码应放在 USER CODE 标记之间，重新生成工程时保留。

---

本笔记由王世伟完成实验，Codex 辅助梳理原理、核对工程参数并整理文字。
