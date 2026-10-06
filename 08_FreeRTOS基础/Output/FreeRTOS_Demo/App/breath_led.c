#include "breath_led.h"
#include "tim.h"
#include "cmsis_os.h"

void BreathLed_Task(void *argument)
{
    (void)argument;
    /* brightness 是比较寄存器 CCR 的值；LED 亮灭方向仍取决于接线和 PWM 极性。 */
    uint16_t brightness = 0;
    int16_t step = 5;
    /* 启动 TIM1 通道1的 PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    for (;;)
    {
        /* 设置占空比，范围为 0～1000 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, brightness);

        /* 等待期间，让串口任务也能运行 */
        /* CMSIS_V2 的参数单位是 tick；当前 1 kHz tick 下对应 5 ms。
         * 保留基础考核原有的相对延时，绝对周期用法见第 10 项。
         */
        osDelay(5);

        /* 到达最亮或最暗时，改变方向 */
        if (brightness >= 1000)
        {
            step = -5;
        }
        else if (brightness == 0)
        {
            step = 5;
        }

        brightness = (uint16_t)((int16_t)brightness + step);
    }
}
