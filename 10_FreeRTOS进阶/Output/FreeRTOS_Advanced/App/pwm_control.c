#include "pwm_control.h"
#include "main.h"
#include "tim.h"
#include "task.h"
#include "queue.h"

/* 队列和 PWM 状态只由本模块管理，解析模块不能直接写 CCR。 */
static QueueHandle_t pwm_command_queue;

void PwmControl_Init(void)
{
    pwm_command_queue = xQueueCreate(8, sizeof(PwmCommand));
    if (pwm_command_queue == NULL)
        Error_Handler();
}

BaseType_t PwmControl_Submit(const PwmCommand *command)
{
    /* 队列复制整个结构体，调用者的局部 command 离开作用域也不会失效。
     * 等待时间 0 表示队列满时立即返回，由命令模块回复 ERR queue full。
     */
    return xQueueSend(pwm_command_queue, command, 0);
}

void PwmControl_Task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t update_period = pdMS_TO_TICKS(5);

    uint32_t max_brightness = 1000;
    uint32_t breath_period_ms = 2000;
    uint32_t phase_ms = 0;

    PwmCommand command;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    for (;;)
    {

        /* 每轮最多处理 8 条命令，限制工作量，给 5 ms 更新留出时间。 */
        for (uint32_t i = 0; i < 8; i++)
        {
            if (xQueueReceive(pwm_command_queue, &command, 0) != pdPASS)
            {
                break;
            }

            switch (command.type)
            {
            case PWM_SET_MAX_BRIGHTNESS:
                if (command.value <= 1000)
                {
                    max_brightness = command.value;
                }
                break;

            case PWM_SET_BREATH_PERIOD:
                if (command.value >= 200 && command.value <= 10000)
                {
                    breath_period_ms = command.value;
                    phase_ms = 0;
                }
                break;

            default:
                break;
            }
        }

        /* 三角波：前半周期变亮，后半周期变暗；B 设置峰值，T 设置总周期。 */
        uint32_t half_period_ms = breath_period_ms / 2;
        uint32_t brightness;

        if (phase_ms < half_period_ms)
        {
            brightness = max_brightness * phase_ms / half_period_ms;
        }
        else
        {
            brightness = max_brightness * (breath_period_ms - phase_ms) /
                         (breath_period_ms - half_period_ms);
        }

        /* CCR 是占空比比较值。B0 写 0；LED 的实际亮灭还取决于接线极性。 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, brightness);

        phase_ms += 5;

        if (phase_ms >= breath_period_ms)
        {
            phase_ms = 0;
        }

        /* 以固定基准推进唤醒时间，避免把本轮运行时间累计进周期。
         * 周期精度仍受 tick 分辨率、抢占和任务执行时间影响。 */
        vTaskDelayUntil(&last_wake, update_period);
    }
}
