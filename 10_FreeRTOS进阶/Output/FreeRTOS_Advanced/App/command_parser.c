#include "command_parser.h"
#include "pwm_control.h"
#include "uart_service.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

void CommandParser_Process(const char *line)
{
    PwmCommand command;
    const char *reply;

    if (line[0] != 'B' && line[0] != 'T')
    {
        reply = "ERR command\r\n";
    }
    else
    {
        const char *number = &line[1];
        size_t digits = strlen(number);

        /* 先限定 1..5 位十进制数字，再转换，拒绝空值、负号、尾随字符。 */
        uint8_t valid = (digits >= 1 && digits <= 5);

        for (size_t i = 0; i < digits && valid; i++)
        {
            if (number[i] < '0' || number[i] > '9')
            {
                valid = 0;
            }
        }

        if (!valid)
        {
            reply = "ERR number\r\n";
        }
        else
        {
            uint32_t value = (uint32_t)strtoul(number, NULL, 10);

            if ((line[0] == 'B' && value > 1000) ||
                (line[0] == 'T' && (value < 200 || value > 10000)))
            {
                reply = "ERR range\r\n";
            }
            else
            {
                command.type = (line[0] == 'B') ? PWM_SET_MAX_BRIGHTNESS : PWM_SET_BREATH_PERIOD;

                command.value = value;

                if (PwmControl_Submit(&command) == pdPASS)
                {
                    /* 这只表示命令进入队列，PWM 任务将在下一轮应用它。 */
                    reply = "OK queued\r\n";
                }
                else
                {
                    reply = "ERR queue full\r\n";
                }
            }
        }
    }

    if (UartSend(&huart1, (uint8_t *)reply, (uint16_t)strlen(reply), 100) != HAL_OK)
    {
        Error_Handler();
    }
}
