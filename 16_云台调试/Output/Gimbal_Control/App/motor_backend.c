#include "motor_backend.h"
#include <string.h>

void MotorBackend_GetHealth(MotorHealth *health)
{
    if (health != 0) { memset(health, 0, sizeof(*health)); }
    /* 待补：四类驱动的身份、单位、反馈时间戳和故障状态。禁止伪造在线反馈。 */
}

uint8_t MotorBackend_OnFrame(const CanFrame *frame)
{
    (void)frame;
    /* 待补：按bus、IDE、完整ID和DLC分发，不能把设备编号直接当报文ID。 */
    return 0U;
}

uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t output_allowed)
{
    (void)request;
    (void)output_allowed;
    /* 待补：角度/速度闭环、摩擦轮ERPM换算、拨盘位置步进及各驱动安全停机。
     * 当前无CanBus_Send调用，即使外部误传output_allowed=1也不会产生动力报文。 */
    return 0U;
}
