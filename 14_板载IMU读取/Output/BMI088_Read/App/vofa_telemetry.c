#include "vofa_telemetry.h"
#include "usart.h"
#include <math.h>
#include <string.h>

#define VOFA_UART_TIMEOUT_MS 3U /* 正常一帧约 1.39 ms；HAL Tick 量化会使故障等待略超过参数。 */
typedef char vofa_requires_32_bit_float[(sizeof(float) == 4U) ? 1 : -1];
static bool needs_resync; /* 串口失败后，接收端可能还留有未遇到帧尾的残缺字节。 */

bool VOFA_BuildEulerFrame(const float euler_deg[3], uint8_t frame[VOFA_EULER_FRAME_BYTES])
{
    uint32_t axis;
    if ((euler_deg == NULL) || (frame == NULL))
    {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        if (!isfinite(euler_deg[axis]))
        {
            return false; /* 不向上位机发布 NaN/无穷角度，防止模型失效。 */
        }
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        uint32_t bits;
        uint32_t offset = axis * 4U; /* I0/I1/I2 分别对应 Roll/Pitch/Yaw。 */
        memcpy(&bits, &euler_deg[axis], sizeof(bits)); /* 安全取得浮点位模式，不违反别名规则。 */
        frame[offset] = (uint8_t)(bits & 0xFFU); /* 明确按小端序编码，低字节先发送。 */
        frame[offset + 1U] = (uint8_t)((bits >> 8U) & 0xFFU);
        frame[offset + 2U] = (uint8_t)((bits >> 16U) & 0xFFU);
        frame[offset + 3U] = (uint8_t)((bits >> 24U) & 0xFFU);
    }
    frame[12] = 0x00U; /* JustFloat 固定帧尾：00 00 80 7F。 */
    frame[13] = 0x00U;
    frame[14] = 0x80U;
    frame[15] = 0x7FU;
    return true;
}

bool VOFA_SendEuler(const float euler_deg[3])
{
    uint8_t frame[VOFA_EULER_FRAME_BYTES];
    if (!VOFA_BuildEulerFrame(euler_deg, frame))
    {
        return false;
    }
    if (needs_resync)
    {
        uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
        if (HAL_UART_Transmit(&huart1, tail, sizeof(tail), VOFA_UART_TIMEOUT_MS) != HAL_OK)
        {
            (void)HAL_UART_AbortTransmit(&huart1); /* 帧尾也未成功时，仍不开始新的有效帧。 */
            return false;
        }
        needs_resync = false; /* 先结束接收缓冲中的残缺包，再发送三个角度的完整新包。 */
    }
    if (HAL_UART_Transmit(&huart1, frame, sizeof(frame), VOFA_UART_TIMEOUT_MS) != HAL_OK)
    {
        needs_resync = true; /* Abort 只清 MCU 状态，不能假定 VOFA 的半帧也被清掉。 */
        (void)HAL_UART_AbortTransmit(&huart1); /* 下一次成功发送前先补帧边界，避免旧字节拼入新包。 */
        return false;
    }
    return true; /* 阻塞发送已结束，局部 frame 此时才可以离开作用域。 */
}
