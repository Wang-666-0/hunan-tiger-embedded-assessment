#ifndef VOFA_TELEMETRY_H
#define VOFA_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define VOFA_EULER_FRAME_BYTES 16U /* 三个 float 共 12 字节，再加 4 字节帧尾。 */

/* 输入顺序固定为 Roll/Pitch/Yaw，单位度；输出为 JustFloat 小端二进制帧。 */
bool VOFA_BuildEulerFrame(const float euler_deg[3], uint8_t frame[VOFA_EULER_FRAME_BYTES]);
/* 仅由 imuTask 调用；USART1 不与 printf 或其他任务共享。 */
bool VOFA_SendEuler(const float euler_deg[3]);

#endif
