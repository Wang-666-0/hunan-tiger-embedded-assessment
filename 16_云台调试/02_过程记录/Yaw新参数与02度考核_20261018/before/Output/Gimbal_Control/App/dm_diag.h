#ifndef DM_DIAG_H
#define DM_DIAG_H

#include <stdint.h>
#include "can_frame.h"

/* 只读 RID。名称沿用达妙官方 SDK，避免依据型号默认值猜测实机配置。 */
#define DM_DIAG_RID_MASTER_ID 7U
#define DM_DIAG_RID_SLAVE_ID 8U
#define DM_DIAG_RID_TIMEOUT 9U
#define DM_DIAG_RID_CONTROL_MODE 10U
#define DM_DIAG_RID_HW_VERSION 13U
#define DM_DIAG_RID_SW_VERSION 14U
#define DM_DIAG_RID_SERIAL_NUMBER 15U
#define DM_DIAG_RID_NPP 16U
#define DM_DIAG_RID_GEAR_RATIO 20U
#define DM_DIAG_RID_PMAX 21U
#define DM_DIAG_RID_VMAX 22U
#define DM_DIAG_RID_TMAX 23U
#define DM_DIAG_RID_CAN_BAUD 35U
#define DM_DIAG_RID_SUB_VERSION 36U

extern const uint8_t DmDiag_ReadableRids[];
extern const uint32_t DmDiag_ReadableRidCount;

uint8_t DmDiag_IsReadableRid(uint8_t rid);
/* 成功返回 1；白名单外或空指针返回 0，不改写输出。 */
uint8_t DmDiag_BuildRead(uint8_t rid, CanFrame *out);
/* 仅匹配当前请求的参数回复，原始值由上位机按 RID 类型解析。 */
uint8_t DmDiag_MatchesReply(uint8_t rid, const CanFrame *frame);

#endif
