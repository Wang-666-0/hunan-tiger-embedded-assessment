/**
 * @file dm_diag.h
 * @brief 达妙只读 RID 查询
 * 职责：声明允许读取的 RID 白名单及请求/回复匹配接口。
 * 主要接口：DmDiag_IsReadableRid, DmDiag_BuildRead, DmDiag_MatchesReply。
 * 实现/调用：app_diagnostics.c；只读取参数，不写入设备或保存设置。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef DM_DIAG_H
#define DM_DIAG_H
#include "app_config.h"

#include <stdint.h>
#include "can_frame.h"

/* 只读 RID。名称沿用达妙官方 SDK，避免依据型号默认值猜测实机配置。 */

extern const uint8_t DmDiag_ReadableRids[];
extern const uint32_t DmDiag_ReadableRidCount;

uint8_t DmDiag_IsReadableRid(uint8_t rid);
/* 成功返回 1；白名单外或空指针返回 0，不改写输出。 */
uint8_t DmDiag_BuildRead(uint8_t rid, CanFrame *out);
/* 仅匹配当前请求的参数回复，原始值由上位机按 RID 类型解析。 */
uint8_t DmDiag_MatchesReply(uint8_t rid, const CanFrame *frame);

#endif
