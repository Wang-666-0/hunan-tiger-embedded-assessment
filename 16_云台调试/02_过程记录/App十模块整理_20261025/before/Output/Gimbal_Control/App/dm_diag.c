#include "dm_diag.h"
#include "app_config.h"
#include <string.h>

/* 依据 dmBots/motor-sdk 官方 Python 示例的 __read_RID_param、
 * __process_set_param_packet 与 DM_variable：
 * https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py
 * 固定发送 0x33 读取操作；本模块没有写参数、保存、使能或运动操作。
 * 7..10、13..16、35..36 为 uint32 小端；20..23 为 float32 小端。
 */
const uint8_t DmDiag_ReadableRids[] = {
    DM_DIAG_RID_MASTER_ID, DM_DIAG_RID_SLAVE_ID, DM_DIAG_RID_TIMEOUT,
    DM_DIAG_RID_CONTROL_MODE, DM_DIAG_RID_HW_VERSION, DM_DIAG_RID_SW_VERSION,
    DM_DIAG_RID_SERIAL_NUMBER, DM_DIAG_RID_NPP, DM_DIAG_RID_GEAR_RATIO,
    DM_DIAG_RID_PMAX, DM_DIAG_RID_VMAX, DM_DIAG_RID_TMAX,
    DM_DIAG_RID_CAN_BAUD, DM_DIAG_RID_SUB_VERSION
};
const uint32_t DmDiag_ReadableRidCount = sizeof(DmDiag_ReadableRids) / sizeof(DmDiag_ReadableRids[0]);

uint8_t DmDiag_IsReadableRid(uint8_t rid)
{
    uint32_t i;
    for (i = 0U; i < DmDiag_ReadableRidCount; ++i) {
        if (DmDiag_ReadableRids[i] == rid) { return 1U; }
    }
    return 0U;
}

uint8_t DmDiag_BuildRead(uint8_t rid, CanFrame *out)
{
    if (out == 0 || !DmDiag_IsReadableRid(rid)) { return 0U; }
    memset(out, 0, sizeof(*out));
    out->bus = APP_PITCH_CAN_BUS;
    out->id = 0x7FFU;
    out->dlc = 8U;
    out->data[0] = (uint8_t)(APP_PITCH_DEVICE_ID & 0xFFU);
    out->data[1] = (uint8_t)((APP_PITCH_DEVICE_ID >> 8) & 0xFFU);
    out->data[2] = 0x33U;
    out->data[3] = rid;
    return 1U;
}

uint8_t DmDiag_MatchesReply(uint8_t rid, const CanFrame *frame)
{
    if (frame == 0 || !DmDiag_IsReadableRid(rid) || !CanFrame_IsValid(frame)) { return 0U; }
    if (frame->bus != APP_PITCH_CAN_BUS || frame->extended || frame->dlc != 8U) { return 0U; }
    /* 参数回复的 CAN ID 是电机配置的 MasterID，可以为 0，不能猜为从机 ID。
     * 0x7FF 是读取请求地址，保守拒绝，防止把请求回显当作零值回复。
     * 如果实机 MasterID 恰为 0x7FF，应使用官方工具核实，不能放宽此检查。
     */
    if (frame->id == 0x7FFU) { return 0U; }
    return (uint8_t)(frame->data[0] == (uint8_t)(APP_PITCH_DEVICE_ID & 0xFFU)
        && frame->data[1] == (uint8_t)((APP_PITCH_DEVICE_ID >> 8) & 0xFFU)
        && frame->data[2] == 0x33U && frame->data[3] == rid);
}
