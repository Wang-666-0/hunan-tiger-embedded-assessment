#ifndef CAN_BUS_H
#define CAN_BUS_H
#include "can_frame.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
typedef struct {
    uint32_t rx_frames[2], tx_frames[2], errors[2], last_error[2];
    /* 首次故障在取消邮箱前保存，后续错误/停止/诊断读取均不覆盖。 */
    uint32_t first_error[2], first_error_ms[2];
    uint8_t first_error_source[2];
    uint32_t dropped, fault_epoch;
    uint8_t started;
} CanBusState;
typedef enum {
    CAN_BUS_SOURCE_NONE = 0U,
    CAN_BUS_SOURCE_TX = 1U,
    CAN_BUS_SOURCE_RX_READ = 2U,
    CAN_BUS_SOURCE_IRQ = 3U,
    CAN_BUS_SOURCE_RX_QUEUE = 4U
} CanBusErrorSource;
typedef struct {
    CanBusState state;
    uint32_t hal_error[2], pending[2];
} CanBusDiagnostics;
void CanBus_CreateQueue(void);
void CanBus_Start(void);
BaseType_t CanBus_Receive(CanFrame *frame, TickType_t wait_ticks);
void CanBus_GetState(CanBusState *state);
/* 纯读取；HAL错误为HAL累计位，不清标志、不取消邮箱、不提交CAN帧。 */
void CanBus_GetDiagnostics(CanBusDiagnostics *out);
void CanBus_SetOutputPermission(uint8_t allowed);
uint8_t CanBus_OutputAllowed(void);
/* Yaw许可独立于CH6；总线故障仍同时撤销两路执行器许可。 */
void CanBus_SetYawPermission(uint8_t allowed);
uint8_t CanBus_YawAllowed(void);
HAL_StatusTypeDef CanBus_YawEnable(void);
HAL_StatusTypeDef CanBus_YawTorque(float torque_nm);
/* 固定FD可在撤权后发送；HAL_OK仅代表邮箱提交，须反馈status0确认失能。 */
HAL_StatusTypeDef CanBus_YawDisable(uint8_t cancel_old);
/* 固定FB人工恢复接口：两路许可必须均关闭、CAN无故障、CAN1邮箱已空。
 * 返回BUSY时不发送；不取消旧FD、不使能、不开放其他任意命令。 */
HAL_StatusTypeDef CanBus_YawClearComms(void);
/* 只读CAN1待发状态；不取消、不提交任何报文。 */
uint32_t CanBus_YawTxPending(void);
/* DM3507专用接口：CAN1、标准ID2、8字节MIT；不开放泛用动力发送。
 * FC和有限数力矩请求须独立Pitch许可；力矩仅按本机±10Nm协议端点编码。
 * FD可在撤权后提交；HAL_OK仅表示提交，必须由新鲜反馈确认失能。
 * CC是固定0x7FF / 02 00 CC 00 00 00 00 00只读状态查询，不要求运动许可。 */
void CanBus_SetPitchPermission(uint8_t allowed);
uint8_t CanBus_PitchAllowed(void);
HAL_StatusTypeDef CanBus_PitchEnable(void);
HAL_StatusTypeDef CanBus_PitchTorque(float torque_nm);
HAL_StatusTypeDef CanBus_PitchDisable(uint8_t cancel_old);
HAL_StatusTypeDef CanBus_PitchReadFeedback(void);
/* Pitch专属待发邮箱掩码；取消只作用于Pitch控制/状态查询，不动Yaw或诊断。
 * 取消尚未完成返回HAL_BUSY，调用方保留停机待确认并在下周期重试。 */
uint32_t CanBus_PitchTxPending(void);
HAL_StatusTypeDef CanBus_CancelPitchTx(void);
/* 泛用发送仅对完整驱动开放；点动开关不能开放任意CAN报文。 */
HAL_StatusTypeDef CanBus_Send(const CanFrame *frame);
/* 摩擦轮写接口：CAN2、70/71、SET_CURRENT，上限按app_config.h逐节点校验。
 * 固定极性按app_config.h校验，不能运行中切换。
 * 包括0在内均须输出许可及无CAN故障；只构造指定节点单帧。
 * 撤权后的回零只走StopFlywheels，HAL_OK仅代表提交到发送邮箱。 */
HAL_StatusTypeDef CanBus_JogVescCurrent(uint8_t node, int32_t milliamps);
/* 原子撤销摩擦轮许可并取消CAN2旧请求，再给70/71提交固定0mA。
 * Yaw编译关闭时保持历史双CAN取消；统一STOP另走Yaw失能路径。
 * HAL_ERROR包括取消仍在进行、总线故障和回零提交失败，调用方须重试。 */
HAL_StatusTypeDef CanBus_StopFlywheels(void);
/* 已完成首次STOP后的固定双0刷新，不取消邮箱中的正常读查询。
 * 门禁须关闭；邮箱尚有请求时返回HAL_BUSY，由调用方稍后刷新。 */
HAL_StatusTypeDef CanBus_KeepFlywheelsStopped(void);
/* 独立读接口只构造固定设备/固定读命令，不接受任意CAN报文。 */
HAL_StatusTypeDef CanBus_ReadDm(uint8_t rid);
HAL_StatusTypeDef CanBus_ReadVesc(uint8_t node, uint8_t command);
/* Pitch接入后只取消参数诊断邮箱，不撤轴许可、不取消任一轴控制/CC查询。 */
void CanBus_CancelReads(void);
#endif
