#ifndef CAN_BUS_H
#define CAN_BUS_H
#include "can_frame.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
typedef struct {
    uint32_t rx_frames[2], tx_frames[2], errors[2], last_error[2];
    uint32_t dropped, fault_epoch;
    uint8_t started;
} CanBusState;
void CanBus_CreateQueue(void);
void CanBus_Start(void);
BaseType_t CanBus_Receive(CanFrame *frame, TickType_t wait_ticks);
void CanBus_GetState(CanBusState *state);
void CanBus_SetOutputPermission(uint8_t allowed);
uint8_t CanBus_OutputAllowed(void);
/* Yaw许可独立于CH6；总线故障仍同时撤销两路执行器许可。 */
void CanBus_SetYawPermission(uint8_t allowed);
uint8_t CanBus_YawAllowed(void);
HAL_StatusTypeDef CanBus_YawEnable(void);
HAL_StatusTypeDef CanBus_YawTorque(float torque_nm);
/* 固定FD可在撤权后发送；HAL_OK仅代表邮箱提交，须反馈status0确认失能。 */
HAL_StatusTypeDef CanBus_YawDisable(uint8_t cancel_old);
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
void CanBus_CancelReads(void);
#endif
