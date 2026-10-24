#ifndef YAW_BACKEND_H
#define YAW_BACKEND_H
#include "yaw_controller.h"
#include "yaw_step.h"
#include "target_manager.h"
#define YAW_CLEAR_OFF_MOTION_HOLD_MS 500U
#define YAW_STEP_RAW_SPEED_SANITY_DPS 30.0f
/* 正式STEP门禁与故障保护分开；位值可组合，CHECK/遥控回执解释拒绝原因。 */
typedef enum {
    YAW_STEP_BLOCK_REFERENCE = 1UL << 0,
    YAW_STEP_BLOCK_INACTIVE = 1UL << 1,
    YAW_STEP_BLOCK_TRANSITION = 1UL << 2,
    YAW_STEP_BLOCK_FEEDBACK = 1UL << 3,
    YAW_STEP_BLOCK_FAULT = 1UL << 4,
    YAW_STEP_BLOCK_PERMISSION = 1UL << 5,
    YAW_STEP_BLOCK_HOME = 1UL << 6,
    YAW_STEP_BLOCK_POSITION = 1UL << 7,
    YAW_STEP_BLOCK_MOVING = 1UL << 8,
    YAW_STEP_BLOCK_STILL = 1UL << 9,
    YAW_STEP_BLOCK_WINDOW = 1UL << 10,
    YAW_STEP_BLOCK_PITCH = 1UL << 11
} YawStepBlock;
typedef struct {
    YawController controller;
    Gl60Feedback feedback;
    uint32_t rx_count, feedback_age_ms;
    uint8_t online, requested, active, stop_pending, enabling;
    uint8_t zero_set, step_hold;
    uint32_t step_blocks, step_still_since_ms, step_still_ms, step_window_left_ms;
    uint8_t step_still_samples;
} YawBackendState;
/* 位值可同时出现；CHECK只读，当前阻碍与上一次清错拒绝分别记录。 */
typedef enum {
    YAW_CLEAR_NOT_MONITORING = 1UL << 0,
    YAW_CLEAR_REQUESTED = 1UL << 1,
    YAW_CLEAR_ACTIVE = 1UL << 2,
    YAW_CLEAR_WAITING_DISABLE = 1UL << 3,
    YAW_CLEAR_OUTPUT_ALLOWED = 1UL << 4,
    YAW_CLEAR_FEEDBACK_STALE = 1UL << 5,
    YAW_CLEAR_STATUS_NOT_13 = 1UL << 6,
    YAW_CLEAR_OTHER_FAULT = 1UL << 7,
    YAW_CLEAR_NOT_STATIONARY = 1UL << 8,
    YAW_CLEAR_DISABLE_UNCONFIRMED = 1UL << 9,
    YAW_CLEAR_CAN_FAULT = 1UL << 10,
    YAW_CLEAR_MAILBOX_BUSY = 1UL << 11,
    YAW_CLEAR_TX_FAILED = 1UL << 12
} YawClearBlock;
typedef struct {
    uint32_t blocks, last_blocks, hold_ms, fd_age_ms, mailbox_pending, can_epoch;
    uint32_t off_motion_faults, required_hold_ms;
    uint8_t samples, monitoring, fd_sent, fd_cancel, fd_result, clear_result;
    uint8_t actuation_started;
} YawClearCheck;
void YawBackend_Init(void);
uint8_t YawBackend_OnFrame(const CanFrame *frame);
void YawBackend_GetState(YawBackendState *out);
/* ON仅准备链路；必须新的CH5 OFF->ON且反馈健康才会使能。 */
uint8_t YawBackend_Request(uint8_t enabled);
/* 人工READY会话首步：只建立FD监测并撤销请求，不发送FC。 */
uint8_t YawBackend_PrepareStop(void);
/* READY全部条件已确认后释放准备门禁；仍需新的遥控人工解锁。 */
void YawBackend_ReleasePreparation(void);
void YawBackend_Stop(void);
void YawBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic);
uint8_t YawBackend_SetParameter(char parameter, float value);
uint8_t YawBackend_SetPid(float kp, float ki, float kd);
uint8_t YawBackend_SetLoopPid(char loop, float kp, float ki, float kd);
uint8_t YawBackend_SetLoopParameter(char loop, char parameter, float value);
uint8_t YawBackend_SetVelocityLimit(float dps);
uint8_t YawBackend_SetFeedforward(char parameter, float value);
uint8_t YawBackend_SetShape(uint32_t duration_ms);
uint8_t YawBackend_SetShapeVelocityGain(float gain);
uint8_t YawBackend_CaptureReference(void);
/* 先安全OFF/ZERO、ON和人工CH5重解锁。STEP只在健康静止近零活动状态接受；
 * ±30是相对软件零点的绝对目标，完成5秒测量后继续保持，不自动失能。 */
uint8_t YawBackend_StartStep(float degrees);
/* 控制任务只需编号时避免复制完整测量快照；不消费报告。 */
uint32_t YawBackend_GetStepId(void);
void YawBackend_GetStep(YawStep *out);
uint8_t YawBackend_TakeStepReport(YawStep *out);
typedef struct { uint32_t sequence; float target_deg; uint8_t result; uint32_t blocks; } YawRemoteStepEvent;
/* 1=开始，2=回参考，3=门禁拒绝，4=Pitch未接驱动；只报告新的遥控事件。 */
uint8_t YawBackend_TakeRemoteStepEvent(YawRemoteStepEvent *out);
/* 人工单次FB：Yaw请求关闭、通信故障13、连续静止、温度正常才接受。
 * 仅尚未尝试FC、请求关闭、连续状态13监测产生的速度/跳变可附带恢复；
 * 须静止500ms。使能后的运动故障及其它故障仍拒绝，STOP不抹掉来源。
 * 不直接清软件故障，不使能；提交后先完成新的FD才可再次准备。 */
uint8_t YawBackend_ClearCommsFault(void);
void YawBackend_GetClearCheck(YawClearCheck *out);
uint8_t YawBackend_TakeRearmRequest(void);
#endif
