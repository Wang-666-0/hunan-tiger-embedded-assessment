/**
 * @file flywheel_jog.h
 * @brief 双摩擦轮受限会话
 * 职责：定义左右轮反馈、速度控制会话、过流过速等保护及输出请求。
 * 主要接口：FlywheelJog_Init, FlywheelJog_OnFrame, FlywheelJog_GetHealth, FlywheelJog_Select, FlywheelJog_Step 等。
 * 实现：motors/flywheel_jog.c；调用关系见实现文件与 App/README。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef FLYWHEEL_JOG_H
#define FLYWHEEL_JOG_H
#include "can_frame.h"
#include "app_config.h"
#include "flywheel_speed.h"
#include <stdint.h>


typedef enum {
    FLYWHEEL_JOG_OFF = 0, FLYWHEEL_JOG_LEFT = 1, FLYWHEEL_JOG_RIGHT = 2,
    FLYWHEEL_JOG_BOTH = 3
} FlywheelJogSelection;

typedef enum {
    FLYWHEEL_STOP_NONE = 0, FLYWHEEL_STOP_INPUT = 1,
    FLYWHEEL_STOP_PERMISSION = 2, FLYWHEEL_STOP_FAULT = 3,
    FLYWHEEL_STOP_MANUAL = 4, FLYWHEEL_STOP_DURATION = 5,
    FLYWHEEL_STOP_NOT_STOPPED = 6
} FlywheelStopReason;

typedef enum {
    FLYWHEEL_JOG_FAULT_NONE = 0,
    FLYWHEEL_JOG_FAULT_FEEDBACK = 1U << 0,
    FLYWHEEL_JOG_FAULT_FRAME = 1U << 1,
    FLYWHEEL_JOG_FAULT_TEMPERATURE = 1U << 2,
    FLYWHEEL_JOG_FAULT_CURRENT = 1U << 3,
    FLYWHEEL_JOG_FAULT_OVERSPEED = 1U << 4,
    FLYWHEEL_JOG_FAULT_CAN_EPOCH = 1U << 5,
    FLYWHEEL_JOG_FAULT_EXTERNAL = 1U << 6
} FlywheelJogFault;

typedef struct {
    uint32_t status1_ms, status4_ms;
    int32_t erpm, motor_current_mA, input_current_mA;
    int16_t fet_deci_c, motor_deci_c, duty_milli;
    uint8_t has_status1, has_status4, motor_temperature_valid;
    uint8_t seen_status1, seen_status4; /* 保留无效帧后的时间屏障。 */
} FlywheelJogFeedback;

typedef struct {
    FlywheelJogFeedback feedback[2]; /* 下标0=node70，下标1=node71。 */
    uint32_t faults;
    uint8_t status1_fresh_mask, status4_fresh_mask, fresh, stopped, valid;
} FlywheelJogHealth;

typedef struct {
    FlywheelJogFeedback feedback[2];
    FlywheelSpeed speed; /* 单轮所选控制器，双轮时为左轮，保持旧接口含义。 */
    FlywheelSpeed speed_right; /* 仅双轮时独立运行，不能共用左轮积分。 */
    uint32_t can_epoch, faults, selected_ms, started_ms, last_ramp_ms, last_command_ms;
    int32_t current_mA, right_current_mA; /* 主控制器与双轮右控制器的非负幅值。 */
    FlywheelJogSelection selection;
    FlywheelJogSelection last_speed_selection; /* 停机后仍标记统计属于哪只轮。 */
    FlywheelStopReason stop_reason;
    uint8_t has_epoch, active, ready, reset_seen, last_allowed, last_centered;
    uint8_t command_started, pending_mask, coasting;
} FlywheelJog;

/* 纯逻辑：不调用HAL/RTOS/CAN发送。所有调用由上层同一临界区串行化。
 * Init默认OFF且不排命令；0mA为滑行，不能保证瞬间停转。
 * now_ms与frame.received_ms必须来自同一个uint32毫秒时钟。 */
void FlywheelJog_Init(FlywheelJog *jog);
uint8_t FlywheelJog_OnFrame(FlywheelJog *jog, const CanFrame *frame,
                          uint32_t now_ms, uint32_t can_epoch);
void FlywheelJog_GetHealth(FlywheelJog *jog, uint32_t now_ms,
                          uint32_t can_epoch, FlywheelJogHealth *health);
/* OFF立即停止；LEFT/RIGHT/BOTH要求当前OFF、禁止输出、回中、无活动、零请求、
 * 两轮新鲜停转且can_epoch==0。任何非0 epoch须断动力后复位，不能现场重置。 */
uint8_t FlywheelJog_Select(FlywheelJog *jog, FlywheelJogSelection selection,
                         uint32_t now_ms, uint32_t can_epoch);
/* positive_request为映射/死区后的yaw_rate>0；centered为yaw_rate==0。
 * 每次点动结束必须先观察到output_allowed=0且centered=1，随后新的
 * OFF->ON且仍回中，才能等待下一次正向输入。负向输入不会满足回中。
 * 选择同毫秒不能累计OFFseen；上层还必须拒绝选择前缓存的遥控帧。
 * 默认按住首档受限电流稳速；关闭速度环时回归1000/600观察。
 * 软上界暂时0mA且保留活动，硬故障/撤权仍结束会话。
 * 时长配置非0时回归受限点动，不启用观察区间。两轮反馈须持续新鲜。
 * 上层仍须处理遥控过期、CH5解锁、循环超时、CAN发送故障并调用Fault。
 * 上层若尚未成功发送所有0mA，不得重新允许输出。 */
void FlywheelJog_Step(FlywheelJog *jog, uint32_t now_ms,
                     uint8_t output_allowed, uint8_t positive_request,
                     uint8_t centered, uint32_t can_epoch);
/* 按app_config.h的节点上限和固定极性生成请求；OFF为0。 */
int32_t FlywheelJog_GetRequestedCurrent(const FlywheelJog *jog);
/* 每个节点独立的带方向请求；双轮时旧GetRequestedCurrent仅表示左轮。 */
int32_t FlywheelJog_GetNodeCurrent(const FlywheelJog *jog, uint8_t node);
/* drain到返回0。周期20ms；仅70/71，按节点独立限流，未选轮永远0。
 * 新停止覆盖尚未取走的非零命令。发送失败由上层处理，不能丢弃停止。 */
uint8_t FlywheelJog_TakeCommand(FlywheelJog *jog, uint8_t *node,
                               int32_t *current_mA);
void FlywheelJog_Fault(FlywheelJog *jog, uint32_t faults);
/* 明确点动会话中才调用；立即排双0，取消本次点动并要求重新OFF->ON。 */
void FlywheelJog_Stop(FlywheelJog *jog);
/* 清故障要求OFF、禁止输出且已回中、两轮新鲜停转；不会恢复选择/点动。 */
uint8_t FlywheelJog_ClearFault(FlywheelJog *jog, uint32_t now_ms,
                             uint32_t can_epoch);
#endif
