#ifndef APP_LOGIC_H
#define APP_LOGIC_H
#include "safety_manager.h"
#include "target_manager.h"
typedef enum {
    APP_STOP_SOURCE_NONE = 0U, APP_STOP_SOURCE_SAFETY, APP_STOP_SOURCE_YAW,
    APP_STOP_SOURCE_PITCH, APP_STOP_SOURCE_WHEELS
} AppStopSource;
enum {
    APP_STOP_TRIGGER_STOP = 1U, APP_STOP_TRIGGER_UART = 2U, APP_STOP_TRIGGER_CAN = 4U,
    APP_STOP_TRIGGER_TIMING = 8U, APP_STOP_TRIGGER_REMOTE = 16U,
    APP_STOP_TRIGGER_FAILSAFE = 32U, APP_STOP_TRIGGER_ACTUATOR = 64U,
    APP_STOP_TRIGGER_FEEDBACK = 128U, APP_STOP_TRIGGER_CH5_OFF = 256U
};
typedef struct {
    uint32_t id, at_ms, remote_age_ms, uart_epoch, can_epoch, stop_sequence, triggers;
    uint16_t channels[8];
    uint8_t source, reason, previous_mode, mode;
} AppStopTrace;
typedef struct {
    SafetyManager safety;
    TargetManager targets;
    RemoteCommand command;
    /* 每次成功解锁后只保存第一次撤权；后续STOP/恢复/OFF不覆盖。只读诊断数据。 */
    AppStopTrace stop_trace;
    uint32_t trace_triggers;
    uint8_t trace_armed, trace_previous_mode;
} AppLogic;
typedef struct { uint8_t requested, online, stop_pending, faulted; } AppAxisHealth;
/* input初始为摩擦轮健康。仅明确请求运行的轴参与共享许可检查。 */
void AppLogic_CombineActuatorHealth(SafetyInput *input, const AppAxisHealth *yaw,
                                   const AppAxisHealth *pitch, uint8_t wheels_wanted);
void AppLogic_Init(AppLogic *logic);
/* 不依赖HAL/RTOS，正式固件与软件测试共同调用这份逻辑。 */
void AppLogic_Step(AppLogic *logic, const SafetyInput *input, uint8_t flywheels_ready);
/* 后端先报告撤权来源，再沿原路径STOP；不改变任何许可或安全判断。 */
void AppLogic_RecordStop(AppLogic *logic, const SafetyInput *input, AppStopSource source);
#endif
