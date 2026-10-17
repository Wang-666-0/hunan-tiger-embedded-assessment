#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H
#include "remote_mapper.h"

typedef enum { SAFETY_SAFE = 0, SAFETY_READY = 1, SAFETY_ARMED = 2, SAFETY_FAULT = 3 } SafetyState;
typedef enum {
    SAFETY_NONE = 0, SAFETY_POWER_ON = 1, SAFETY_REMOTE = 2,
    SAFETY_FAILSAFE = 3, SAFETY_UART = 4, SAFETY_CAN = 5,
    SAFETY_TIMING = 6, SAFETY_STOP = 7, SAFETY_NOT_NEUTRAL = 8,
    SAFETY_ARM_POSITION = 9, SAFETY_FEEDBACK = 10, SAFETY_ACTUATOR = 11
} SafetyReason;

typedef struct {
    RemoteFrame remote;
    uint32_t now_ms, uart_epoch, can_epoch, stop_sequence;
    uint8_t control_late, backend_ready, feedback_ready, actuator_fault;
} SafetyInput;

typedef struct {
    SafetyState state;
    SafetyReason reason, last_fault;
    uint32_t last_sequence, uart_epoch, can_epoch, stop_sequence, barrier_ms;
    uint8_t have_sequence, off_seen, barrier_valid, operator_armed, output_allowed;
} SafetyManager;

void SafetyManager_Init(SafetyManager *manager);
/* 单调用方持有状态；故障优先于新帧解锁。允许uint32毫秒/序号自然回卷。 */
void SafetyManager_Step(SafetyManager *manager, const SafetyInput *input,
                        const RemoteCommand *command);
#endif
