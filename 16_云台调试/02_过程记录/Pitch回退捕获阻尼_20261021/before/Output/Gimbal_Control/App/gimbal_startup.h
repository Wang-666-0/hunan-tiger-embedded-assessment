#ifndef GIMBAL_STARTUP_H
#define GIMBAL_STARTUP_H
#include "safety_manager.h"

typedef enum {
    GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_PREPARING, GIMBAL_STARTUP_READY,
    GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_MANUAL
} GimbalStartupPhase;
typedef enum {
    GIMBAL_STARTUP_NONE, GIMBAL_STARTUP_REMOTE, GIMBAL_STARTUP_CAN,
    GIMBAL_STARTUP_FAULT, GIMBAL_STARTUP_REFERENCE_LOST, GIMBAL_STARTUP_DIAGNOSTIC,
    GIMBAL_STARTUP_STOP, GIMBAL_STARTUP_TIMEOUT
} GimbalStartupReason;
typedef struct {
    uint32_t sequence;
    uint8_t phase, reason, ready, reference_latched;
} GimbalStartupState;
void GimbalStartup_Init(void);
/* ControlTask only, before health aggregation; no UART formatting or transmission. */
void GimbalStartup_Update(const SafetyInput *input, uint8_t diagnostic);
uint8_t GimbalStartup_OutputReady(void);
/* Explicit manual setup takes ownership until GIMBAL READY or reboot. */
void GimbalStartup_ManualOverride(void);
/* Manual declaration: current supported Pitch pose is the measured horizontal baseline. */
uint8_t GimbalStartup_Request(const RemoteFrame *frame, uint32_t now_ms, uint8_t diagnostic);
void GimbalStartup_GetState(GimbalStartupState *out);
uint8_t GimbalStartup_TakeEvent(GimbalStartupState *out);
#endif
