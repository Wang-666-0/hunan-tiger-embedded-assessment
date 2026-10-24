#ifndef FEEDER_BACKEND_H
#define FEEDER_BACKEND_H
#include "feeder_controller.h"
#include "target_manager.h"
typedef struct {
    FeederController controller;
    uint32_t single_accepted, continuous_accepted, completed_interval_ms;
    uint8_t armed, online, stop_pending;
} FeederBackendState;
void FeederBackend_Init(void);
void FeederBackend_Stop(void);
/* Console/VOFA OFF holds automatic preparation for deliberate tuning, until ARM/READY. */
void FeederBackend_ManualOff(void);
uint8_t FeederBackend_OnFrame(const CanFrame *frame);
void FeederBackend_Apply(const TargetRequest *request, uint8_t output_allowed,
                         uint8_t wheels_ready, uint8_t diagnostic);
void FeederBackend_GetState(FeederBackendState *state);
uint8_t FeederBackend_TakeRearmRequest(void);
/* Preparation/configuration only in fresh neutral CH5 OFF / CH6 OFF / CH8 release.
 * ARM never moves. SINGLE additionally needs armed normal mode and real wheel readiness.
 * Once accepted, finish that step on a speed dip; loss of permission/health still stops. */
uint8_t FeederBackend_SetConfig(const FeederConfig *config);
uint8_t FeederBackend_Clear(void);
uint8_t FeederBackend_Zero(void);
uint8_t FeederBackend_Arm(void);
/* Safe-OFF manual CLEAR + ZERO + ARM using this installation's compiled calibration. */
uint8_t FeederBackend_Ready(void);
/* Automatic safe-OFF preparation, after gimbal readiness. Never clears faults. */
uint8_t FeederBackend_PrepareRemote(void);
uint8_t FeederBackend_Single(void);
#endif
