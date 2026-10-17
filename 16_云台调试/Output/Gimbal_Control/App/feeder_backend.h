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
uint8_t FeederBackend_OnFrame(const CanFrame *frame);
void FeederBackend_Apply(const TargetRequest *request, uint8_t output_allowed,
                         uint8_t wheels_ready, uint8_t diagnostic);
void FeederBackend_GetState(FeederBackendState *state);
uint8_t FeederBackend_TakeRearmRequest(void);
/* Preparation/configuration only in fresh neutral CH5 OFF / CH6 OFF / CH8 release.
 * ARM never moves. SINGLE additionally needs armed normal mode and real wheel readiness. */
uint8_t FeederBackend_SetConfig(const FeederConfig *config);
uint8_t FeederBackend_Clear(void);
uint8_t FeederBackend_Zero(void);
uint8_t FeederBackend_Arm(void);
uint8_t FeederBackend_Single(void);
#endif
