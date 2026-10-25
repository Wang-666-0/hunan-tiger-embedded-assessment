/**
 * @file feeder_backend.h
 * @brief 拨盘执行后端
 * 职责：声明拨盘会话、配置、准备、单发/连发、停机及只读状态接口。
 * 主要接口：FeederBackend_Init, FeederBackend_Stop, FeederBackend_ManualOff, FeederBackend_OnFrame, FeederBackend_Apply 等。
 * 实现：motors/feeder_backend.c；调用关系见实现文件与 App/README。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef FEEDER_BACKEND_H
#define FEEDER_BACKEND_H
#include "app_config.h"
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
                         uint8_t wheels_ready, uint8_t inhibited);
void FeederBackend_GetState(FeederBackendState *state);
uint8_t FeederBackend_TakeRearmRequest(void);
/* Preparation/configuration only in fresh neutral CH5 OFF / CH6 OFF / CH8 release.
 * ARM never moves. SINGLE needs armed normal mode; independent feeder test
 * requires CH6 middle instead of wheel readiness and forbids wheel output.
 * CH6 high enables continuous feed in this mode; normal builds retain real
 * two-wheel readiness for both modes (see app_config.h).
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
