#include "pitch_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "task.h"
#include <math.h>
#include <string.h>

static PitchBackendState state;
static PitchStep step;
static PitchRemoteStepEvent remote_event;
static uint32_t last_step_sequence, last_tx_ms, enable_ms, disable_ms, last_query_ms;
static uint8_t rearm_request, remote_pending, cancel_pending;
static uint8_t disable_submitted, actuation_started, last_mode;
static float submitted_torque;
static uint8_t submitted_limited;

static uint8_t Online(uint32_t now)
{
    CanBusState bus;
    CanBus_GetState(&bus);
    return (uint8_t)(state.controller.have_feedback && bus.started && !bus.fault_epoch &&
        (uint32_t)(now - state.feedback.received_ms) < PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        state.feedback.fault_epoch == bus.fault_epoch);
}

void PitchBackend_Init(void)
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    PitchController_Init(&state.controller); PitchStep_Init(&step);
    memset(&remote_event, 0, sizeof(remote_event));
    last_step_sequence = last_tx_ms = enable_ms = disable_ms = last_query_ms = 0U;
    rearm_request = remote_pending = cancel_pending = 0U;
    disable_submitted = actuation_started = 0U; last_mode = GIMBAL_MANUAL;
    submitted_torque = 0.0f; submitted_limited = 0U;
    /* Do not transmit FC or FD merely because the MCU was reset. */
    taskEXIT_CRITICAL();
}

void PitchBackend_GetState(PitchBackendState *out)
{
    uint32_t now;
    if (!out) { return; }
    taskENTER_CRITICAL(); now = HAL_GetTick();
    state.online = Online(now);
    state.feedback_age_ms = state.controller.have_feedback ? now - state.feedback.received_ms : 0U;
    *out = state; taskEXIT_CRITICAL();
}

uint8_t PitchBackend_QueryFeedback(void)
{
    uint8_t result;
    taskENTER_CRITICAL();
    result = (uint8_t)(CanBus_PitchReadFeedback() == HAL_OK);
    if (result) { state.monitoring = 1U; last_query_ms = HAL_GetTick(); }
    taskEXIT_CRITICAL(); return result;
}

static void StopWithReason(PitchStopReason reason)
{
    uint8_t had_session;
    taskENTER_CRITICAL();
    had_session = (uint8_t)(state.active || state.enabling || actuation_started);
    PitchStep_Abort(&step, HAL_GetTick(), state.controller.faults ? PITCH_STEP_FAULT : PITCH_STEP_STOP,
        state.controller.faults);
    CanBus_SetPitchPermission(0U);
    PitchController_Stop(&state.controller);
    state.requested = state.active = state.enabling = state.step_hold = 0U;
    state.view_hold_pending = state.view_locked;
    state.view_prepare_pending = 0U;
    state.view_hold_suspended = state.view_locked;
    last_mode = GIMBAL_MANUAL;
    if (had_session && !state.stop_pending) {
        ++state.stop_id; state.stopped_ms = HAL_GetTick(); state.stop_reason = reason;
        state.stop_pending = 1U; cancel_pending = 1U; disable_submitted = 0U;
        disable_ms = HAL_GetTick();
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_Stop(void) { StopWithReason(PITCH_STOP_EXTERNAL); }

uint8_t PitchBackend_Request(uint8_t enabled)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (!enabled) {
        /* Explicit OFF requests FD even if the user has only queried the motor. */
        actuation_started = 1U; PitchBackend_Stop(); accepted = 1U;
    } else if (APP_PITCH_ENABLED && state.coord_confirmed && state.zero_set &&
        state.controller.have_reference && Online(HAL_GetTick()) && state.feedback.status == 0U &&
        !state.controller.faults && !state.active && !state.enabling && !state.stop_pending) {
        state.requested = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetCoordinate(uint8_t divisor)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((divisor == 1U || divisor == 7U) && !state.requested && !state.active &&
        !state.enabling && !state.stop_pending && Online(HAL_GetTick()) && state.feedback.status == 0U) {
        accepted = PitchController_SetFeedbackDivisor(&state.controller, divisor);
        if (accepted) {
            state.coord_confirmed = 1U; state.zero_set = 0U;
            state.controller.have_reference = 0U;
            PitchController_InvalidateGravity(&state.controller);
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_TARGET_CHANGE, 0U);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_CaptureReference(void)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (state.coord_confirmed && !state.requested && !state.active && !state.enabling &&
        !state.stop_pending && Online(HAL_GetTick()) && state.feedback.status == 0U) {
        uint32_t faults = state.controller.faults;
        /* Healthy disabled feedback can recover a prior enable timeout/disabled report.
         * TX/CAN faults remain latched; changing coordinates is not a bus recovery. */
        state.controller.faults &= ~(PITCH_BACKEND_FAULT_ENABLE | PITCH_BACKEND_FAULT_DISABLED);
        accepted = PitchController_CaptureReference(&state.controller);
        if (accepted) { state.zero_set = 1U; actuation_started = 0U; }
        else { state.controller.faults |= faults; }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_OnFrame(const CanFrame *frame)
{
    Dm3507Feedback feedback;
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    if (!Dm3507_Decode(frame, &feedback)) { return 0U; }
    taskENTER_CRITICAL();
    if ((uint32_t)(now - feedback.received_ms) < PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        (!state.controller.have_feedback || (int32_t)(feedback.received_ms - state.feedback.received_ms) > 0)) {
        CanBusState bus;
        CanBus_GetState(&bus);
        if (feedback.fault_epoch != bus.fault_epoch) { taskEXIT_CRITICAL(); return 0U; }
        state.feedback = feedback; ++state.rx_count;
        accepted = PitchController_OnFeedback(&state.controller, &feedback);
        if (feedback.status == 1U) { actuation_started = 1U; }
        if (!state.controller.have_reference) { state.zero_set = 0U; }
        /* Status0 confirms FD, not a brake or stationary mechanical support. */
        if (state.stop_pending && disable_submitted && !cancel_pending && feedback.status == 0U &&
            (int32_t)(feedback.received_ms - disable_ms) > 0) {
            state.stop_pending = 0U; actuation_started = 0U;
        }
        if (state.active && feedback.status == 0U) { state.controller.faults |= PITCH_BACKEND_FAULT_DISABLED; }
        if (step.phase == PITCH_STEP_RUNNING) {
            if (state.controller.faults) {
                PitchStep_Abort(&step, feedback.received_ms, PITCH_STEP_FAULT, state.controller.faults);
            } else if (state.active && feedback.status == 1U && CanBus_PitchAllowed()) {
                PitchStep_Sample(&step, feedback.received_ms, state.controller.angle_deg,
                    state.controller.velocity_dps, submitted_torque, submitted_limited);
            } else { PitchStep_Abort(&step, feedback.received_ms, PITCH_STEP_STOP, 0U); }
        }
        if (state.controller.faults && (state.active || state.enabling)) {
            StopWithReason(PITCH_STOP_FEEDBACK_FAULT); rearm_request = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetLoopPid(char loop, float kp, float ki, float kd)
{
    uint8_t accepted;
    uint32_t revision;
    taskENTER_CRITICAL(); revision = state.controller.cascade.revision;
    accepted = PitchController_SetPid(&state.controller, loop, kp, ki, kd);
    if (accepted && revision != state.controller.cascade.revision) {
        PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_PID_CHANGE, 0U);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetLoopParameter(char loop, char parameter, float value)
{
    YawCascadePid pid;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (loop == 'P' || loop == 'V') {
        pid = loop == 'P' ? state.controller.cascade.position : state.controller.cascade.speed;
        if (parameter == 'P') { pid.kp = value; }
        else if (parameter == 'I') { pid.ki = value; }
        else if (parameter == 'D') { pid.kd = value; }
        if (parameter == 'P' || parameter == 'I' || parameter == 'D') {
            accepted = PitchBackend_SetLoopPid(loop, pid.kp, pid.ki, pid.kd);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetTarget(float degrees)
{
    uint8_t accepted = 0U;
    uint32_t now;
    float previous;
    taskENTER_CRITICAL();
    now = HAL_GetTick();
    if (!state.view_locked && state.active && state.requested && state.zero_set && Online(now) &&
        state.feedback.status == 1U && !state.controller.faults && CanBus_PitchAllowed()) {
        previous = state.controller.target_deg;
        accepted = PitchController_StartMove(&state.controller, degrees, now);
        if (accepted) {
            if (previous != degrees) { PitchStep_Abort(&step, now, PITCH_STEP_TARGET_CHANGE, 0U); }
            state.step_hold = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetGravityGain(float kg_nm)
{
    uint8_t accepted = 0U;
    float previous;
    taskENTER_CRITICAL(); previous = state.controller.gravity_kg_nm;
    if (kg_nm == 0.0f || (state.coord_confirmed && state.zero_set && Online(HAL_GetTick()) &&
        !state.enabling && !state.stop_pending && !state.controller.faults &&
        ((!state.active && state.feedback.status == 0U) ||
         (state.active && state.feedback.status == 1U && CanBus_PitchAllowed())))) {
        accepted = PitchController_SetGravityGain(&state.controller, kg_nm);
        if (accepted && previous != state.controller.gravity_kg_nm) {
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_FF_CHANGE, 0U);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t PitchBackend_SetGravityOffset(float offset_deg, uint8_t capture_horizontal)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (state.coord_confirmed && state.zero_set && Online(HAL_GetTick()) &&
        !state.active && !state.enabling && !state.stop_pending && state.feedback.status == 0U &&
        !state.controller.faults) {
        accepted = capture_horizontal ? PitchController_CaptureHorizontal(&state.controller) :
            PitchController_SetGravityOffset(&state.controller, offset_deg);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_StartStep(float degrees)
{
    uint8_t accepted = 0U;
    uint32_t now;
    taskENTER_CRITICAL();
    now = HAL_GetTick();
    if (!state.view_locked && (degrees == 30.0f || degrees == -30.0f) && state.coord_confirmed && state.zero_set &&
        state.requested && state.active && state.controller.active && !state.enabling &&
        !state.stop_pending && Online(now) && state.feedback.status == 1U &&
        !state.controller.faults && CanBus_PitchAllowed() &&
        fabsf(state.controller.angle_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(state.controller.target_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(state.controller.velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        state.controller.stationary_samples >= 2U &&
        state.controller.last_feedback_ms - state.controller.stationary_started_ms >=
            PITCH_CONTROLLER_STATIONARY_HOLD_MS && step.phase != PITCH_STEP_RUNNING) {
        if (PitchController_StartMove(&state.controller, degrees, now)) {
            accepted = PitchStep_Start(&step, now, state.controller.angle_deg, degrees);
            if (accepted) {
                step.kp = state.controller.cascade.position.kp;
                step.ki = state.controller.cascade.position.ki;
                step.kd = state.controller.cascade.position.kd;
                step.vkp = state.controller.cascade.speed.kp;
                step.vki = state.controller.cascade.speed.ki;
                step.vkd = state.controller.cascade.speed.kd;
                step.gravity_kg_nm = state.controller.gravity_kg_nm;
                step.gravity_offset_deg = state.controller.gravity_offset_deg;
                step.gravity_calibrated = state.controller.gravity_calibrated;
                step.shape_ms = state.controller.shape.duration_ms;
                step.shape_velocity_gain = state.controller.shape.velocity_gain;
                step.pitch_brake_enabled = state.controller.brake.enabled;
                step.cascade = 1U; step.output_limits = 0U; step.vmax_dps = 0.0f;
                step.last_velocity_dps = state.controller.velocity_dps;
                step.last_torque_nm = submitted_torque;
                state.step_hold = 1U;
            }
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetShape(uint32_t duration_ms, float velocity_gain)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (step.phase != PITCH_STEP_RUNNING ||
        (duration_ms == state.controller.shape.duration_ms &&
         velocity_gain == state.controller.shape.velocity_gain)) {
        accepted = PitchController_SetShape(&state.controller, duration_ms, velocity_gain);
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t PitchBackend_SetBrakeEnabled(uint8_t enabled)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (step.phase != PITCH_STEP_RUNNING || enabled == state.controller.brake.enabled) {
        accepted = PitchController_SetBrakeEnabled(&state.controller, enabled);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint32_t PitchBackend_GetStepId(void)
{
    uint32_t id;
    taskENTER_CRITICAL(); id = step.id; taskEXIT_CRITICAL();
    return id;
}
void PitchBackend_GetStep(PitchStep *out)
{ if (out) { taskENTER_CRITICAL(); *out = step; taskEXIT_CRITICAL(); } }
uint8_t PitchBackend_TakeStepReport(PitchStep *out)
{
    uint8_t result = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (step.pending) { *out = step; step.pending = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t PitchBackend_TakeRemoteStepEvent(PitchRemoteStepEvent *out)
{
    uint8_t result = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (remote_pending) { *out = remote_event; remote_pending = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t PitchBackend_TakeRearmRequest(void)
{
    uint8_t result;
    taskENTER_CRITICAL(); result = rearm_request; rearm_request = 0U; taskEXIT_CRITICAL(); return result;
}
static void Notice(const TargetRequest *request, uint8_t result)
{
    remote_event.sequence = request->step_sequence;
    remote_event.target_deg = result == 1U || result == 2U ? state.controller.target_deg :
        (request->step_return_event ? state.controller.target_deg + request->step_delta_deg : request->step_delta_deg);
    remote_event.result = result; remote_pending = 1U;
}

static uint8_t StationarySupport(void)
{
    const PitchController *pitch = &state.controller;
    return (uint8_t)(fabsf(pitch->target_deg - pitch->angle_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(pitch->velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        fabsf(pitch->cascade.velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        pitch->stationary_samples >= 2U &&
        pitch->last_feedback_ms - pitch->stationary_started_ms >= PITCH_CONTROLLER_STATIONARY_HOLD_MS);
}

static void TransferPositionSupport(void)
{
    float next = state.controller.cascade.speed_i +
        state.controller.cascade.speed.kp * state.controller.cascade.position_i;
    if (next > PITCH_CONTROLLER_TORQUE_LIMIT_NM) { next = PITCH_CONTROLLER_TORQUE_LIMIT_NM; }
    else if (next < -PITCH_CONTROLLER_TORQUE_LIMIT_NM) { next = -PITCH_CONTROLLER_TORQUE_LIMIT_NM; }
    state.controller.cascade.speed_i = next;
    state.controller.cascade.position_i = 0.0f;
}

uint8_t PitchBackend_HoldCurrent(void)
{
    uint8_t accepted = 0U;
    uint8_t carry_support;
    float target;
    taskENTER_CRITICAL();
    carry_support = StationarySupport();
    target = state.controller.angle_deg;
    if (target > PITCH_CONTROLLER_TARGET_MAX_DEG &&
        target <= PITCH_CONTROLLER_TARGET_MAX_DEG + PITCH_STEP_BAND_DEG) {
        target = PITCH_CONTROLLER_TARGET_MAX_DEG;
    } else if (target < PITCH_CONTROLLER_TARGET_MIN_DEG &&
        target >= PITCH_CONTROLLER_TARGET_MIN_DEG - PITCH_STEP_BAND_DEG) {
        target = PITCH_CONTROLLER_TARGET_MIN_DEG;
    }
    if (state.active && state.requested && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 1U && CanBus_PitchAllowed() &&
        !state.controller.faults) {
        if (state.view_locked && !state.view_hold_pending) { accepted = 1U; }
        else if (PitchController_SetTarget(&state.controller, target)) {
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_MANUAL, 0U);
            if (state.view_locked) {
                /* Only a settled near-target outer I is treated as load
                 * support. Motion I must not become a persistent torque bias
                 * after cancelling that movement and latching a new pose. */
                if (carry_support) { TransferPositionSupport(); }
                else { state.controller.cascade.position_i = 0.0f; }
                state.view_hold_deg = target;
            } else { state.controller.cascade.position_i = 0.0f; }
            /* Gravity and existing inner load compensation remain active. */
            state.step_hold = 1U; state.view_hold_pending = state.view_prepare_pending = 0U; accepted = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

void PitchBackend_SetViewLock(uint8_t locked)
{
    taskENTER_CRITICAL(); locked = (uint8_t)(locked != 0U);
    if (state.view_locked != locked) {
        if (!locked && state.active && !state.view_hold_pending && !state.controller.faults) {
            if (StationarySupport()) { TransferPositionSupport(); }
            else { state.controller.cascade.position_i = 0.0f; }
        }
        state.view_locked = state.view_hold_pending = locked;
        state.view_prepare_pending = locked;
        state.view_hold_suspended = 0U;
        if (!locked) { state.step_hold = 0U; }
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_PrepareViewHold(void)
{
    taskENTER_CRITICAL();
    if (!state.active && !state.enabling) {
        state.view_prepare_pending = 1U;
        state.view_hold_suspended = 0U;
        /* Refresh stopped-motor feedback through the existing read-only CC
         * path, rather than waiting forever on an expired last frame. */
        state.monitoring = 1U;
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic)
{
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    uint8_t wanted, event = 0U, new_step = 0U;
    taskENTER_CRITICAL();
    if (request && request->step_event && request->step_sequence != last_step_sequence) {
        last_step_sequence = request->step_sequence;
        new_step = 1U;
        event = (uint8_t)(request->step_axis == GIMBAL_STEP_PITCH);
    }
    if ((state.active || state.enabling) && !Online(now)) {
        state.controller.faults |= PITCH_CONTROLLER_FAULT_FEEDBACK;
        StopWithReason(PITCH_STOP_FEEDBACK); rearm_request = 1U;
    }
    /* Selected view is durable intent, unlike V3's one-shot ticket which READY
     * could clear permanently. A stop made while permission is still live
     * suspends reconciliation. Observe blocked output (normal global STOP/OFF
     * and rearm path), or receive a fresh VIEW, before joining again. */
    if (!allowed) { state.view_hold_suspended = 0U; }
    if (state.view_locked && state.view_hold_pending && !state.requested &&
        !state.active && !state.enabling && !state.view_hold_suspended) {
        state.view_prepare_pending = 1U; state.monitoring = 1U;
    }
    /* Reuse Request + original FC/new status1 handshake. Existing reference,
     * calibration, faults, fresh feedback and global permission remain gates. */
    if (state.view_locked && state.view_hold_pending && !state.view_hold_suspended &&
        !state.requested && allowed && !diagnostic && request &&
        state.coord_confirmed && state.zero_set && state.controller.have_reference &&
        !state.active && !state.enabling && !state.stop_pending && !state.controller.faults &&
        Online(now) && state.feedback.status == 0U &&
        (state.controller.gravity_kg_nm == 0.0f || state.controller.gravity_calibrated)) {
        if (PitchBackend_Request(1U)) { state.view_prepare_pending = 0U; }
    }
    wanted = (uint8_t)(APP_PITCH_ENABLED && state.requested && state.coord_confirmed &&
        state.zero_set && allowed && !diagnostic && request && Online(now) &&
        !state.controller.faults && !state.stop_pending &&
        (state.controller.gravity_kg_nm == 0.0f || state.controller.gravity_calibrated));
    if (!wanted) {
        if (event) { Notice(request, 3U); }
        if (state.active || state.enabling) { StopWithReason(PITCH_STOP_PERMISSION); rearm_request = 1U; }
        CanBus_SetPitchPermission(0U);
        if (state.stop_pending && (!disable_submitted || now - last_tx_ms >= 10U)) {
            result = CanBus_PitchDisable(cancel_pending); last_tx_ms = now;
            if (result == HAL_OK) {
                if (!disable_submitted) { disable_ms = now; }
                disable_submitted = 1U; cancel_pending = 0U;
                submitted_torque = 0.0f; submitted_limited = 0U;
            }
        }
        /* The user's explicit CHECK/READY starts read-only monitoring. This keeps
         * disabled/ready feedback current while the user reads a summary or moves CH5.
         * No monitoring is started automatically on MCU reset. */
        if (state.monitoring && now - last_query_ms >= 10U) {
            if (CanBus_PitchReadFeedback() == HAL_OK) { last_query_ms = now; }
        }
        taskEXIT_CRITICAL(); return;
    }
    CanBus_SetPitchPermission(1U);
    if (!CanBus_PitchAllowed()) {
        StopWithReason(PITCH_STOP_CAN_PERMISSION); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (!state.active && !state.enabling) {
        if (event) { Notice(request, 3U); }
        /* Preserve the measured reference, but enable holding the current supported pose. */
        if (state.controller.angle_deg < PITCH_CONTROLLER_TARGET_MIN_DEG ||
            state.controller.angle_deg > PITCH_CONTROLLER_TARGET_MAX_DEG ||
            !PitchController_SetTarget(&state.controller, state.controller.angle_deg)) {
            state.controller.faults |= PITCH_CONTROLLER_FAULT_LIMIT;
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        result = CanBus_PitchEnable(); last_tx_ms = now;
        if (result == HAL_OK) {
            state.enabling = 1U; actuation_started = 1U; enable_ms = now;
        } else {
            state.controller.faults |= PITCH_BACKEND_FAULT_TX;
            StopWithReason(PITCH_STOP_ENABLE_TX); rearm_request = 1U;
        }
        taskEXIT_CRITICAL(); return;
    }
    if (state.enabling) {
        if (event) { Notice(request, 3U); }
        if (state.feedback.status == 1U && (int32_t)(state.feedback.received_ms - enable_ms) > 0) {
            state.enabling = 0U; state.active = state.controller.active = 1U;
        } else if (now - enable_ms >= APP_PITCH_ENABLE_TIMEOUT_MS) {
            state.controller.faults |= PITCH_BACKEND_FAULT_ENABLE;
            StopWithReason(PITCH_STOP_ENABLE_TIMEOUT); rearm_request = 1U;
        } else if (now - last_tx_ms >= 10U) {
            result = CanBus_PitchEnable(); last_tx_ms = now;
            if (result != HAL_OK) {
                state.controller.faults |= PITCH_BACKEND_FAULT_TX;
                StopWithReason(PITCH_STOP_ENABLE_TX); rearm_request = 1U;
            }
        }
        taskEXIT_CRITICAL(); return;
    }
    if (last_mode == GIMBAL_TEST && request->gimbal_mode != GIMBAL_TEST) {
        PitchStep_Abort(&step, now, PITCH_STEP_MANUAL, 0U);
        StopWithReason(PITCH_STOP_MODE_EXIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (state.view_locked && state.view_hold_pending && !PitchBackend_HoldCurrent()) {
        StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (event && state.view_locked) { Notice(request, 3U); event = 0U; }
#if APP_REMOTE_PROFILE_ASSESSMENT
    if (request->gimbal_mode == GIMBAL_TEST &&
        (last_mode != GIMBAL_TEST || (new_step && request->step_axis == GIMBAL_STEP_YAW))) {
        if (!PitchBackend_HoldCurrent()) {
            if (event) { Notice(request, 3U); }
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
    }
#endif
    last_mode = (uint8_t)request->gimbal_mode;
    if (event) {
        if (request->gimbal_mode != GIMBAL_TEST) { Notice(request, 3U); }
        else if (request->step_return_event) {
            Notice(request, PitchBackend_SetTarget(state.controller.target_deg + request->step_delta_deg) ? 2U : 3U);
        } else if (PitchBackend_StartStep(request->step_delta_deg)) {
            Notice(request, 1U);
        } else { Notice(request, 3U); }
    }
    if (!state.view_locked && state.step_hold && request->gimbal_mode == GIMBAL_MANUAL && request->pitch_rate_dps != 0.0f) {
        PitchStep_Abort(&step, now, PITCH_STEP_MANUAL, 0U);
        StopWithReason(PITCH_STOP_MANUAL_OVERRIDE); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (state.view_locked) {
        /* The latched target is independent of disturbed feedback. Never
         * capture again until unlock/rearm; leave all original safety gates. */
        if ((state.controller.target_deg != state.view_hold_deg || state.controller.shape.active ||
             state.controller.brake.tracking) && !PitchController_SetTarget(&state.controller, state.view_hold_deg)) {
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        state.step_hold = 1U;
        (void)PitchController_UpdateViewHold(&state.controller, now, 1U);
    } else {
        (void)PitchController_Update(&state.controller,
            request->gimbal_mode == GIMBAL_MANUAL && !state.step_hold ? request->pitch_rate_dps : 0.0f, now, 1U);
    }
    if (state.controller.faults) {
        StopWithReason(PITCH_STOP_CONTROL_FAULT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (now - last_tx_ms >= APP_PITCH_COMMAND_PERIOD_MS) {
        result = CanBus_PitchTorque(state.controller.torque_nm); last_tx_ms = now;
        if (result != HAL_OK) {
            state.controller.faults |= PITCH_BACKEND_FAULT_TX;
            StopWithReason(PITCH_STOP_TORQUE_TX); rearm_request = 1U;
        } else {
            submitted_torque = state.controller.torque_nm;
            submitted_limited = state.controller.cascade.torque_limited;
            PitchStep_Command(&step, now, submitted_torque);
        }
    }
    taskEXIT_CRITICAL();
}
