#include "gimbal_startup.h"
#include "app_config.h"
#include "app_tasks.h"
#include "yaw_backend.h"
#include "pitch_backend.h"
#include "can_bus.h"
#include "task.h"
#include <string.h>

static GimbalStartupState state;
static uint32_t remote_sequence, guard_stop, guard_uart, guard_can;
static uint32_t barrier_ms, off_since_ms, started_ms, query_ms;
static uint8_t have_sequence, have_guard, off_seen, pending_event;
static uint8_t begun, yaw_zero_done, pitch_zero_done, clear_sent, manual_rezero;

static void Status(uint8_t phase, uint8_t reason, uint8_t ready)
{
    if (state.phase != phase || state.reason != reason || state.ready != ready) {
        state.phase = phase; state.reason = reason; state.ready = ready;
        ++state.sequence; pending_event = 1U;
    }
}
void GimbalStartup_Init(void)
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    have_sequence = have_guard = off_seen = pending_event = 0U;
    begun = yaw_zero_done = pitch_zero_done = clear_sent = 0U;
    manual_rezero = 0U;
    remote_sequence = guard_stop = guard_uart = guard_can = 0U;
    barrier_ms = off_since_ms = started_ms = query_ms = 0U;
    taskEXIT_CRITICAL();
}
static uint8_t SafeOff(const RemoteFrame *frame, uint32_t now)
{
    RemoteCommand command;
    if (!frame || !frame->has_frame || (uint32_t)(now - frame->received_ms) >= REMOTE_TIMEOUT_MS) { return 0U; }
    RemoteMapper_Map(frame, &command);
    return (uint8_t)(command.valid && command.failsafe_ok && command.arm == RC_LOW && command.neutral);
}
static void Barrier(uint32_t now)
{
    AppTasks_RequireRearm(); ++guard_stop;
    barrier_ms = now; off_seen = 0U;
}
static void CancelInitial(void)
{
    if (begun && !state.reference_latched) {
        (void)YawBackend_Request(0U); (void)PitchBackend_Request(0U);
    }
    begun = yaw_zero_done = pitch_zero_done = clear_sent = 0U;
}
void GimbalStartup_ManualOverride(void)
{
    taskENTER_CRITICAL(); CancelInitial(); off_seen = 0U;
    Status(GIMBAL_STARTUP_MANUAL, GIMBAL_STARTUP_NONE, 0U);
    taskEXIT_CRITICAL();
}
uint8_t GimbalStartup_Request(const RemoteFrame *frame, uint32_t now, uint8_t diagnostic)
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL(); CanBus_GetState(&bus);
    if (!diagnostic && SafeOff(frame, now) && bus.started && !bus.fault_epoch) {
        GimbalStartup_Init(); manual_rezero = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}
void GimbalStartup_GetState(GimbalStartupState *out)
{ if (out) { taskENTER_CRITICAL(); *out = state; taskEXIT_CRITICAL(); } }
uint8_t GimbalStartup_TakeEvent(GimbalStartupState *out)
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (out && pending_event) { *out = state; pending_event = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t GimbalStartup_OutputReady(void)
{ return (uint8_t)(state.phase == GIMBAL_STARTUP_MANUAL || state.ready); }

void GimbalStartup_Update(const SafetyInput *input, uint8_t diagnostic)
{
    CanBusState bus;
    YawBackendState yaw;
    PitchBackendState pitch;
    uint8_t fresh, off;
    uint32_t now;
    if (!input || state.phase == GIMBAL_STARTUP_MANUAL) { return; }
    now = input->now_ms;
    fresh = (uint8_t)(input->remote.has_frame && (!have_sequence || remote_sequence != input->remote.sequence));
    if (fresh) { remote_sequence = input->remote.sequence; have_sequence = 1U; }
    if (!have_guard) {
        guard_stop = input->stop_sequence; guard_uart = input->uart_epoch; guard_can = input->can_epoch;
        have_guard = 1U; barrier_ms = now;
    } else if (guard_stop != input->stop_sequence || guard_uart != input->uart_epoch || guard_can != input->can_epoch) {
        guard_stop = input->stop_sequence; guard_uart = input->uart_epoch; guard_can = input->can_epoch;
        CancelInitial(); barrier_ms = now; off_seen = 0U;
        Status(GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_STOP, 0U);
    }
    CanBus_GetState(&bus);
    if (!bus.started) {
        off_seen = 0U; Status(GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_CAN, 0U); return;
    }
    if (bus.fault_epoch || input->can_epoch) {
        CancelInitial(); off_seen = 0U;
        Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_CAN, 0U); return;
    }
    if (state.phase == GIMBAL_STARTUP_BLOCKED) {
        if (state.reference_latched) {
            YawBackend_GetState(&yaw); PitchBackend_GetState(&pitch);
            if (!yaw.zero_set || !pitch.zero_set || !pitch.controller.gravity_calibrated) {
                Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_REFERENCE_LOST, 0U);
            }
        }
        return;
    }
    if (diagnostic || input->control_late) {
        CancelInitial(); off_seen = 0U; barrier_ms = now;
        Status(GIMBAL_STARTUP_WAIT_OFF, diagnostic ? GIMBAL_STARTUP_DIAGNOSTIC : GIMBAL_STARTUP_STOP, 0U); return;
    }
    YawBackend_GetState(&yaw); PitchBackend_GetState(&pitch);
    off = SafeOff(&input->remote, now);
    if (state.reference_latched) {
        /* Never move the mechanical limits by silently capturing a new zero after use. */
        if (!yaw.zero_set || !pitch.zero_set || !pitch.coord_confirmed || !pitch.controller.gravity_calibrated) {
            Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_REFERENCE_LOST, 0U); return;
        }
        if (yaw.controller.faults || pitch.controller.faults) {
            Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_FAULT, 0U); return;
        }
        if (yaw.online && pitch.online && !yaw.stop_pending && !pitch.stop_pending && yaw.requested && pitch.requested) {
            if (!state.ready) {
                if (!off || !fresh || (int32_t)(input->remote.received_ms - barrier_ms) <= 0) { return; }
                Barrier(now);
            }
            Status(GIMBAL_STARTUP_READY, GIMBAL_STARTUP_NONE, 1U); return;
        }
        Status(GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_NONE, 0U);
        if (!off || !fresh || (int32_t)(input->remote.received_ms - barrier_ms) <= 0 ||
            !yaw.online || !pitch.online || yaw.active || pitch.active || yaw.enabling || pitch.enabling ||
            yaw.stop_pending || pitch.stop_pending || yaw.feedback.status != 0U || pitch.feedback.status != 0U) { return; }
        if (!yaw.requested) { (void)YawBackend_Request(1U); }
        if (!pitch.requested) { (void)PitchBackend_Request(1U); }
        return;
    }
    if (!off || (int32_t)(input->remote.received_ms - barrier_ms) <= 0) {
        if (begun) { CancelInitial(); Barrier(now); }
        off_seen = 0U;
        Status(GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_REMOTE, 0U); return;
    }
    if (!off_seen) {
        if (!fresh) { return; }
        off_seen = 1U; off_since_ms = now;
    }
    if (!begun) {
        if (!fresh || (uint32_t)(now - off_since_ms) < APP_GIMBAL_STARTUP_OFF_HOLD_MS) { return; }
        if (!YawBackend_PrepareStop()) { return; }
        (void)PitchBackend_Request(0U); (void)PitchBackend_QueryFeedback();
        begun = 1U; started_ms = query_ms = now;
        yaw_zero_done = pitch_zero_done = clear_sent = 0U;
        Status(GIMBAL_STARTUP_PREPARING, GIMBAL_STARTUP_NONE, 0U); return;
    }
    if ((uint32_t)(now - started_ms) >= APP_GIMBAL_STARTUP_TIMEOUT_MS) {
        CancelInitial(); Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_TIMEOUT, 0U); return;
    }
    if (state.phase == GIMBAL_STARTUP_BLOCKED) { return; }
    if ((uint32_t)(now - query_ms) >= 20U) { (void)PitchBackend_QueryFeedback(); query_ms = now; }
    /* Only the existing, fully guarded status13 recovery may send a single FB at boot. */
    if (yaw.online && yaw.feedback.status == 13U && !clear_sent) {
        if (YawBackend_ClearCommsFault()) { clear_sent = 1U; }
    }
    if (!manual_rezero && ((yaw.controller.faults && !((yaw.feedback.status == 13U || (clear_sent && yaw.feedback.status == 0U)) &&
        (yaw.controller.faults & ~(uint32_t)(YAW_CONTROLLER_FAULT_STATUS | YAW_CONTROLLER_FAULT_SPEED | YAW_CONTROLLER_FAULT_JUMP)) == 0U)) ||
        pitch.controller.faults || (yaw.online && yaw.feedback.status != 0U && yaw.feedback.status != 13U) ||
        (pitch.online && pitch.feedback.status != 0U))) {
        /* A status1 still in flight immediately after FD is awaited, not enabled by this path. */
        if (yaw.feedback.status != 1U && pitch.feedback.status != 1U) {
            CancelInitial(); Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_FAULT, 0U); return;
        }
    }
    if (!yaw_zero_done && yaw.online && !yaw.stop_pending && yaw.feedback.status == 0U &&
        (!yaw.controller.faults || manual_rezero || (clear_sent && (yaw.controller.faults &
        ~(uint32_t)(YAW_CONTROLLER_FAULT_STATUS | YAW_CONTROLLER_FAULT_SPEED | YAW_CONTROLLER_FAULT_JUMP)) == 0U))) {
        if (YawBackend_CaptureReference() && YawBackend_Request(1U)) { yaw_zero_done = 1U; }
    }
    if (!pitch_zero_done && pitch.online && !pitch.stop_pending && pitch.feedback.status == 0U &&
        (!pitch.controller.faults || manual_rezero)) {
        if ((!pitch.coord_confirmed && !PitchBackend_SetCoordinate(1U))) { return; }
        if (PitchBackend_CaptureReference() && PitchBackend_SetGravityOffset(0.0f, 1U) && PitchBackend_Request(1U)) {
            pitch_zero_done = 1U;
        }
    }
    YawBackend_GetState(&yaw); PitchBackend_GetState(&pitch);
    if (yaw_zero_done && pitch_zero_done && yaw.requested && pitch.requested &&
        yaw.online && pitch.online && !yaw.stop_pending && !pitch.stop_pending &&
        !yaw.controller.faults && !pitch.controller.faults && yaw.feedback.status == 0U && pitch.feedback.status == 0U) {
        YawBackend_ReleasePreparation(); state.reference_latched = 1U;
        Barrier(now); Status(GIMBAL_STARTUP_READY, GIMBAL_STARTUP_NONE, 1U);
    }
}
