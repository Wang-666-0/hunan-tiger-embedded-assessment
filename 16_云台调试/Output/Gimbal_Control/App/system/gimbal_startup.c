/**
 * @file gimbal_startup.c
 * @brief 双轴上电准备和参考建立
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（7）：
 *   GimbalStartup_Init / GimbalStartup_ManualOverride / GimbalStartup_Request /
 *   GimbalStartup_GetState / GimbalStartup_TakeEvent / GimbalStartup_OutputReady /
 *   GimbalStartup_Update
 * 内部函数（4）：
 *   Status / SafeOff / Barrier / CancelInitial
 * 文件级变量/常量（18；局部变量见对应函数）：
 *   state：本模块维护并向外发布的设备或流程状态快照。
 *   remote_sequence：启动流程最近已观察的有效遥控帧序号。
 *   guard_stop：启动流程已同步的停止、串口和 CAN 故障代次。
 *   guard_uart：启动流程已同步的停止、串口和 CAN 故障代次。
 *   guard_can：启动流程已同步的停止、串口和 CAN 故障代次。
 *   barrier_ms：启动流程的新鲜输入时间屏障。
 *   off_since_ms：本次安全 OFF 连续保持的起始时间。
 *   started_ms：当前双轴准备流程开始时间。
 *   query_ms：最近查询时间与已观察接收计数。
 *   have_sequence：是否已经建立遥控序号和故障代次观察基准。
 *   have_guard：是否已经建立遥控序号和故障代次观察基准。
 *   off_seen：本次流程是否已经观察到有效安全 OFF 输入。
 *   pending_event：准备流程是否存在新的待报告事件。
 *   begun：初始双轴准备是否已经开始。
 *   yaw_zero_done：本次准备是否已完成对应轴的参考捕获。
 *   pitch_zero_done：本次准备是否已完成对应轴的参考捕获。
 *   clear_sent：本次双轴准备是否已经尝试过允许的 Yaw 固定清错。
 *   manual_rezero：当前流程是否为人工请求的重新建立零点。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 双轴启动协调：安全 OFF、建立基准和原使能握手。 */
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
/* 功能：更新双轴准备阶段、原因和就绪标记，状态变化时产生待报告事件。 */
{
    if (state.phase != phase || state.reason != reason || state.ready != ready) {
        state.phase = phase; state.reason = reason; state.ready = ready;
        ++state.sequence; pending_event = 1U;
    }
}
void GimbalStartup_Init(void)
/* 功能：初始化双轴上电准备阶段、参考有效性及重新解锁屏障。 */
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
/* 功能：检查启动流程所需的新鲜安全 OFF 遥控条件。 */
{
    RemoteCommand command;
    if (!frame || !frame->has_frame || (uint32_t)(now - frame->received_ms) >= REMOTE_TIMEOUT_MS) { return 0U; }
    RemoteMapper_Map(frame, &command);
    return (uint8_t)(command.valid && command.failsafe_ok && command.arm == RC_LOW && command.neutral);
}
static void Barrier(uint32_t now)
/* 功能：更新时间屏障，要求后续使用更新的安全 OFF 帧重新准备。 */
{
    AppTasks_RequireRearm(); ++guard_stop;
    barrier_ms = now; off_seen = 0U;
}
static void CancelInitial(void)
/* 功能：撤销尚未完成的双轴初始准备及对应会话请求。 */
{
    if (begun && !state.reference_latched) {
        (void)YawBackend_Request(0U); (void)PitchBackend_Request(0U);
    }
    begun = yaw_zero_done = pitch_zero_done = clear_sent = 0U;
}
void GimbalStartup_ManualOverride(void)
/* 功能：把准备流程控制权交给显式人工操作，避免自动流程覆盖人工请求。 */
{
    taskENTER_CRITICAL(); CancelInitial(); off_seen = 0U;
    Status(GIMBAL_STARTUP_MANUAL, GIMBAL_STARTUP_NONE, 0U);
    taskEXIT_CRITICAL();
}
uint8_t GimbalStartup_Request(const RemoteFrame *frame, uint32_t now, uint8_t inhibited)
/* 功能：核对安全条件后提出双轴重新准备请求，等待原流程逐步完成。 */
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL(); CanBus_GetState(&bus);
    if (!inhibited && SafeOff(frame, now) && bus.started && !bus.fault_epoch) {
        GimbalStartup_Init(); manual_rezero = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}
void GimbalStartup_GetState(GimbalStartupState *out)
/* 功能：复制双轴准备阶段、原因和参考状态，供只读查询。 */
{ if (out) { taskENTER_CRITICAL(); *out = state; taskEXIT_CRITICAL(); } }
uint8_t GimbalStartup_TakeEvent(GimbalStartupState *out)
/* 功能：取出并清除一条新的双轴准备状态事件。 */
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (out && pending_event) { *out = state; pending_event = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t GimbalStartup_OutputReady(void)
/* 功能：检查双轴参考和准备阶段是否允许后续运行解锁。 */
{ return (uint8_t)(state.phase == GIMBAL_STARTUP_MANUAL || state.ready); }

void GimbalStartup_Update(const SafetyInput *input, uint8_t inhibited)
/* 功能：按原安全门禁推进双轴停止确认、参考捕获和准备就绪状态。 */
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
    if (inhibited || input->control_late) {
        CancelInitial(); off_seen = 0U; barrier_ms = now;
        Status(GIMBAL_STARTUP_WAIT_OFF, inhibited ? GIMBAL_STARTUP_DIAGNOSTIC : GIMBAL_STARTUP_STOP, 0U); return;
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
