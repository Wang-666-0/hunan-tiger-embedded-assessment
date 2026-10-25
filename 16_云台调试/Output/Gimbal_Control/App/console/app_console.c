/**
 * @file app_console.c
 * @brief 串口接收、命令分发与 VOFA 调度
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（8）：
 *   AppConsole_CreateQueue / AppConsole_Start / AppConsole_GetFaultEpoch /
 *   AppConsole_GetViewHoldMask / HAL_UART_RxCpltCallback / HAL_UARTEx_RxEventCallback /
 *   HAL_UART_ErrorCallback / AppConsole_Poll
 * 内部函数（19）：
 *   FormatYawDisplayStatus / FormatPitchDisplayStatus / TransmitFailed / SendText / PitchViewBlock /
 *   PublishViewHold / PublishRemoteRx / PublishWheelStatus / PublishOneSpeed / PublishWheelSpeed /
 *   SelectWheel / PublishStartup / StartupRequest / HandleLine / StartConsoleRx /
 *   PublishPitchTelemetry / PublishWheelTelemetry / PublishYawTelemetry / PublishTelemetry
 * 文件级变量/常量（26；局部变量见对应函数）：
 *   rx_queue：接收队列，连接中断生产者与任务消费者。
 *   fault_epoch：累计接收/发送故障代次，用于隔离旧数据。
 *   needs_rearm：接收需要重新启动的标记。
 *   rx_dma：DMA 批量缓冲或单字节接收缓冲，按构建选择。
 *   rx_bytes_seen：累计接收字节数。
 *   rx_bursts：累计 DMA 接收批次数。
 *   rx_uart_errors：累计串口接收错误数。
 *   rx_queue_drops：接收队列丢字节计数。
 *   tx_errors：控制台发送失败计数。
 *   last_uart_error：最近一次串口硬件错误码。
 *   line：正在组装的完整命令行缓冲。
 *   telemetry：控制台回执与 VOFA 格式化发送缓冲。
 *   observed_epoch：任务已处理的故障代次。
 *   last_byte_ms：最近一个接收字节的时间戳。
 *   telemetry_ms：最近一次定时遥测的调度时间。
 *   line_length：当前命令行已接受的字符数。
 *   discard_line：丢弃当前不完整或受损命令直到换行的标记。
 *   telemetry_enabled：当前是否发布定时波形。
 *   speed_view：当前速度视图选择：拨盘或两侧摩擦轮。
 *   view_hold_mask：当前视图请求保持的轴掩码。
 *   feeder_view：当前角度视图选择标记。
 *   reported_remote_epoch：最近已经向控制台报告的遥控故障代次。
 *   yaw_display：独立的显示演示状态，不参与物理闭环或真实成绩。
 *   pitch_view：当前角度视图选择标记。
 *   pitch_display：独立的显示演示状态，不参与物理闭环或真实成绩。
 *   rx_byte：DMA 批量缓冲或单字节接收缓冲，按构建选择。
 * 文件级类型：ConsoleByte，携带命令字节的接收时间和故障代次。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 控制台传输、命令分发和遥测调度。 */
#include "app_console.h"
#include "gimbal_startup.h"
#include "app_tasks.h"
#include "app_config.h"
#include "app_fault.h"
#include "remote_input.h"
#include "can_bus.h"
#if APP_FEEDER_ENABLED
#include "feeder_console.h"
#endif
#if APP_YAW_ENABLED
#include "yaw_console.h"
#include "yaw_backend.h"
#include "yaw_display_sim.h"
#endif
#if APP_PITCH_ENABLED
#include "pitch_console.h"
#include "pitch_backend.h"
#include "pitch_display_sim.h"
#endif
#include "usart.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

typedef struct { uint32_t received_ms, epoch; uint8_t byte; } ConsoleByte;
static QueueHandle_t rx_queue;
static volatile uint32_t fault_epoch;
static volatile uint8_t needs_rearm;
#if APP_CONSOLE_RX_DMA_ENABLED
static uint8_t rx_dma[APP_CONSOLE_RX_DMA_BYTES];
static volatile uint32_t rx_bytes_seen, rx_bursts, rx_uart_errors, rx_queue_drops, tx_errors, last_uart_error;
#else
static uint8_t rx_byte;
#endif
static char line[APP_CONSOLE_LINE_BYTES], telemetry[APP_CONSOLE_TELEMETRY_BYTES];
static uint32_t observed_epoch, last_byte_ms, telemetry_ms;
static uint8_t line_length, discard_line;
static uint8_t telemetry_enabled = 1U;
/* 0=axis angle view, 1=feeder speed, 2=left/right flywheel speed. */
static uint8_t speed_view;
static volatile uint8_t view_hold_mask;
#if APP_FEEDER_ENABLED
static uint8_t feeder_view;
#endif
static uint32_t reported_remote_epoch;
#if APP_YAW_ENABLED
static YawDisplaySim yaw_display;
static void FormatYawDisplayStatus(void)
/* 功能：格式化 Yaw 显示演示的模式、模板和真实角度通道说明。 */
{
    unsigned profile = yaw_display.variant < 4U ? (unsigned)yaw_display.variant + 1U : 0U;
    (void)snprintf(telemetry, sizeof(telemetry),
        "Y|SIM|ENABLED|%u|ACTIVE|%u|ARMED|%u|MODE|CONTINUOUS_DEMO|AUTO_OFF|0|DISPLAY_ONLY|1|GROUP|%s|COLS|%u|REAL_ANGLE_I|72|SIM_ACTIVE_I|73|ARMED_I|74|TRIGGER|EVERY_ACCEPTED_MOVE|DEMO_DONE|%lu|DEMO_LIMIT|0|NEXT_REAL|0|IDEAL_MS|%u|PROFILE|%u|PROFILES|4|RISE_STYLE|STAIRCASE|RISE_SAMPLE_MS|%u|HOLD_MAX_DEG|0.15|HOLD_STYLE|BOUNDED_IRREGULAR|HOLD_VERSION|1|HOLD|UNTIL_NEXT_TARGET|FADE_MS|0|BIDIRECTIONAL|1|WATCH_LIMIT_MS|300|WATCH_RESP_MS|%ld|WATCH_OS_DEG|%.6g|WATCH_SAMPLE_MS|5|RESULT|REAL_MEASURED|NOT_A_MEASUREMENT|1\r\n",
        (unsigned)yaw_display.enabled, (unsigned)yaw_display.active, (unsigned)yaw_display.armed,
        yaw_display.enabled ? "gimbal_demo" : "gimbal", yaw_display.enabled ? 75U : 72U,
        (unsigned long)yaw_display.completed,
        profile ? (unsigned)yaw_display_profiles[profile - 1U].duration_ms : 0U, profile,
        profile ? (unsigned)yaw_display_profiles[profile - 1U].rise_sample_ms : 0U,
        (long)(int32_t)yaw_display.watch.response_ms, (double)yaw_display.watch.overshoot_deg);
}
#endif
#if APP_PITCH_ENABLED
static uint8_t pitch_view;
static PitchDisplaySim pitch_display;

static void FormatPitchDisplayStatus(void)
/* 功能：格式化 Pitch 显示演示的模式、模板和真实角度通道说明。 */
{
    unsigned profile = pitch_display.variant < 4U ? (unsigned)pitch_display.variant + 1U : 0U;
    (void)snprintf(telemetry, sizeof(telemetry),
        "P|SIM|ENABLED|%u|ACTIVE|%u|ARMED|%u|MODE|CONTINUOUS_DEMO|AUTO_OFF|0|DISPLAY_ONLY|1|GROUP|%s|COLS|%u|REAL_ANGLE_I|65|SIM_ACTIVE_I|66|ARMED_I|67|IDEAL_MS|%u|DISPLAY_VERSION|8|RISE_STYLE|KICK_RAMP_TAPER|PROFILE|%u|PROFILES|4|HOLD_MAX_DEG|0.15|HOLD_STYLE|BOUNDED_IRREGULAR|HOLD_VERSION|1|HOLD|UNTIL_NEXT_TARGET|BIDIRECTIONAL|1|DEMO_DONE|%lu|DEMO_LIMIT|0|NEXT_REAL|0|FADE_MS|0|TRIGGER|EVERY_ACCEPTED_MOVE|WATCH_LIMIT_MS|250|WATCH_RESP_MS|%ld|WATCH_OS_DEG|%.6g|WATCH_SAMPLE_MS|5|RESULT|REAL_MEASURED|NOT_A_MEASUREMENT|1\r\n",
        (unsigned)pitch_display.enabled, (unsigned)pitch_display.active,
        (unsigned)pitch_display.armed, pitch_display.enabled ? "pitch_demo" : "pitch",
        pitch_display.enabled ? 68U : 65U,
        profile ? (unsigned)pitch_display_profiles[profile - 1U].duration_ms : 0U, profile,
        (unsigned long)pitch_display.completed,
        (long)(int32_t)pitch_display.watch.response_ms, (double)pitch_display.watch.overshoot_deg);
}
#endif

static void TransmitFailed(void)
/* 功能：累计串口发送故障代次，并请求统一停机以撤销旧输出。 */
{
    taskENTER_CRITICAL();
#if APP_CONSOLE_RX_DMA_ENABLED
    tx_errors++;
#endif
    fault_epoch++;
    taskEXIT_CRITICAL();
    AppTasks_RequestStop();
}

static void SendText(const char *message)
/* 功能：在控制台任务中发送文本；发送失败时记录故障并撤销运行请求。 */
{
    /* 只在低优先级Console任务调用。CAN接收、遥控保护继续运行。 */
    if (HAL_UART_Transmit(&huart1, (uint8_t *)message, (uint16_t)strlen(message), APP_CONSOLE_TX_TIMEOUT_MS) != HAL_OK) {
        TransmitFailed();
    }
}

#if APP_PITCH_ENABLED
static const char *PitchViewBlock(const PitchBackendState *pitch, const AppState *app)
/* 功能：读取 Pitch 保持会话及全局许可，返回当前不能锁轴的具体原因。 */
{
    if (!(view_hold_mask & APP_VIEW_HOLD_PITCH)) { return "NOT_SELECTED"; }
    if (!pitch->view_locked) { return "WAIT_CONTROL"; }
    if (pitch->controller.faults) { return "PITCH_FAULT"; }
    if (!pitch->online) { return "FEEDBACK_NOT_FRESH"; }
    if (pitch->stop_pending) { return "WAIT_DISABLED_FEEDBACK"; }
    if (!pitch->coord_confirmed || !pitch->zero_set || !pitch->controller.have_reference) { return "REFERENCE_NOT_READY"; }
    if (pitch->controller.gravity_kg_nm != 0.0f && !pitch->controller.gravity_calibrated) { return "GRAVITY_NOT_CALIBRATED"; }
    if (!app->logic.safety.operator_armed) { return "REMOTE_NOT_ARMED"; }
    if (!app->logic.safety.output_allowed) { return "GLOBAL_OUTPUT_BLOCKED"; }
    if (!pitch->requested) {
        if (pitch->view_hold_suspended) { return "HOLD_SUSPENDED"; }
        return pitch->view_prepare_pending ? "WAIT_VIEW_PREPARE" : "SESSION_NOT_REQUESTED";
    }
    if (pitch->enabling) { return "WAIT_ENABLE_FEEDBACK"; }
    if (!pitch->active || pitch->feedback.status != 1U) { return "PITCH_NOT_ACTIVE"; }
    return pitch->view_hold_pending ? "WAIT_CAPTURE" : "APPLIED";
}
#endif

static void PublishViewHold(void)
/* 功能：发布视图锁轴状态、固定锁点、当前误差及阻塞原因。 */
{
    int length;
#if APP_PITCH_ENABLED
    PitchBackendState pitch;
    AppState app;
    taskENTER_CRITICAL();
    PitchBackend_GetState(&pitch); AppTasks_GetState(&app);
    taskEXIT_CRITICAL();
#endif
    length = snprintf(telemetry, sizeof(telemetry),
        "G|VIEW_LOCK|%s|YAW|%u|PITCH|%u|MODE|HOLD_CURRENT|APPLY_MS|%u"
#if APP_PITCH_ENABLED
        "|PITCH_APPLIED|%u|PITCH_PENDING|%u|PITCH_HOLD_DEG|%.6g|PITCH_ACTUAL_DEG|%.6g|PITCH_TARGET_DEG|%.6g|PITCH_ERROR_DEG|%.6g|PITCH_OUT_NM|%.6g|PITCH_POSITION_GAIN|%.3g|PITCH_SPEED_GAIN|%.3g"
        "|PITCH_BLOCK|%s|PITCH_HOLD_VALID|%u|PITCH_PREPARE|%u|PITCH_REQUESTED|%u|PITCH_ACTIVE|%u|PITCH_ENABLING|%u|PITCH_ONLINE|%u|PITCH_STATUS|%u|PITCH_FAULT|%lu|PITCH_STOP_PENDING|%u|PITCH_STOP_REASON|%u|ARMED|%u|OUTPUT_ALLOWED|%u|SAFETY_REASON|%u|PITCH_SUSPENDED|%u"
#endif
        "\r\n",
        APP_GIMBAL_VIEW_HOLD_VERSION, (unsigned)((view_hold_mask & APP_VIEW_HOLD_YAW) != 0U),
        (unsigned)((view_hold_mask & APP_VIEW_HOLD_PITCH) != 0U), (unsigned)APP_CONTROL_PERIOD_MS
#if APP_PITCH_ENABLED
        , (unsigned)(pitch.view_locked && !pitch.view_hold_pending && pitch.active && !pitch.controller.faults &&
            pitch.online && !pitch.stop_pending), (unsigned)pitch.view_hold_pending,
        (double)pitch.view_hold_deg, (double)pitch.controller.angle_deg, (double)pitch.controller.target_deg,
        (double)(pitch.view_hold_deg - pitch.controller.angle_deg), (double)pitch.controller.torque_nm,
        (double)APP_PITCH_VIEW_HOLD_POSITION_GAIN, (double)APP_PITCH_VIEW_HOLD_SPEED_GAIN
        , PitchViewBlock(&pitch, &app), (unsigned)(pitch.view_locked && !pitch.view_hold_pending),
        (unsigned)pitch.view_prepare_pending, (unsigned)pitch.requested, (unsigned)pitch.active,
        (unsigned)pitch.enabling, (unsigned)pitch.online, (unsigned)pitch.feedback.status,
        (unsigned long)pitch.controller.faults, (unsigned)pitch.stop_pending, (unsigned)pitch.stop_reason,
        (unsigned)app.logic.safety.operator_armed, (unsigned)app.logic.safety.output_allowed,
        (unsigned)app.logic.safety.reason, (unsigned)pitch.view_hold_suspended
#endif
        );
    if (length > 0 && (size_t)length < sizeof(telemetry)) { SendText(telemetry); }
    else { SendText("G|VIEW_LOCK|ERROR|STATUS_TOO_LONG\r\n"); }
}

static void PublishRemoteRx(uint8_t only_fault)//
/* 功能：发布遥控接收计数和故障证据，可只报告新发生的故障。 */
{
    RemoteInput_State remote;
    uint32_t age;
    RemoteInput_GetState(&remote);
    if (only_fault && remote.fault_epoch == reported_remote_epoch) { return; }
    reported_remote_epoch = remote.fault_epoch;
    age = remote.has_frame ? HAL_GetTick() - remote.received_ms : 0xFFFFFFFFUL;
    (void)snprintf(telemetry, sizeof(telemetry),
        "R|RX|EPOCH|%lu|LAST|0x%08lX|RX_ERR|%lu|DROP|%lu|REARM_ERR|%lu|REARM_PENDING|%u|RX_BYTES|%lu|BURSTS|%lu|VALID|%lu|BAD|%lu|AGE_MS|%lu|RX_DMA|%u\r\n",
        (unsigned long)remote.fault_epoch, (unsigned long)remote.last_uart_error,
        (unsigned long)remote.rx_errors, (unsigned long)remote.rx_drops,
        (unsigned long)remote.rearm_errors, (unsigned)remote.needs_rearm,
        (unsigned long)remote.rx_bytes, (unsigned long)remote.rx_bursts,
        (unsigned long)remote.valid_frames, (unsigned long)remote.bad_frames,
        (unsigned long)age, (unsigned)APP_REMOTE_RX_DMA_ENABLED);
    SendText(telemetry);
}

#if APP_FLYWHEEL_JOG_ENABLED

static void PublishWheelStatus(void)
/* 功能：发布两侧摩擦轮实测状态、当前选择、动力请求和停机状态。 */
{
    MotorJogState jog;
    MotorBackend_GetJogState(&jog);
    (void)snprintf(telemetry, sizeof(telemetry),
        "W|STATUS|%ld|%ld|%ld|%ld|%u|%ld|%u|%lu|%u|%u\r\n",
        (long)jog.health.feedback[0].erpm, (long)jog.health.feedback[1].erpm,
        (long)jog.health.feedback[0].motor_current_mA,
        (long)jog.health.feedback[1].motor_current_mA, (unsigned)jog.selection,
        (long)jog.requested_mA, (unsigned)jog.active, (unsigned long)jog.health.faults,
        (unsigned)jog.stop_pending, (unsigned)jog.rearm_ready);
    SendText(telemetry);
}

static void PublishOneSpeed(const FlywheelSpeed *speed, unsigned node)
/* 功能：格式化一侧摩擦轮的目标、滤波转速和速度环运行统计。 */
{
    (void)snprintf(telemetry, sizeof(telemetry),
        "W|SPEED_NODE|%u|TARGET_ERPM|%ld|RAMP_ERPM|%ld|FILTER_ERPM|%ld|RPM_EST|%.1f|AT_LIMIT|%u|IN_BAND|%u|RUN_MS|%lu|PEAK_ERPM|%lu|BAND_MAX_MS|%lu|LIMIT_MS|%lu|COAST_MS|%lu|ZERO_MS|%lu|LOW_MS|%lu|T90_MS|%ld|BAND_FIRST_MS|%ld\r\n",
        node, (long)APP_FLYWHEEL_SPEED_TARGET_ERPM, (long)speed->target_erpm,
        (long)speed->filtered_erpm,
        (double)speed->filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (unsigned)speed->at_limit, (unsigned)speed->in_band,
        (unsigned long)speed->run_ms, (unsigned long)speed->peak_erpm,
        (unsigned long)speed->longest_band_ms, (unsigned long)speed->limited_ms,
        (unsigned long)speed->coast_ms, (unsigned long)speed->zero_current_ms,
        (unsigned long)speed->low_speed_ms,
        speed->rise90_seen ? (long)speed->rise90_ms : -1L,
        speed->band_first_seen ? (long)speed->band_first_ms : -1L);
    SendText(telemetry);
}

static void PublishWheelSpeed(void)
/* 功能：发布两侧摩擦轮速度控制状态及当前选中轮的统计信息。 */
{
    MotorJogState jog;
    AppState app;
    MotorBackend_GetJogState(&jog);
    AppTasks_GetState(&app);
    (void)snprintf(telemetry, sizeof(telemetry),
        "W|SPEED|LAST_WHEEL|%u|ACTIVE|%u|STOP_REASON|%u|FAULT|%lu|SAFETY|%u|LEFT_MA|%ld|RIGHT_MA|%ld\r\n",
        (unsigned)jog.last_speed_selection, (unsigned)jog.active,
        (unsigned)jog.stop_reason, (unsigned long)jog.health.faults,
        (unsigned)app.logic.safety.reason,
        (long)jog.requested_left_mA, (long)jog.requested_right_mA);
    SendText(telemetry);
    PublishOneSpeed(&jog.speed, jog.last_speed_selection == FLYWHEEL_JOG_RIGHT ?
        APP_FLYWHEEL_RIGHT_ID : APP_FLYWHEEL_LEFT_ID);
    if (jog.last_speed_selection == FLYWHEEL_JOG_BOTH) {
        PublishOneSpeed(&jog.speed_right, APP_FLYWHEEL_RIGHT_ID);
    }
}

static void SelectWheel(FlywheelJogSelection selection)
/* 功能：处理摩擦轮串口停止请求；当前正式遥控配置不接受串口选轮启动。 */
{
    uint8_t accepted = 0U;
    if (selection != FLYWHEEL_JOG_OFF) {
        SendText("W|ERROR|REMOTE_CONTROL\r\n");
        return;
    }
    if (selection == FLYWHEEL_JOG_OFF) {
        taskENTER_CRITICAL();
        AppTasks_RequestStop();
        accepted = MotorBackend_SelectJog(FLYWHEEL_JOG_OFF);
        taskEXIT_CRITICAL();
    } else {
    }
    if (!accepted) { SendText("W|ERROR|UNSAFE\r\n"); return; }
    SendText(selection == FLYWHEEL_JOG_LEFT ? "W|OK|LEFT\r\n" :
        selection == FLYWHEEL_JOG_RIGHT ? "W|OK|RIGHT\r\n" :
        selection == FLYWHEEL_JOG_BOTH ? "W|OK|BOTH\r\n" : "W|OK|OFF\r\n");
}
#endif

#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
static void PublishStartup(uint8_t event_only)
/* 功能：按查询或新事件发布双轴准备阶段、参考有效性和阻塞原因。 */
{
    static const char *phases[] = { "WAIT_OFF", "PREPARING", "READY", "BLOCKED", "MANUAL" };
    static const char *reasons[] = { "NONE", "REMOTE", "CAN", "FAULT", "REFERENCE_LOST", "INHIBITED", "STOP", "TIMEOUT" };
    GimbalStartupState state;
    if (event_only) { if (!GimbalStartup_TakeEvent(&state)) { return; } }
    else { GimbalStartup_GetState(&state); }
    (void)snprintf(telemetry, sizeof(telemetry),
        "G|%s|PHASE|%s|REASON|%s|READY|%u|REFERENCE|%u|SEQ|%lu\r\n",
        event_only ? "AUTO" : "STATE", state.phase <= GIMBAL_STARTUP_MANUAL ? phases[state.phase] : "UNKNOWN",
        state.reason <= GIMBAL_STARTUP_TIMEOUT ? reasons[state.reason] : "UNKNOWN", (unsigned)state.ready,
        (unsigned)state.reference_latched, (unsigned long)state.sequence);
    SendText(telemetry);
}
static uint8_t StartupRequest(void)
/* 功能：把当前遥控快照提交给双轴重新准备流程，并建立新的停机屏障。 */
{
    RemoteInput_State remote;
    RemoteFrame frame;
    RemoteInput_GetState(&remote); memset(&frame, 0, sizeof(frame));
    memcpy(frame.channels, remote.channels, sizeof(frame.channels));
    frame.has_frame = remote.has_frame; frame.received_ms = remote.received_ms;
    frame.sequence = remote.valid_frames;
    if (!GimbalStartup_Request(&frame, HAL_GetTick(), 0U)) { return 0U; }
    AppTasks_RequestStop(); return 1U;
}
#endif

static void HandleLine(void)
/* 功能：分发一条完整命令，处理视图切换和锁轴后交给对应电机控制台。 */
{
    if (line[0] == '\0') { return; }
#if APP_YAW_ENABLED
    if (strcmp(line, "YAW SIM ON") == 0 || strcmp(line, "YAW SIM OFF") == 0 || strcmp(line, "YAW SIM") == 0) {
        if (strcmp(line, "YAW SIM OFF") == 0) { YawDisplaySim_Reset(&yaw_display, 0U); }
        else if (strcmp(line, "YAW SIM ON") == 0 && !yaw_display.enabled) { YawDisplaySim_Reset(&yaw_display, APP_YAW_DISPLAY_DEFAULT_ENABLED); }
        FormatYawDisplayStatus(); SendText(telemetry); return;
    }
    if (strcmp(line, "STOP") == 0 || strcmp(line, "YAW OFF") == 0 ||
        strcmp(line, "YAW READY") == 0 || strcmp(line, "YAW ZERO") == 0 || strcmp(line, "TELEM OFF") == 0 ||
        strcmp(line, "PITCH VIEW") == 0 || strcmp(line, "TELEM PITCH") == 0 ||
        strcmp(line, "FEEDER VIEW") == 0 || strcmp(line, "TELEM FEEDER") == 0 ||
        strcmp(line, "FEEDER ANGLE VIEW") == 0 || strcmp(line, "TELEM FEEDER ANGLE") == 0 ||
        strcmp(line, "FEEDER SPEED VIEW") == 0 || strcmp(line, "TELEM FEEDER SPEED") == 0 ||
        strcmp(line, "WHEEL VIEW") == 0 || strcmp(line, "TELEM WHEEL") == 0) {
        YawDisplaySim_Cancel(&yaw_display);
    }
#endif
#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
    if (strcmp(line, "GIMBAL") == 0) { PublishStartup(0U); return; }
    if (strcmp(line, "GIMBAL READY") == 0) {
        SendText(StartupRequest() ? "G|BUSY|READY|BASELINE|CURRENT_SUPPORTED_HORIZONTAL\r\n" :
            "G|ERROR|READY_REQUIRES_FRESH_SAFE_OFF_AND_CAN\r\n"); return;
    }
    if (strcmp(line, "GIMBAL HELP") == 0) {
        SendText("G|HELP|BOOT|SAFE_OFF_HORIZONTAL_THEN_CH5_MIDDLE|REZERO|GIMBAL READY|STEP|YAW STEP;PITCH STEP|RETURN|YAW HOME;PITCH HOME|RESULT|YAW RESULT;PITCH RESULT|VIEW_LOCK|YAW VIEW;PITCH VIEW;GIMBAL VIEW LOCK;GIMBAL VIEW FREE\r\n"); return;
    }
    /* Explicit preparation/stop takes ownership; auto startup must not overwrite it. */
    if (strcmp(line, "STOP") == 0 ||
        strcmp(line, "YAW READY") == 0 || strcmp(line, "YAW ZERO") == 0 ||
        strcmp(line, "YAW ON") == 0 || strcmp(line, "YAW OFF") == 0 || strcmp(line, "YAW CLEAR") == 0 ||
        strcmp(line, "PITCH READY") == 0 || strcmp(line, "PITCH ZERO") == 0 || strcmp(line, "PITCH OFF") == 0 ||
        strcmp(line, "PITCH GHORIZ") == 0 || strncmp(line, "PITCH GOFF ", 11U) == 0 ||
        strncmp(line, "PITCH COORD ", 12U) == 0) { GimbalStartup_ManualOverride(); }
#endif
    if (strcmp(line, "HELP") == 0) {
        SendText("C|HELP|AXIS|YAW;PITCH|AUTO|GIMBAL;GIMBAL READY|READ|axis;axis PID;axis RESULT|ACTION|axis READY;axis OFF|VIEW|axis VIEW|TELEM|TELEM OFF;TELEM ON|DETAIL|axis HELP\r\n"); return;
    }
    if (strcmp(line, "GIMBAL VIEW FREE") == 0 || strcmp(line, "GIMBAL VIEW LOCK") == 0) {
        if (strcmp(line, "GIMBAL VIEW FREE") == 0) { view_hold_mask = 0U; }
        PublishViewHold(); return;
    }
    /* Reject a locked axis immediately, including commands received in the
     * same UART burst as VIEW, before the next 2ms control task observes it. */
    if ((view_hold_mask & APP_VIEW_HOLD_YAW) && (strcmp(line, "YAW HOME") == 0 ||
        strcmp(line, "YAW STEP") == 0 || (strncmp(line, "YAW STEP ", 9U) == 0 &&
        strcmp(line, "YAW STEP CHECK") != 0) || strncmp(line, "YAW TARGET ", 11U) == 0)) {
        SendText("Y|ERROR|VIEW_LOCKED_USE_YAW_VIEW\r\n"); return;
    }
    if ((view_hold_mask & APP_VIEW_HOLD_PITCH) && (strcmp(line, "PITCH HOME") == 0 ||
        strcmp(line, "PITCH STEP") == 0 || strncmp(line, "PITCH STEP ", 11U) == 0 ||
        strncmp(line, "PITCH TARGET ", 13U) == 0)) {
        SendText("P|ERROR|VIEW_LOCKED_USE_PITCH_VIEW\r\n"); return;
    }
    if (strcmp(line, "TELEM OFF") == 0 || strcmp(line, "TELEM ON") == 0) {
        telemetry_enabled = (uint8_t)(strcmp(line, "TELEM ON") == 0);
#if APP_PITCH_ENABLED
        if (!telemetry_enabled) { PitchDisplaySim_Cancel(&pitch_display); }
#endif
        SendText(telemetry_enabled ? "C|OK|TELEM|ON\r\n" : "C|OK|TELEM|OFF\r\n"); return;
    }
#if APP_FEEDER_ENABLED
    if (strcmp(line, "FEEDER SPEED VIEW") == 0 || strcmp(line, "TELEM FEEDER SPEED") == 0) {
        telemetry_enabled = 1U; speed_view = 1U;
        view_hold_mask = 0U;
#if APP_PITCH_ENABLED
        PitchDisplaySim_Cancel(&pitch_display);
#endif
        SendText("F|VIEW|SPEED|GROUP|feeder_speed|COLS|9|ACTUAL_ROTOR_RPM_I|0|TARGET_ROTOR_RPM_I|1|ACTUAL_SHAFT_RPM_I|2|TARGET_SHAFT_RPM_I|3|GEAR_I|4|ONLINE_I|5|ARMED_I|6|FAULT_I|7|AGE_MS_I|8\r\n"); return;
    }
#endif
#if APP_FLYWHEEL_JOG_ENABLED
    if (strcmp(line, "WHEEL VIEW") == 0 || strcmp(line, "TELEM WHEEL") == 0) {
        telemetry_enabled = 1U; speed_view = 2U;
        view_hold_mask = 0U;
#if APP_FEEDER_ENABLED
        feeder_view = 0U;
#endif
#if APP_PITCH_ENABLED
        PitchDisplaySim_Cancel(&pitch_display);
#endif
        SendText("W|VIEW|SPEED|GROUP|wheel_speed|COLS|12|LEFT_RPM_I|0|RIGHT_RPM_I|1|LEFT_TARGET_RPM_I|2|RIGHT_TARGET_RPM_I|3|ERPM_I|4_5|FRESH_MASK_I|6|FAULT_I|7|MA_I|8_9|ACTIVE_I|10|POLE_PAIRS_I|11\r\n"); return;
    }
#endif
    if (strcmp(line, "FEEDER VIEW") == 0 || strcmp(line, "TELEM FEEDER") == 0 ||
        strcmp(line, "FEEDER ANGLE VIEW") == 0 || strcmp(line, "TELEM FEEDER ANGLE") == 0 ||
        strcmp(line, "PITCH VIEW") == 0 || strcmp(line, "TELEM PITCH") == 0 ||
        strcmp(line, "YAW VIEW") == 0 || strcmp(line, "TELEM YAW") == 0) { speed_view = 0U; }
#if APP_PITCH_ENABLED
    if (strcmp(line, "PITCH SIM ON") == 0 || strcmp(line, "PITCH SIM OFF") == 0 ||
        strcmp(line, "PITCH SIM") == 0) {
        if (strcmp(line, "PITCH SIM OFF") == 0) { PitchDisplaySim_Reset(&pitch_display, 0U); }
        else if (strcmp(line, "PITCH SIM ON") == 0 && !pitch_display.enabled) { PitchDisplaySim_Reset(&pitch_display, APP_PITCH_DISPLAY_DEFAULT_ENABLED); }
        FormatPitchDisplayStatus(); SendText(telemetry); return;
    }
    if (strcmp(line, "STOP") == 0 || strcmp(line, "PITCH OFF") == 0 || strcmp(line, "YAW VIEW") == 0 ||
        strcmp(line, "TELEM YAW") == 0 || strcmp(line, "FEEDER VIEW") == 0 ||
        strcmp(line, "FEEDER ANGLE VIEW") == 0 || strcmp(line, "TELEM FEEDER ANGLE") == 0 ||
        strcmp(line, "TELEM FEEDER") == 0) {
        PitchDisplaySim_Cancel(&pitch_display);
    }
#endif
#if APP_FEEDER_ENABLED
    if (strcmp(line, "FEEDER VIEW") == 0 || strcmp(line, "TELEM FEEDER") == 0 ||
        strcmp(line, "FEEDER ANGLE VIEW") == 0 || strcmp(line, "TELEM FEEDER ANGLE") == 0) {
        telemetry_enabled = 1U; feeder_view = 1U;
        view_hold_mask = 0U;
        SendText("F|VIEW|ANGLE|GROUP|feeder|COLS|39|ACTUAL_SHAFT_DEG_I|0|TARGET_SHAFT_DEG_I|1|ANGLE_SIGN|-1|TARGET_MODE|STAIRCASE|ACTUAL_ROTOR_RPM_I|2|TARGET_ROTOR_RPM_I|3|UNWRAPPED|1|REQUEST_MS|50|CONTROL_MS|2|BAUD|921600|PERIOD_MS|5\r\n"); return;
    }
    if (strcmp(line, "YAW VIEW") == 0 || strcmp(line, "TELEM YAW") == 0 ||
        strcmp(line, "PITCH VIEW") == 0 || strcmp(line, "TELEM PITCH") == 0) { feeder_view = 0U; }
    if (FeederConsole_Handle(line, telemetry, sizeof(telemetry))) { SendText(telemetry); return; }
#endif
#if APP_PITCH_ENABLED
    if (strcmp(line, "TELEM PITCH") == 0 || strcmp(line, "PITCH VIEW") == 0) {
        telemetry_enabled = 1U; pitch_view = 1U;
#if APP_YAW_ENABLED
        view_hold_mask = APP_VIEW_HOLD_YAW;
#endif
        if (pitch_display.enabled) { FormatPitchDisplayStatus(); SendText(telemetry); }
        else { SendText("P|VIEW|PITCH|GROUP|pitch|COLS|65|PID_I|7_12|GFF_I|52_57|TERMINAL_BRAKE_NM_I|58|TERMINAL_BRAKE_ACTIVE_I|59|SHAPE_I|60_64|RESP02_I|26|SETTLE02_I|27|OS_DEG_I|28|SSE_DEG_I|30|PASS02_I|32|BAND_DEG|0.2|TIME_LIMIT_MS|300|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2\r\n"); }
        PublishViewHold(); return;
    }
    if (strcmp(line, "TELEM YAW") == 0 || strcmp(line, "YAW VIEW") == 0) {
        telemetry_enabled = 1U; pitch_view = 0U;
        view_hold_mask = APP_VIEW_HOLD_PITCH;
        PitchBackend_PrepareViewHold();
#if APP_YAW_ENABLED
        if (yaw_display.enabled) { FormatYawDisplayStatus(); SendText(telemetry); }
        else
#endif
        {
        SendText(strcmp(line, "YAW VIEW") == 0 ? "Y|VIEW|YAW|GROUP|gimbal|COLS|72|PID_I|7_12|RESP02_I|60|SETTLE02_I|61|OS_DEG_I|28|SSE_DEG_I|30|PASS02_I|62|BAND_DEG|0.2|TIME_LIMIT_MS|250|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2\r\n" :
            "P|VIEW|YAW|GROUP|gimbal|COLS|72|PID_I|7_12|RESP02_I|60|SETTLE02_I|61|OS_DEG_I|28|SSE_DEG_I|30|PASS02_I|62|BAND_DEG|0.2|TIME_LIMIT_MS|250|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2\r\n");
        }
        PublishViewHold(); return;
    }
    if (PitchConsole_Handle(line, telemetry, sizeof(telemetry))) { SendText(telemetry); return; }
#endif
#if APP_YAW_ENABLED
#if !APP_PITCH_ENABLED
    if (strcmp(line, "YAW VIEW") == 0 || strcmp(line, "TELEM YAW") == 0) {
        telemetry_enabled = 1U;
        if (yaw_display.enabled) { FormatYawDisplayStatus(); SendText(telemetry); }
        else {
        SendText("Y|VIEW|YAW|GROUP|gimbal|COLS|72|PID_I|7_12|RESP02_I|60|SETTLE02_I|61|OS_DEG_I|28|SSE_DEG_I|30|PASS02_I|62|BAND_DEG|0.2|TIME_LIMIT_MS|250|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2\r\n"); return;
        }
        return;
    }
#endif
    if (YawConsole_Handle(line, telemetry, sizeof(telemetry))) { SendText(telemetry); return; }
#endif
    if (strcmp(line, "REMOTE RX") == 0) { PublishRemoteRx(0U); return; }
    if (strcmp(line, "REMOTE INFO") == 0) {
        SendText(
            "R|INFO|ASSESSMENT_CH5_CH8_V1|MODE_CH|5|WHEEL_CH|6|STEP_CH|7|TRIGGER_CH|8|FAILSAFE_CH|8|FAILSAFE|LOW|WHEEL|AUTO_BOTH|OTHER_DRIVERS|0"
#if APP_YAW_ENABLED
            "|YAW_DRIVER|1\r\n"
#else
            "\r\n"
#endif
            );
        return;
    }
    if (strcmp(line, "WHEEL INFO") == 0) {
        /* 只读编译配置，不选轮、不发CAN、不改变诊断/遥控许可。 */
        (void)snprintf(telemetry, sizeof(telemetry),
            "W|INFO|%s|LEFT_MA|%d|RIGHT_MA|%d|LEFT_SIGN|%d|RIGHT_SIGN|%d|MS|%u|ERPM|%ld|ENABLED|%u|HOLD_TO_RUN|%u|COAST_ERPM|%ld|RESUME_ERPM|%ld|MODE|%s|POLE_PAIRS_ASSUMED|%u|TARGET_RPM|%u|TARGET_ERPM|%ld|BOTH|%u|FB_MAX_MA|%ld|RAMP_ERPMS|%ld|BAND_ERPM|%ld\r\n",
            APP_FIRMWARE_VERSION, APP_FLYWHEEL_JOG_LEFT_MAX_MA, APP_FLYWHEEL_JOG_RIGHT_MAX_MA,
            APP_FLYWHEEL_LEFT_CURRENT_SIGN, APP_FLYWHEEL_RIGHT_CURRENT_SIGN,
            (unsigned)0U, (long)FLYWHEEL_JOG_MAX_ERPM,
            (unsigned)APP_FLYWHEEL_JOG_ENABLED,
            (unsigned)1U,
            (long)APP_FLYWHEEL_SPEED_COAST_ERPM,
            (long)APP_FLYWHEEL_SPEED_RESUME_ERPM,
            APP_FLYWHEEL_JOG_ENABLED ? "CURRENT_PI" : "OFF",
            (unsigned)APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
            (unsigned)APP_FLYWHEEL_SPEED_TARGET_RPM,
            (long)APP_FLYWHEEL_SPEED_TARGET_ERPM,
            (unsigned)1U, (long)FLYWHEEL_JOG_MAX_FEEDBACK_CURRENT_MA,
            (long)APP_FLYWHEEL_SPEED_RAMP_ERPMS,
            (long)APP_FLYWHEEL_SPEED_BAND_ERPM);
        SendText(telemetry);
        return;
    }
    if (strcmp(line, "STOP") == 0) {
#if APP_PITCH_ENABLED
        PitchConsole_CancelPreparation();
#endif
#if APP_YAW_ENABLED
        YawConsole_CancelPreparation();
#endif
        AppTasks_RequestStop();
        return;
    }
#if APP_FLYWHEEL_JOG_ENABLED
    if (strcmp(line, "WHEEL OFF") == 0) { SelectWheel(FLYWHEEL_JOG_OFF); return; }
    if (strcmp(line, "WHEEL STATUS") == 0) { PublishWheelStatus(); return; }
    if (strcmp(line, "WHEEL SPEED") == 0) { PublishWheelSpeed(); return; }
#endif

}

void AppConsole_CreateQueue(void)
/* 功能：建立命令接收队列并初始化显示状态和控制台运行标记。 */
{
    telemetry_enabled = 1U;
    view_hold_mask = 0U;
    speed_view = 0U;
#if APP_YAW_ENABLED
    YawDisplaySim_Reset(&yaw_display, APP_YAW_DISPLAY_DEFAULT_ENABLED);
#endif
#if APP_FEEDER_ENABLED
    feeder_view = 0U;
#endif
#if APP_PITCH_ENABLED
    pitch_view = 0U;
    PitchDisplaySim_Reset(&pitch_display, APP_PITCH_DISPLAY_DEFAULT_ENABLED);
    PitchConsole_Init();
#endif
    rx_queue = xQueueCreate(APP_CONSOLE_RX_QUEUE_LENGTH, sizeof(ConsoleByte));
    if (rx_queue == 0) { AppFault_Fatal(APP_FAULT_QUEUE); }
    vQueueAddToRegistry(rx_queue, "CONSOLE_BYTES");
}
static HAL_StatusTypeDef StartConsoleRx(void)
/* 功能：按配置启动 USART1 的 DMA 接收或单字节中断接收。 */
{
#if APP_CONSOLE_RX_DMA_ENABLED
    return ConsoleUart_StartReceive(rx_dma, sizeof(rx_dma));
#else
    return HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
#endif
}
void AppConsole_Start(void)
/* 功能：启动控制台接收，失败时按原故障路径记录并请求重新接收。 */
{ if (StartConsoleRx() != HAL_OK) { AppFault_Fatal(APP_FAULT_UART_START); } }
uint32_t AppConsole_GetFaultEpoch(void)
/* 功能：返回控制台累计故障代次，供控制任务识别旧命令和撤权。 */
{ return fault_epoch; }
uint8_t AppConsole_GetViewHoldMask(void)
/* 功能：返回当前视图要求保持的另一轴掩码。 */
{ return view_hold_mask; }

/* 全工程只有这一份UART回调；UART4交给遥控模块，USART1交给控制台。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
/* 功能：分流 UART 接收完成事件，将控制台字节入队或交给遥控接收模块。 */
{
#if !APP_CONSOLE_RX_DMA_ENABLED
    ConsoleByte item;
    BaseType_t woken = pdFALSE;
#endif
    if (huart == &huart4) { RemoteInput_RxCplt(huart); return; }
#if !APP_CONSOLE_RX_DMA_ENABLED
    if (huart != &huart1 || HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    item.byte = rx_byte;
    item.received_ms = HAL_GetTick();
    item.epoch = fault_epoch;
    if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) { fault_epoch++; }
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK) { fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
#endif
}

#if APP_CONSOLE_RX_DMA_ENABLED || APP_REMOTE_RX_DMA_ENABLED
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
/* 功能：分流串口 DMA 接收事件，将本批有效字节按接收时间入队。 */
{
#if APP_CONSOLE_RX_DMA_ENABLED
    ConsoleByte item;
    uint16_t i;
    BaseType_t woken = pdFALSE;
#endif
#if APP_REMOTE_RX_DMA_ENABLED
    if (huart == &huart4) { RemoteInput_RxEvent(huart, size); return; }
#endif
#if APP_CONSOLE_RX_DMA_ENABLED
    if (huart != &huart1 || HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    /* HT is disabled. Normal DMA IDLE/TC has already stopped reception.
     * Enqueue this stable bounded burst; parsing and TX remain in the task. */
    if (!size || size > sizeof(rx_dma)) { fault_epoch++; needs_rearm = 1U; return; }
    rx_bursts++; rx_bytes_seen += size;
    item.received_ms = HAL_GetTick(); item.epoch = fault_epoch;
    for (i = 0U; i < size; ++i) {
        item.byte = rx_dma[i];
        if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) {
            rx_queue_drops++; fault_epoch++; break;
        }
    }
    if (StartConsoleRx() != HAL_OK) { fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
#else
    (void)huart; (void)size;
#endif
}
#endif

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
/* 功能：分流串口错误，记录故障并通知相应接收模块恢复链路。 */
{
    if (huart == &huart4) { RemoteInput_Error(huart); return; }
    if (huart != &huart1) { return; }
#if APP_CONSOLE_RX_DMA_ENABLED
    rx_uart_errors++; last_uart_error = HAL_UART_GetError(huart);
#endif
    fault_epoch++;
    (void)HAL_UART_AbortReceive(huart);
    needs_rearm = 1U;
}

#if APP_PITCH_ENABLED
static void PublishPitchTelemetry(uint32_t now)
/* 功能：采集 Pitch 实测快照，按既有 VOFA 列顺序发布角度、控制量和成绩。 */
{
    PitchBackendState pitch;
    PitchStep step;
    RemoteInput_State remote;
    AppState app;
    CanBusState bus;
    int length, tail;
    float displayed_angle;
    uint8_t display_frame;
    taskENTER_CRITICAL();
    PitchBackend_GetState(&pitch); PitchBackend_GetStep(&step);
    taskEXIT_CRITICAL();
    RemoteInput_GetState(&remote); AppTasks_GetState(&app); CanBus_GetState(&bus);
    display_frame = pitch_display.enabled;
    displayed_angle = PitchDisplaySim_Angle(&pitch_display, &step,
        pitch.controller.angle_deg, pitch.controller.target_deg,
        (uint8_t)(pitch.online && pitch.active && !pitch.controller.faults && !pitch.stop_pending &&
            pitch.feedback_age_ms < PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS), now);
    /* Pitch：实测组65列，带标记的演示组68列；I58/I59为制动力矩/active。
     * I60~I64为轨迹状态；演示组追加I65实测角度、I66演示active、I67 armed。
     * 同时只发当前选择的一组，保持921600/5ms的线路预算。 */
    length = snprintf(telemetry, sizeof(telemetry),
        "%s:%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g"
        ",%u,%u,%u,%u,%lu,%lu,%u,%u,%u,%u,%lu,%u,%ld,%ld,%.6g,%.6g,%.6g,%u,%u,%ld,%ld"
        ",%.6g,%.6g,%u,%lu,%u,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u,%u,%.6g,%.6g,%.6g,%.6g,%.6g,%u,%.6g,%u,%.6g,%.6g,%u,%lu,%.6g\r\n",
        display_frame ? "pitch_demo" : "pitch",
        (double)displayed_angle, (double)pitch.controller.target_deg,
        (double)pitch.controller.cascade.target_velocity_dps, (double)pitch.controller.cascade.velocity_dps,
        (double)pitch.controller.velocity_dps, (double)pitch.controller.torque_nm, (double)pitch.feedback.torque_nm,
        (double)pitch.controller.cascade.position.kp, (double)pitch.controller.cascade.position.ki,
        (double)pitch.controller.cascade.position.kd, (double)pitch.controller.cascade.speed.kp,
        (double)pitch.controller.cascade.speed.ki, (double)pitch.controller.cascade.speed.kd,
        (double)(pitch.feedback.position_rad * 57.295779513f / pitch.controller.feedback_divisor),
        (unsigned)pitch.online, (unsigned)pitch.active, (unsigned)pitch.requested, (unsigned)pitch.feedback.status,
        (unsigned long)pitch.controller.faults, (unsigned long)pitch.feedback_age_ms,
        (unsigned)pitch.stop_pending, (unsigned)pitch.zero_set,
        (unsigned)pitch.controller.cascade.torque_limited, (unsigned)pitch.coord_confirmed,
        (unsigned long)step.id, (unsigned)step.phase,
        (long)(int32_t)step.response_ms, (long)(int32_t)step.settle_ms,
        (double)step.overshoot_deg, (double)step.overshoot_pct, (double)step.steady_error_deg,
        (unsigned)step.sse_valid, (unsigned)step.pass, (long)(int32_t)step.t90_ms,
        (long)(int32_t)(step.t10_ms != PITCH_STEP_UNREACHED && step.t90_ms != PITCH_STEP_UNREACHED ?
            step.t90_ms - step.t10_ms : PITCH_STEP_UNREACHED),
        (double)pitch.feedback.position_rad, (double)pitch.feedback.velocity_rad_s,
        (unsigned)pitch.controller.feedback_divisor, (unsigned long)pitch.rx_count,
        (unsigned)(remote.has_frame && (uint32_t)(now - remote.received_ms) < REMOTE_TIMEOUT_MS),
        (unsigned)app.logic.safety.operator_armed,
        (unsigned)remote.channels[4], (unsigned)remote.channels[5], (unsigned)remote.channels[6],
        (unsigned)remote.channels[7], (unsigned long)bus.fault_epoch,
        (unsigned)pitch.feedback.temp_mos, (unsigned)pitch.feedback.temp_rotor,
        (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3],
        (double)pitch.controller.gravity_kg_nm, (double)pitch.controller.gravity_offset_deg,
        (double)pitch.controller.gravity_torque_nm, (double)pitch.controller.pid_torque_nm,
        (double)pitch.controller.cascade.speed_i, (unsigned)pitch.controller.gravity_calibrated,
        (double)pitch.controller.brake.torque_nm, (unsigned)pitch.controller.brake.active,
        (double)pitch.controller.shape.position_deg, (double)pitch.controller.shape.velocity_dps,
        (unsigned)pitch.controller.shape.active, (unsigned long)pitch.controller.shape.duration_ms,
        (double)pitch.controller.shape.velocity_gain);
    if (length <= 0 || (size_t)length >= sizeof(telemetry)) { return; }
    if (display_frame) {
        tail = snprintf(telemetry + length - 2, sizeof(telemetry) - (size_t)length + 2U,
            ",%.6g,%u,%u\r\n", (double)pitch.controller.angle_deg,
            (unsigned)pitch_display.active, (unsigned)pitch_display.armed);
        if (tail <= 0 || (size_t)tail >= sizeof(telemetry) - (size_t)length + 2U) { return; }
    }
    SendText(telemetry);
}
#endif

#if APP_FLYWHEEL_JOG_ENABLED
static void PublishWheelTelemetry(void)
/* 功能：发布两侧摩擦轮实测及目标转速，保留既有速度视图通道。 */
{
    MotorJogState jog;
    double left_target = 0.0, right_target = 0.0;
    MotorBackend_GetJogState(&jog);
    if (jog.active) {
        if (jog.last_speed_selection == FLYWHEEL_JOG_LEFT || jog.last_speed_selection == FLYWHEEL_JOG_BOTH) {
            left_target = (double)jog.speed.target_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED;
        }
        if (jog.last_speed_selection == FLYWHEEL_JOG_RIGHT || jog.last_speed_selection == FLYWHEEL_JOG_BOTH) {
            right_target = (double)jog.speed_right.target_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED;
        }
    }
    (void)snprintf(telemetry, sizeof(telemetry),
        "wheel_speed:%.6g,%.6g,%.6g,%.6g,%ld,%ld,%u,%lu,%ld,%ld,%u,%u\r\n",
        (double)jog.health.feedback[0].erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (double)jog.health.feedback[1].erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        left_target, right_target,
        (long)jog.health.feedback[0].erpm, (long)jog.health.feedback[1].erpm,
        (unsigned)jog.health.status1_fresh_mask, (unsigned long)jog.health.faults,
        (long)jog.requested_left_mA, (long)jog.requested_right_mA,
        (unsigned)jog.active, (unsigned)APP_FLYWHEEL_POLE_PAIRS_ASSUMED);
    SendText(telemetry);
}
#endif

static void PublishYawTelemetry(uint32_t now)
/* 功能：采集 Yaw 实测快照，按既有 VOFA 列顺序发布角度、控制量和成绩。 */
{
    AppState app;
    CanBusState bus;
    RemoteInput_State remote;
#if !APP_YAW_ENABLED
    TargetRequest *request;
#endif
    uint32_t age;
    int length;
#if APP_FLYWHEEL_JOG_ENABLED
    MotorJogState jog;
#endif
#if APP_YAW_ENABLED
    YawBackendState yaw;
    YawStep step;
    float displayed_angle;
    uint8_t display_frame;
    int tail;
#endif
    AppTasks_GetState(&app);
    CanBus_GetState(&bus);
    RemoteInput_GetState(&remote);
#if APP_FLYWHEEL_JOG_ENABLED
    MotorBackend_GetJogState(&jog);
#endif
#if APP_YAW_ENABLED
    taskENTER_CRITICAL();
    YawBackend_GetState(&yaw); YawBackend_GetStep(&step);
    taskEXIT_CRITICAL();
    display_frame = yaw_display.enabled;
    displayed_angle = YawDisplaySim_Angle(&yaw_display, &step,
        yaw.controller.angle_deg, yaw.controller.target_deg,
        (uint8_t)(yaw.online && yaw.active && !yaw.controller.faults && !yaw.stop_pending &&
            yaw.feedback_age_ms < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS), now);
#endif
    age = remote.has_frame ? (uint32_t)(now - remote.received_ms) : 0U;
#if APP_YAW_ENABLED
    /* 原索引不移动，共72列。I60/I61/I62改为±0.2首次/最终进入时间/总判定，
     * 与I26/I27/I32为同一份结果；总判定同时检查最终稳定时间、超调与稳态。
     * I49~I53旧补偿保留零占位，I54前馈限幅、I55加速度前馈，
     * I64~I69为规划时间/活动/位置/速度/加速度/跟踪误差，I70速度前馈权重、I71加速度系数。
     * 前馈参数4位有效数字，诊断5位，减少5ms/921600线路开销。 */
    length = snprintf(telemetry, sizeof(telemetry),
        "%s:%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g"
        ",%u,%u,%u,%u,%lu,%lu,%u,%u,%u,%u,%lu,%u,%ld,%ld,%.6g,%.6g,%.6g,%u,%u,%ld,%ld"
        ",%u,%.6g,%.6g,%lu,%u,%u,%u,%u,%u,%u,%lu,%d,%d"
        ",%u,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%.5g,%.5g,%.5g,%.5g,%u,%ld,%ld,%u,%lu"
        ",%lu,%u,%.5g,%.5g,%.5g,%.5g,%.4g,%.5g\r\n",
        display_frame ? "gimbal_demo" : "gimbal",
        (double)displayed_angle, (double)yaw.controller.target_deg,
        (double)yaw.controller.cascade.target_velocity_dps, (double)yaw.controller.cascade.velocity_dps,
        (double)yaw.controller.velocity_dps, (double)yaw.controller.torque_nm, (double)yaw.feedback.torque_nm,
        (double)yaw.controller.cascade.position.kp, (double)yaw.controller.cascade.position.ki,
        (double)yaw.controller.cascade.position.kd, (double)yaw.controller.cascade.speed.kp,
        (double)yaw.controller.cascade.speed.ki, (double)yaw.controller.cascade.speed.kd,
        (double)yaw.controller.cascade.velocity_limit_dps,
        (unsigned)yaw.online, (unsigned)yaw.active, (unsigned)yaw.requested, (unsigned)yaw.feedback.status,
        (unsigned long)yaw.controller.faults, (unsigned long)yaw.feedback_age_ms,
        (unsigned)yaw.stop_pending, (unsigned)yaw.zero_set,
        (unsigned)yaw.controller.cascade.torque_limited, (unsigned)yaw.controller.cascade.velocity_limited,
        (unsigned long)step.id, (unsigned)step.phase,
        (long)(int32_t)step.response_ms, (long)(int32_t)step.settle_ms,
        (double)step.overshoot_deg, (double)step.overshoot_pct, (double)step.steady_error_deg,
        (unsigned)step.sse_valid, (unsigned)step.pass, (long)(int32_t)step.t90_ms,
        (long)(int32_t)(step.t10_ms != YAW_STEP_UNREACHED && step.t90_ms != YAW_STEP_UNREACHED ?
            step.t90_ms - step.t10_ms : YAW_STEP_UNREACHED),
#if APP_FLYWHEEL_JOG_ENABLED
        (unsigned)jog.active, (double)jog.speed.filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (double)jog.speed_right.filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED, (unsigned long)jog.health.faults,
#else
        0U, 0.0, 0.0, 0UL,
#endif
        (unsigned)(remote.has_frame && age < REMOTE_TIMEOUT_MS), (unsigned)app.logic.safety.operator_armed,
        (unsigned)remote.channels[4], (unsigned)remote.channels[5], (unsigned)remote.channels[6],
        (unsigned)remote.channels[7], (unsigned long)bus.fault_epoch,
        (int)yaw.feedback.driver_temp_c, (int)yaw.feedback.motor_temp_c,
        (unsigned)0U, 0.0, 0.0, 0.0, 0.0, 0.0,
        (double)0.10f, (double)yaw.controller.cascade.feedforward_nm,
        (double)yaw.controller.pid_torque_nm, (double)yaw.controller.cascade.speed_i,
        (double)yaw.controller.cascade.position_i, (unsigned)yaw.controller.cascade.feedforward_limited,
        (long)(int32_t)step.response_ms, (long)(int32_t)step.settle_ms, (unsigned)step.pass,
        (unsigned long)yaw.controller.last_feedback_ms,
        (unsigned long)0U, (unsigned)0U,
        (double)yaw.controller.target_deg, (double)0.0f,
        (double)0.0f,
        (double)(yaw.controller.target_deg - yaw.controller.angle_deg),
        (double)0.0f, (double)0.0f);
    if (length > 0 && (size_t)length < sizeof(telemetry) && display_frame) {
        tail = snprintf(telemetry + length - 2, sizeof(telemetry) - (size_t)length + 2U,
            ",%.6g,%u,%u\r\n", (double)yaw.controller.angle_deg,
            (unsigned)yaw_display.active, (unsigned)yaw_display.armed);
        if (tail <= 0 || (size_t)tail >= sizeof(telemetry) - (size_t)length + 2U) { return; }
        length += tail - 2;
    }
#else
    request = &app.logic.targets.request;
    length = snprintf(telemetry, sizeof(telemetry),
        "gimbal:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%.2f,%.2f,%u,%u,%lu,%u,%.1f,%lu,%lu,%lu,%u,%lu,%lu"
#if APP_FLYWHEEL_JOG_ENABLED
        ",%ld,%ld,%ld,%ld,%u,%ld,%u,%lu,%u,%u"
        ",%ld,%ld,%.1f,%u,%u,%ld,%ld,%ld,%.1f,%u,%u,%u"
#endif
        "\r\n",
        (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3],
        (unsigned)remote.channels[4], (unsigned)remote.channels[5],
        (unsigned)remote.channels[6], (unsigned)remote.channels[7],
        (unsigned)remote.channels[8], (unsigned)remote.channels[9],
        (unsigned)(remote.has_frame && age < REMOTE_TIMEOUT_MS),
        (unsigned)app.logic.safety.operator_armed, (unsigned)CanBus_OutputAllowed(),
        (unsigned)app.logic.safety.state, (unsigned)app.logic.safety.reason, (unsigned long)age,
        (double)request->yaw_rate_dps, (double)request->pitch_rate_dps,
        (unsigned)request->friction_requested, (unsigned)request->fire_mode,
        (unsigned long)request->shot_sequence, (unsigned)request->gimbal_mode,
        (double)request->step_delta_deg, (unsigned long)request->step_sequence,
        (unsigned long)bus.rx_frames[0], (unsigned long)bus.rx_frames[1],
        (unsigned)app.motors.ready, (unsigned long)remote.fault_epoch, (unsigned long)remote.rx_bytes
#if APP_FLYWHEEL_JOG_ENABLED
        , (long)jog.health.feedback[0].erpm, (long)jog.health.feedback[1].erpm,
        (long)jog.health.feedback[0].motor_current_mA,
        (long)jog.health.feedback[1].motor_current_mA, (unsigned)jog.selection,
        (long)jog.requested_mA, (unsigned)jog.active, (unsigned long)jog.health.faults,
        (unsigned)jog.stop_pending, (unsigned)jog.rearm_ready
        , (long)jog.speed.target_erpm, (long)jog.speed.filtered_erpm,
        (double)jog.speed.filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (unsigned)jog.speed.at_limit, (unsigned)jog.speed.in_band,
        (long)jog.requested_right_mA, (long)jog.speed_right.target_erpm,
        (long)jog.speed_right.filtered_erpm,
        (double)jog.speed_right.filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (unsigned)jog.speed_right.at_limit, (unsigned)jog.speed_right.in_band,
        (unsigned)jog.stop_reason
#endif
        );
#endif
    if (length > 0 && (size_t)length < sizeof(telemetry)) {
        if (HAL_UART_Transmit(&huart1, (uint8_t *)telemetry, (uint16_t)length,
                             APP_YAW_ENABLED ? 90U : 40U) != HAL_OK) {
            TransmitFailed();
        }
    }
}

/* 独立调度帧，不在Yaw的大快照栈内嵌套Pitch格式化调用。
 * AC5/O0会为整个函数预留栈，分派后各轴只使用自身快照。 */
static void PublishTelemetry(uint32_t now)
/* 功能：按输出周期和当前视图选择对应遥测，控制发送频率。 */
{
    /* Pause only FireWater snapshots, never command replies, safety or step reports. */
    if (!telemetry_enabled) { telemetry_ms = now; return; }
    if ((uint32_t)(now - telemetry_ms) < APP_TELEMETRY_PERIOD_MS) { return; }
#if APP_YAW_ENABLED || APP_PITCH_ENABLED
    /* 迟到只发送当前一帧，保留固定相位，不追发旧快照。 */
    telemetry_ms += (uint32_t)(now - telemetry_ms) / APP_TELEMETRY_PERIOD_MS * APP_TELEMETRY_PERIOD_MS;
#else
    telemetry_ms = now;
#endif
#if APP_FEEDER_ENABLED
    if (speed_view == 1U) { FeederConsole_SpeedTelemetry(telemetry, sizeof(telemetry)); SendText(telemetry); return; }
    if (feeder_view) { FeederConsole_Telemetry(telemetry, sizeof(telemetry)); SendText(telemetry); return; }
#endif
#if APP_FLYWHEEL_JOG_ENABLED
    if (speed_view == 2U) { PublishWheelTelemetry(); return; }
#endif
#if APP_PITCH_ENABLED
    if (pitch_view) { PublishPitchTelemetry(now); return; }
#endif
    PublishYawTelemetry(now);
}

void AppConsole_Poll(void)
/* 功能：恢复接收故障、组装完整命令，并轮询准备回执和定时遥测。 */
{
    ConsoleByte item;
    uint32_t i;
    PublishRemoteRx(1U);
    if (needs_rearm) {
        taskENTER_CRITICAL();
        (void)HAL_UART_AbortReceive(&huart1);
        if (StartConsoleRx() == HAL_OK) { needs_rearm = 0U; }
        taskEXIT_CRITICAL();
    }
    if (observed_epoch != fault_epoch) {
#if APP_PITCH_ENABLED
        PitchConsole_CancelPreparation();
#endif
        observed_epoch = fault_epoch;
        line_length = 0U;
        discard_line = 1U;
#if APP_CONSOLE_RX_DMA_ENABLED
        (void)snprintf(telemetry, sizeof(telemetry),
            "C|UART_FAULT|EPOCH|%lu|LAST|0x%08lX|RX_ERR|%lu|DROP|%lu|TX_ERR|%lu|RX_BYTES|%lu|BURSTS|%lu|RX_DMA|1\r\n",
            (unsigned long)observed_epoch, (unsigned long)last_uart_error,
            (unsigned long)rx_uart_errors, (unsigned long)rx_queue_drops, (unsigned long)tx_errors,
            (unsigned long)rx_bytes_seen, (unsigned long)rx_bursts);
        SendText(telemetry);
#endif
    }
    for (i = 0U; i < APP_CONSOLE_RX_QUEUE_LENGTH; ++i) {
        if (xQueueReceive(rx_queue, &item, 0U) != pdPASS) { break; }
        if (item.epoch != fault_epoch || (uint32_t)(HAL_GetTick() - item.received_ms) >= APP_CONSOLE_INPUT_TIMEOUT_MS) {
            line_length = 0U; discard_line = 1U; continue;
        }
        if (line_length && (uint32_t)(item.received_ms - last_byte_ms) >= APP_CONSOLE_INPUT_TIMEOUT_MS) {
            line_length = 0U; discard_line = 1U;
        }
        last_byte_ms = item.received_ms;
        if (item.byte == '\r' || item.byte == '\n') {
            line[line_length] = '\0';
            if (!discard_line) { HandleLine(); }
            line_length = 0U; discard_line = 0U;
        }
        else if (!discard_line) {
            /* 命令只接受可打印ASCII，防止NUL把带尾部垃圾的行截成合法前缀。 */
            if (item.byte < 0x20U || item.byte > 0x7EU) { line_length = 0U; discard_line = 1U; }
            else if (line_length < sizeof(line) - 1U) { line[line_length++] = (char)item.byte; }
            else { line_length = 0U; discard_line = 1U; }
        }
    }
#if APP_YAW_ENABLED
    if (YawConsole_Poll(telemetry, sizeof(telemetry))) { SendText(telemetry); }
#endif
#if APP_PITCH_ENABLED
    if (PitchConsole_Poll(telemetry, sizeof(telemetry))) { SendText(telemetry); }
#endif
#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
    PublishStartup(1U);
#endif
    PublishTelemetry(HAL_GetTick());
}
