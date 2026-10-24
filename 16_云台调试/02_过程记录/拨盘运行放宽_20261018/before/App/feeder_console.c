#include "feeder_console.h"
#include "feeder_backend.h"
#include "motor_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static void Status(const char *operation, uint8_t ok, char *out, size_t capacity)
{
    FeederBackendState state;
    MotorHealth wheels;
    const FeederController *c;
    FeederBackend_GetState(&state); MotorBackend_GetHealth(&wheels); c = &state.controller;
    (void)snprintf(out, capacity,
        "F|%s|%s|%s|ONLINE|%u|RX|%lu|AGE|%lu|RAW|%u|ROTOR_DEG|%.4f|POS_DEG|%.4f|TARGET_DEG|%.4f|RPM|%d|VREF|%.2f|CMD_RAW|%d|FB_TORQUE_RAW|%d|ARMED|%u|ZERO|%u|BUSY|%u|FAULT|%lu|STOP_PENDING|%u|WHEELS_READY|%u|STEP|%.6g|DIR|%d|GEAR|%.6g|CAL|%u|PKP|%.6g|VKP|%.6g|VKI|%.6g|MAX_RPM|%.6g|LIMIT|%.6g|TOL|%.6g|SETTLE_RPM|%.6g|SETTLE_MS|%lu|TIMEOUT_MS|%lu|ACCEPT|%lu|DONE|%lu|REJECT|%lu|ABORT|%lu|SINGLE|%lu|CONT|%lu|DONE_DT_MS|%lu\r\n",
        operation, ok ? "OK" : "REJECTED", APP_FEEDER_VERSION,
        (unsigned)state.online, (unsigned long)c->feedback_sequence,
        c->have_feedback ? (unsigned long)(HAL_GetTick() - c->last_feedback_ms) : 0xFFFFFFFFUL,
        (unsigned)c->raw_angle, c->position_deg, c->axis_position_deg, c->axis_target_deg,
        (int)c->raw_rpm, (double)c->target_rpm, (int)c->command_raw, (int)c->raw_current,
        (unsigned)state.armed, (unsigned)c->have_zero, (unsigned)c->busy, (unsigned long)c->faults,
        (unsigned)state.stop_pending, (unsigned)wheels.flywheels_ready,
        (double)c->config.step_deg, (int)c->config.direction, (double)c->config.gear_ratio,
        (unsigned)c->config.calibrated, (double)c->config.pos_kp, (double)c->config.speed_kp,
        (double)c->config.speed_ki, (double)c->config.max_rpm, (double)c->config.current_limit,
        (double)c->config.tolerance_deg, (double)c->config.settle_rpm,
        (unsigned long)c->config.settle_ms, (unsigned long)c->config.motion_timeout_ms,
        (unsigned long)c->accepted, (unsigned long)c->completed, (unsigned long)c->rejected,
        (unsigned long)c->aborted, (unsigned long)state.single_accepted,
        (unsigned long)state.continuous_accepted, (unsigned long)state.completed_interval_ms);
}

static void Check(char *out, size_t capacity)
{
    FeederBackendState state;
    RemoteInput_State remote;
    AppState app;
    CanBusState bus;
    const FeederJumpDiagnostic *jump;
    const char *cause;
    char data[17];
    uint32_t i, now, uart_epoch;
    static const char hex[] = "0123456789ABCDEF";
    /* Read one coherent snapshot, format only after interrupts are unmasked. */
    taskENTER_CRITICAL();
    FeederBackend_GetState(&state); RemoteInput_GetState(&remote);
    AppTasks_GetState(&app); CanBus_GetState(&bus);
    now = HAL_GetTick(); uart_epoch = remote.fault_epoch + AppConsole_GetFaultEpoch();
    taskEXIT_CRITICAL();
    jump = &state.controller.jump;
    cause = !jump->valid ? "NONE" : jump->flags == FEEDER_JUMP_DELTA ? "DELTA" :
        jump->flags == FEEDER_JUMP_HALF_TURN ? "HALF_TURN" : "DELTA_AND_HALF_TURN";
    for (i = 0U; i < 8U; ++i) {
        data[i * 2U] = hex[jump->data[i] >> 4];
        data[i * 2U + 1U] = hex[jump->data[i] & 15U];
    }
    data[16] = '\0';
    (void)snprintf(out, capacity,
        "F|CHECK|%s|FAULT|%lu|ONLINE|%u|ARMED|%u|BUSY|%u|CAL|%u|DIR|%d|STEP|%.6g|JUMP_VALID|%u|JUMP_COUNT|%lu|CAUSE|%s|FLAGS|%lu|AT_MS|%lu|SEQ|%lu|PREV_RAW|%u|RAW|%u|DELTA_COUNTS|%ld|GAP_MS|%lu|FRAME_AGE_MS|%lu|PREV_RPM|%d|RPM|%d|LIMIT_COUNTS|%.6g|EXPECTED_COUNTS|%.6g|FB_TORQUE_RAW|%d|BEFORE_CMD|%d|BEFORE_BUSY|%u|BEFORE_ZERO|%u|FRAME|%s|RC_AGE_MS|%lu|CH|%u,%u,%u,%u,%u,%u,%u,%u|SAFETY|%u|SAFETY_REASON|%u|BARRIER|%u|OP_ARMED|%u|UART_CURRENT|%lu|UART_SEEN|%lu|CAN_EPOCH|%lu\r\n",
        APP_FEEDER_VERSION, (unsigned long)state.controller.faults,
        (unsigned)state.online, (unsigned)state.armed, (unsigned)state.controller.busy,
        (unsigned)state.controller.config.calibrated, (int)state.controller.config.direction,
        (double)state.controller.config.step_deg, (unsigned)jump->valid,
        (unsigned long)jump->events, cause, (unsigned long)jump->flags,
        (unsigned long)jump->at_ms, (unsigned long)jump->sequence,
        (unsigned)jump->previous_angle, (unsigned)jump->angle, (long)jump->delta_counts,
        (unsigned long)jump->gap_ms, (unsigned long)jump->age_ms,
        (int)jump->previous_rpm, (int)jump->rpm, (double)jump->limit_counts,
        (double)jump->expected_counts, (int)jump->feedback_raw, (int)jump->command_raw,
        (unsigned)jump->busy, (unsigned)jump->zero_valid, data,
        remote.has_frame ? (unsigned long)(now - remote.received_ms) : 0xFFFFFFFFUL,
        (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3],
        (unsigned)remote.channels[4], (unsigned)remote.channels[5],
        (unsigned)remote.channels[6], (unsigned)remote.channels[7],
        (unsigned)app.logic.safety.state, (unsigned)app.logic.safety.reason,
        (unsigned)app.logic.safety.barrier_valid, (unsigned)app.logic.safety.operator_armed,
        (unsigned long)uart_epoch, (unsigned long)app.logic.safety.uart_epoch,
        (unsigned long)bus.fault_epoch);
}

uint8_t FeederConsole_Handle(const char *line, char *out, size_t capacity)
{
    const char *argument;
    char *end;
    char key[16];
    float value;
    size_t length;
    uint8_t ok = 0U;
    FeederBackendState state;
    FeederConfig config;
    if (strncmp(line, "FEEDER", 6U) != 0 || (line[6] && line[6] != ' ')) { return 0U; }
    if (!strcmp(line, "FEEDER HELP")) {
        (void)snprintf(out, capacity,
            "F|HELP|READ|FEEDER INFO;STATUS;PID;VIEW;CHECK|CHECK|FIRST_JUMP_SINCE_CLEAR|SAFE_OFF|OFF;DIR +-1;GEAR ratio;CAL 1;CLEAR;ZERO;ARM|ACTION|SINGLE_REQUIRES_NORMAL_AND_REAL_WHEELS_READY|TUNE|PKP;VKP;VKI;RPM;LIMIT;TOL;SETTLE;TIMEOUT;STEP|BOOT|CAL0_DIR0_NO_MOTION|SHAFT|40DEG_CLOCKWISE_FROM_TOP|RAM_ONLY|1\r\n"); return 1U;
    }
    if (!strcmp(line, "FEEDER CHECK")) { Check(out, capacity); return 1U; }
    if (!strcmp(line, "FEEDER") || !strcmp(line, "FEEDER INFO") ||
        !strcmp(line, "FEEDER STATUS") || !strcmp(line, "FEEDER PID")) {
        Status("STATUS", 1U, out, capacity); return 1U;
    }
    if (!strcmp(line, "FEEDER OFF")) { FeederBackend_Stop(); ok = 1U; }
    else if (!strcmp(line, "FEEDER CLEAR")) { ok = FeederBackend_Clear(); }
    else if (!strcmp(line, "FEEDER ZERO")) { ok = FeederBackend_Zero(); }
    else if (!strcmp(line, "FEEDER ARM")) { ok = FeederBackend_Arm(); }
    else if (!strcmp(line, "FEEDER SINGLE")) { ok = FeederBackend_Single(); }
    else {
        argument = strchr(line + 7U, ' ');
        if (!argument) { Status("BAD_COMMAND", 0U, out, capacity); return 1U; }
        length = (size_t)(argument - (line + 7U));
        if (!length || length >= sizeof(key)) { Status("BAD_COMMAND", 0U, out, capacity); return 1U; }
        memcpy(key, line + 7U, length); key[length] = '\0';
        value = strtof(argument + 1U, &end);
        if (end == argument + 1U || *end || !isfinite(value)) {
            Status("BAD_VALUE", 0U, out, capacity); return 1U;
        }
        FeederBackend_GetState(&state); config = state.controller.config;
        if (!strcmp(key, "PKP")) { config.pos_kp = value; }
        else if (!strcmp(key, "VKP")) { config.speed_kp = value; }
        else if (!strcmp(key, "VKI")) { config.speed_ki = value; }
        else if (!strcmp(key, "RPM")) { config.max_rpm = value; }
        else if (!strcmp(key, "LIMIT")) { config.current_limit = value; }
        else if (!strcmp(key, "TOL")) { config.tolerance_deg = value; }
        else if (!strcmp(key, "STEP")) { config.step_deg = value; }
        else if (!strcmp(key, "GEAR")) { config.gear_ratio = value; }
        else if (!strcmp(key, "SETTLE") && value >= 0 && value <= 1000 && floorf(value) == value) {
            config.settle_ms = (uint32_t)value;
        } else if (!strcmp(key, "TIMEOUT") && value >= 20 && value <= 10000 && floorf(value) == value) {
            config.motion_timeout_ms = (uint32_t)value;
        } else if (!strcmp(key, "DIR") && (value == -1 || value == 1 || value == 0)) {
            config.direction = (int8_t)value;
        } else if (!strcmp(key, "CAL") && (value == 0 || value == 1)) {
            config.calibrated = (uint8_t)value;
        } else { Status("BAD_KEY_OR_VALUE", 0U, out, capacity); return 1U; }
        ok = FeederBackend_SetConfig(&config);
    }
    Status(line + 7U, ok, out, capacity); return 1U;
}

void FeederConsole_Telemetry(char *out, size_t capacity)
{
    FeederBackendState state;
    MotorHealth wheels;
    const FeederController *c;
    FeederBackend_GetState(&state); MotorBackend_GetHealth(&wheels); c = &state.controller;
    (void)snprintf(out, capacity,
        "feeder:%.7g,%.7g,%d,%.6g,%d,%d,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d,%.6g,%u,%u,%u,%u,%u,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu,%.6g,%.6g,%lu,%lu,0,%.8g,%.8g\r\n",
        c->axis_position_deg, c->axis_target_deg, (int)c->raw_rpm, (double)c->target_rpm,
        (int)c->command_raw, (int)c->raw_current,
        (double)c->config.pos_kp, (double)c->config.speed_kp, (double)c->config.speed_ki,
        (double)c->config.max_rpm, (double)c->config.current_limit, (double)c->config.step_deg,
        (int)c->config.direction, (double)c->config.gear_ratio, (unsigned)c->config.calibrated,
        (unsigned)c->have_zero, (unsigned)state.online, (unsigned)state.armed, (unsigned)c->busy,
        (unsigned long)c->faults, (unsigned)wheels.flywheels_ready,
        (unsigned long)c->accepted, (unsigned long)c->completed, (unsigned long)c->rejected,
        (unsigned long)c->aborted, (unsigned long)c->feedback_sequence,
        c->have_feedback ? (unsigned long)(HAL_GetTick() - c->last_feedback_ms) : 0xFFFFFFFFUL,
        (unsigned)c->raw_angle, (unsigned)state.stop_pending,
        (unsigned long)state.single_accepted, (unsigned long)state.continuous_accepted,
        (unsigned long)state.completed_interval_ms, (double)c->config.tolerance_deg,
        (double)c->config.settle_rpm, (unsigned long)c->config.settle_ms,
        (unsigned long)c->config.motion_timeout_ms, c->position_deg, c->target_deg);
}
