#include "feeder_console.h"
#include "feeder_backend.h"
#include "motor_backend.h"
#include "app_config.h"
#include "can_bus.h"
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
            "F|HELP|READ|FEEDER INFO;STATUS;PID;VIEW|SAFE_OFF|OFF;DIR +-1;GEAR ratio;CAL 1;CLEAR;ZERO;ARM|ACTION|SINGLE_REQUIRES_NORMAL_AND_REAL_WHEELS_READY|TUNE|PKP;VKP;VKI;RPM;LIMIT;TOL;SETTLE;TIMEOUT;STEP|BOOT|CAL0_DIR0_NO_MOTION|SHAFT|40DEG_CLOCKWISE_FROM_TOP|RAM_ONLY|1\r\n"); return 1U;
    }
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
