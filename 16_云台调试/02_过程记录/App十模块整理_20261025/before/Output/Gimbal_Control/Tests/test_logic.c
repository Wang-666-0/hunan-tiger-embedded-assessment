#include "app_logic.h"
#include "app_config.h"
#include "motor_backend.h"
#include "can_frame.h"
#include <math.h>
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static void Near(float actual, float expected) { CHECK(fabsf(actual - expected) < 0.0001f); }

static void InputInit(SafetyInput *input, uint32_t now)
{
    uint32_t i;
    memset(input, 0, sizeof(*input));
    input->now_ms = now;
    input->remote.received_ms = now;
    input->remote.sequence = 1U;
    input->remote.has_frame = 1U;
    for (i = 0; i < 14U; ++i) { input->remote.channels[i] = 1500U; }
    input->remote.channels[RC_TRIGGER_CHANNEL] = 1000U;
    input->remote.channels[RC_ARM_CHANNEL] = 1000U;
    input->remote.channels[RC_FAILSAFE_CHANNEL] = 2000U;
    input->remote.channels[RC_FIRE_MODE_CHANNEL] = 1000U;
    input->remote.channels[RC_FRICTION_CHANNEL] = 1000U;
    input->remote.channels[RC_TEST_MODE_CHANNEL] = 1000U;
}
static void Frame(SafetyInput *input, uint16_t arm)
{
    input->now_ms += 7U;
    input->remote.received_ms = input->now_ms;
    input->remote.sequence++;
    input->remote.channels[RC_ARM_CHANNEL] = arm;
}
static void Arm(AppLogic *logic, SafetyInput *input, uint32_t now)
{
    AppLogic_Init(logic); InputInit(input, now);
    AppLogic_Step(logic, input, 0U);
    CHECK(logic->safety.state == SAFETY_READY);
    Frame(input, 2000U); AppLogic_Step(logic, input, 0U);
    CHECK(logic->safety.operator_armed);
    CHECK(!logic->safety.output_allowed);
}

static void CheckMapper(void)
{
    SafetyInput input;
    RemoteCommand command;
    uint32_t i;
    test_stage = 1U; InputInit(&input, 10U);
    RemoteMapper_Map(&input.remote, &command); CHECK(command.valid && command.neutral);
    for (i = 0U; i < 10U; ++i) {
        uint16_t saved = input.remote.channels[i];
        input.remote.channels[i] = 799U; RemoteMapper_Map(&input.remote, &command); CHECK(!command.valid);
        input.remote.channels[i] = 2201U; RemoteMapper_Map(&input.remote, &command); CHECK(!command.valid);
        input.remote.channels[i] = saved;
    }
    input.remote.channels[RC_YAW_CHANNEL] = 1000U;
    input.remote.channels[RC_PITCH_CHANNEL] = 2000U;
    RemoteMapper_Map(&input.remote, &command);
    Near(command.yaw_rate_dps, -APP_YAW_RATE_LIMIT_DPS);
    Near(command.pitch_rate_dps, APP_PITCH_RATE_LIMIT_DPS);
    CHECK(!command.neutral);
    input.remote.channels[RC_YAW_CHANNEL] = 1530U;
    input.remote.channels[RC_PITCH_CHANNEL] = 1470U;
    RemoteMapper_Map(&input.remote, &command); CHECK(command.neutral);
    input.remote.channels[RC_ARM_CHANNEL] = 1700U;
    RemoteMapper_Map(&input.remote, &command); CHECK(command.arm == RC_MIDDLE);
    input.remote.channels[RC_ARM_CHANNEL] = 1701U;
    RemoteMapper_Map(&input.remote, &command); CHECK(command.arm == RC_HIGH);
}

static void CheckSafety(void)
{
    AppLogic logic;
    SafetyInput input;
    test_stage = 2U;
    AppLogic_Init(&logic); InputInit(&input, 10U);
    input.remote.channels[RC_ARM_CHANNEL] = 2000U;
    AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.operator_armed);
    Frame(&input, 1000U); AppLogic_Step(&logic, &input, 0U);
    input.remote.channels[RC_ARM_CHANNEL] = 2000U; /* 旧快照不能产生解锁沿。 */
    AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.operator_armed);
    Frame(&input, 2000U); input.remote.channels[RC_YAW_CHANNEL] = 1600U;
    AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_NOT_NEUTRAL && !logic.safety.operator_armed);
    input.remote.channels[RC_YAW_CHANNEL] = 1500U;
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.operator_armed);
    Frame(&input, 1000U); AppLogic_Step(&logic, &input, 0U);
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.operator_armed);
    input.now_ms = input.remote.received_ms + 99U; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.operator_armed);
    input.now_ms++; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_REMOTE && !logic.safety.operator_armed);
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.operator_armed);

    Arm(&logic, &input, 200U);
    input.remote.channels[RC_FAILSAFE_CHANNEL] = 1000U;
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_FAILSAFE && !logic.safety.operator_armed);
    input.remote.channels[RC_FAILSAFE_CHANNEL] = 2000U;
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.operator_armed);

    Arm(&logic, &input, 300U);
    input.stop_sequence++; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_STOP && !logic.safety.operator_armed);
    input.remote.sequence++; input.remote.channels[RC_ARM_CHANNEL] = 1000U;
    AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.off_seen); /* 同毫秒OFF拒绝。 */
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.operator_armed);
    Frame(&input, 1000U); AppLogic_Step(&logic, &input, 0U);
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.operator_armed);
    input.uart_epoch++; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_UART && !logic.safety.operator_armed);
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.operator_armed);
    Arm(&logic, &input, 400U); input.can_epoch++;
    AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.reason == SAFETY_CAN);
    Arm(&logic, &input, 500U); input.control_late = 1U;
    AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.reason == SAFETY_TIMING);
    Arm(&logic, &input, 600U); input.actuator_fault = 1U;
    AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.reason == SAFETY_ACTUATOR);
    Arm(&logic, &input, 700U); input.backend_ready = 1U; input.feedback_ready = 1U;
    AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.output_allowed);
    input.feedback_ready = 0U; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_FEEDBACK && !logic.safety.operator_armed);
    Arm(&logic, &input, 0xFFFFFFF0U);
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.operator_armed);
    input.remote.sequence = 0xFFFFFFFFU;
    Frame(&input, 1000U); AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.off_seen);
    Frame(&input, 2000U); AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.operator_armed);
}

static void CheckTargets(void)
{
    TargetManager targets;
    RemoteCommand command;
    uint32_t shots;
    test_stage = 3U; memset(&command, 0, sizeof(command));
    command.valid = 1U; command.fire_mode = FIRE_SINGLE; command.friction_on = 1U;
    command.trigger = RC_HIGH;
    TargetManager_Init(&targets);
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 0U);
    CHECK(!targets.request.shot_event); /* 已按住扳机时切入模式，不立即发射。 */
    command.trigger = RC_LOW;
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 7U);
    command.trigger = RC_HIGH;
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 14U);
    CHECK(targets.request.shot_event && targets.request.shot_sequence == 1U);
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 21U);
    CHECK(!targets.request.shot_event && targets.request.shot_sequence == 1U);
    TargetManager_Step(&targets, &command, 1U, 1U, 0U, 28U);
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 35U);
    CHECK(!targets.request.shot_event); /* 稳速许可恢复，不能补发旧请求。 */
    command.fire_mode = FIRE_CONTINUOUS;
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 42U); CHECK(!targets.request.shot_event);
    command.trigger = RC_LOW; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 49U);
    command.trigger = RC_HIGH; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 56U);
    CHECK(targets.request.shot_event);
    TargetManager_Step(&targets, &command, 0U, 1U, 1U, 105U); CHECK(!targets.request.shot_event);
    TargetManager_Step(&targets, &command, 0U, 1U, 1U, 106U); CHECK(targets.request.shot_event);
    shots = targets.request.shot_sequence;
    TargetManager_Step(&targets, &command, 0U, 1U, 1U, 1000U);
    CHECK(targets.request.shot_sequence == shots + 1U);
    TargetManager_Step(&targets, &command, 0U, 1U, 1U, 1002U); CHECK(!targets.request.shot_event);
    TargetManager_Step(&targets, &command, 0U, 0U, 1U, 1004U);
    CHECK(!targets.request.friction_requested && !targets.firing);
    Near(targets.request.flywheel_rpm, 0.0f);
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1010U); CHECK(!targets.request.shot_event);

    command.gimbal_mode = GIMBAL_STEP_YAW; command.step = RC_HIGH;
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1020U);
    CHECK(!targets.request.step_event && !targets.request.friction_requested);
    command.step = RC_MIDDLE; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1027U);
    command.step = RC_HIGH; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1034U);
    CHECK(targets.request.step_event); Near(targets.request.step_delta_deg, 30.0f);
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1041U); CHECK(!targets.request.step_event);
    command.step = RC_LOW; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1048U);
    CHECK(!targets.request.step_event); /* 必须回中，不能直接+30切-30。 */
    command.step = RC_MIDDLE; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1055U);
    Near(targets.request.step_delta_deg, 0.0f);
    command.step = RC_LOW; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1062U);
    CHECK(targets.request.step_event); Near(targets.request.step_delta_deg, -30.0f);
    command.gimbal_mode = GIMBAL_STEP_PITCH;
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1064U);
    CHECK(!targets.request.step_event); Near(targets.request.step_delta_deg, 0.0f);
    command.step = RC_MIDDLE; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1065U);
    command.step = RC_HIGH; TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1066U);
    CHECK(targets.request.step_event); Near(targets.request.step_delta_deg, 30.0f);
    command.gimbal_mode = GIMBAL_MANUAL;
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 1067U);
    CHECK(!targets.request.step_event); Near(targets.request.step_delta_deg, 0.0f);
    TargetManager_Step(&targets, &command, 1U, 0U, 1U, 1070U);
    CHECK(!targets.request.step_event); Near(targets.request.step_delta_deg, 0.0f);

    TargetManager_Init(&targets); command.gimbal_mode = GIMBAL_MANUAL;
    command.fire_mode = FIRE_CONTINUOUS; command.trigger = RC_LOW;
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 0xFFFFFFD0U);
    command.trigger = RC_HIGH;
    TargetManager_Step(&targets, &command, 1U, 1U, 1U, 0xFFFFFFD7U);
    CHECK(targets.request.shot_event);
    TargetManager_Step(&targets, &command, 0U, 1U, 1U, 8U); CHECK(!targets.request.shot_event);
    TargetManager_Step(&targets, &command, 0U, 1U, 1U, 9U); CHECK(targets.request.shot_event);
}

static void CheckBackendAndFrames(void)
{
    CanFrame frame = {0};
    MotorHealth health;
    TargetRequest request = {0};
    test_stage = 4U;
    memset(&health, 0xFF, sizeof(health)); MotorBackend_GetHealth(&health);
    CHECK(!health.ready && !health.feedback_ready && !health.flywheels_ready);
    request.flywheel_rpm = 6000.0f; request.shot_event = 1U;
    CHECK(!MotorBackend_Apply(&request, 1U));
    CHECK(!MotorBackend_Apply(&request, 0U));
    CHECK(!MotorBackend_OnFrame(&frame));
    CHECK(!CanFrame_IsValid(&frame));
    frame.bus = 1U; frame.id = 0x7FFU; frame.dlc = 8U; CHECK(CanFrame_IsValid(&frame));
    frame.id++; CHECK(!CanFrame_IsValid(&frame));
    frame.extended = 1U; frame.id = 0x1FFFFFFFU; CHECK(CanFrame_IsValid(&frame));
    frame.id++; CHECK(!CanFrame_IsValid(&frame));
    frame.id = 0x946U; frame.bus = 2U; frame.dlc = 4U; CHECK(CanFrame_IsValid(&frame));
    frame.dlc = 9U; CHECK(!CanFrame_IsValid(&frame));
    frame.dlc = 4U; frame.extended = 2U; CHECK(!CanFrame_IsValid(&frame));
}

void test_run(void)
{
    CheckMapper(); CheckSafety(); CheckTargets(); CheckBackendAndFrames();
    test_done = 1U;
}
