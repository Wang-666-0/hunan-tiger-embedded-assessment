#include "app_logic.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static AppLogic logic;
static SafetyInput input;
static uint32_t now = 100U;
static void Frame(void)
{
    now += 7U;
    input.now_ms = input.remote.received_ms = now;
    input.remote.sequence++;
    AppLogic_Step(&logic, &input, 0U);
}
static void Arm(void)
{
    uint32_t i;
    memset(&input, 0, sizeof(input)); AppLogic_Init(&logic);
    for (i = 0U; i < 14U; ++i) { input.remote.channels[i] = 1500U; }
    input.remote.has_frame = input.backend_ready = input.feedback_ready = 1U;
    input.remote.channels[4] = input.remote.channels[5] = 1000U;
    Frame(); input.remote.channels[4] = 2000U; Frame();
    CHECK(logic.safety.operator_armed && logic.safety.output_allowed);
    CHECK(logic.targets.request.gimbal_mode == GIMBAL_TEST);
}
static void Release(void) { input.remote.channels[6] = 1500U; Frame(); }
static void Return(void) { input.remote.channels[6] = 1000U; Frame(); }
static void Confirm(void) { input.remote.channels[6] = 2000U; Frame(); }

void test_run(void)
{
    uint32_t sequence;
    test_stage = 1U;
    CHECK(APP_PITCH_ENABLED && APP_REMOTE_PROFILE_ASSESSMENT);
    Arm();
    CHECK(logic.targets.last_test_axis == GIMBAL_STEP_YAW && logic.command.step_selection_centered);
    Return();
    CHECK(logic.targets.request.step_return_event && logic.targets.request.step_axis == GIMBAL_STEP_YAW);
    sequence = logic.targets.request.step_sequence;
    Frame(); CHECK(!logic.targets.request.step_event && logic.targets.request.step_sequence == sequence);
    Release(); input.remote.channels[2] = 2000U; Confirm();
    CHECK(logic.targets.request.step_event && !logic.targets.request.step_return_event);
    CHECK(logic.targets.request.step_axis == GIMBAL_STEP_PITCH && logic.targets.request.step_delta_deg == 30.0f);
    CHECK(!logic.targets.request.friction_requested && !logic.targets.request.shot_event);
    /* 发布合法请求仍不表示后端已接受；模拟拒绝，不记录axis。 */
    CHECK(logic.targets.last_test_axis == GIMBAL_STEP_YAW);
    input.remote.channels[2] = 1500U; Release(); Return();
    CHECK(logic.targets.request.step_return_event && logic.targets.request.step_axis == GIMBAL_STEP_YAW);
    Release(); input.remote.channels[2] = 1000U; Confirm();
    CHECK(logic.targets.request.step_event && logic.targets.request.step_delta_deg == -30.0f);
    TargetManager_RecordAcceptedTestAxis(&logic.targets, GIMBAL_STEP_PITCH);
    CHECK(logic.targets.last_test_axis == GIMBAL_STEP_PITCH);
    sequence = logic.targets.request.step_sequence;
    input.remote.channels[2] = 1500U; Frame();
    CHECK(!logic.targets.request.step_event && logic.targets.request.step_sequence == sequence);
    Release(); Return();
    CHECK(logic.targets.request.step_return_event && logic.targets.request.step_axis == GIMBAL_STEP_PITCH);

    test_stage = 2U;
    Release(); input.remote.channels[3] = 2000U; Return();
    CHECK(logic.targets.request.step_return_event && logic.targets.request.step_axis == GIMBAL_STEP_YAW);
    CHECK(logic.targets.last_test_axis == GIMBAL_STEP_PITCH); /* 返回不伪造新阶跃接受。 */
    Release(); input.remote.channels[2] = 2000U;
    sequence = logic.targets.request.step_sequence; Return();
    CHECK(!logic.command.step_valid && !logic.command.step_selection_centered);
    CHECK(!logic.targets.request.step_event && !logic.targets.request.step_return_event);
    CHECK(logic.targets.request.step_sequence == sequence);
    input.remote.channels[2] = input.remote.channels[3] = 1500U; Frame();
    CHECK(!logic.targets.request.step_event && logic.targets.request.step_sequence == sequence);
    Release(); Return();
    CHECK(logic.targets.request.step_return_event && logic.targets.request.step_axis == GIMBAL_STEP_PITCH);
    Release(); input.remote.channels[2] = input.remote.channels[3] = 2000U;
    sequence = logic.targets.request.step_sequence; Confirm();
    CHECK(!logic.targets.request.step_event && logic.targets.request.step_sequence == sequence);
    input.remote.channels[3] = 1500U; Frame();
    CHECK(!logic.targets.request.step_event && logic.targets.request.step_sequence == sequence);
    Release(); Confirm(); CHECK(logic.targets.request.step_axis == GIMBAL_STEP_PITCH);
    CHECK(logic.targets.request.step_event && logic.targets.request.step_sequence == sequence + 1U);
    TargetManager_RecordAcceptedTestAxis(&logic.targets, GIMBAL_TEST);
    CHECK(logic.targets.last_test_axis == GIMBAL_STEP_PITCH);
    TargetManager_RecordAcceptedTestAxis(0, GIMBAL_STEP_YAW);

    test_stage = 3U;
    input.remote.channels[7] = 1000U; Frame();
    CHECK(!logic.safety.operator_armed && !logic.targets.request.step_event);
    CHECK(logic.targets.last_test_axis == GIMBAL_STEP_PITCH);
    input.remote.channels[7] = 1500U; input.remote.channels[4] = 1000U;
    input.remote.channels[2] = 1500U; input.remote.channels[6] = 1500U; Frame();
    input.remote.channels[4] = 2000U; Frame();
    CHECK(logic.safety.operator_armed);
    Return(); CHECK(logic.targets.request.step_return_event && logic.targets.request.step_axis == GIMBAL_STEP_PITCH);
    Release(); input.remote.channels[4] = 1500U; Frame();
    input.remote.channels[1] = 2000U; Frame();
    CHECK(logic.targets.request.pitch_rate_dps == APP_PITCH_RATE_LIMIT_DPS);
    CHECK(!logic.targets.request.step_event && !logic.targets.request.shot_event);
    input.remote.channels[1] = 1000U; Frame();
    CHECK(logic.targets.request.pitch_rate_dps == -APP_PITCH_RATE_LIMIT_DPS);
    test_done = 1U;
}
