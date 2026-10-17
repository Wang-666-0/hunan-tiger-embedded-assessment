#include "app_logic.h"
#include <string.h>
volatile uint32_t test_checks, test_failures, test_failed_line, test_stage, test_done;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) test_failed_line = __LINE__; } } while (0)
static SafetyInput input;
static AppLogic logic;
static void frame(uint32_t now, uint16_t ch5)
{
    uint32_t i;
    for (i = 0U; i < 14U; ++i) { input.remote.channels[i] = 1500U; }
    input.remote.channels[4] = ch5; input.remote.channels[5] = 1000U;
    input.remote.received_ms = input.now_ms = now; ++input.remote.sequence;
    input.remote.has_frame = 1U;
}
void test_run(void)
{
    AppAxisHealth yaw = {0U, 0U, 1U, 1U}, pitch = {1U, 1U, 0U, 0U};
    test_stage = 1U;
    memset(&input, 0, sizeof(input)); AppLogic_Init(&logic);
    /* Pitch only: absent/unprepared Yaw and powered-off wheels do not fake readiness
     * and do not prevent real Pitch feedback from participating in remote unlock. */
    input.actuator_fault = 1U;
    AppLogic_CombineActuatorHealth(&input, &yaw, &pitch, 0U);
    CHECK(input.backend_ready && input.feedback_ready && !input.actuator_fault);
    frame(10U, 1000U); AppLogic_Step(&logic, &input, 0U);
    CHECK(!logic.safety.operator_armed);
    frame(20U, 1500U); AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.operator_armed && logic.safety.output_allowed);
    test_stage = 2U;
    /* A shared running Yaw must actually be healthy. */
    yaw.requested = 1U; AppLogic_CombineActuatorHealth(&input, &yaw, &pitch, 0U);
    CHECK(!input.feedback_ready && input.actuator_fault);
    frame(30U, 1500U); AppLogic_Step(&logic, &input, 0U);
    CHECK(!logic.safety.output_allowed && !logic.safety.operator_armed);
    CHECK(logic.safety.reason == SAFETY_ACTUATOR);
    test_stage = 3U;
    yaw.requested = 0U; pitch.stop_pending = 1U;
    input.backend_ready = input.feedback_ready = 0U;
    AppLogic_CombineActuatorHealth(&input, &yaw, &pitch, 0U);
    CHECK(!input.feedback_ready && !input.actuator_fault);
    pitch.stop_pending = 0U;
    AppLogic_CombineActuatorHealth(&input, &yaw, &pitch, 0U);
    frame(40U, 1500U); AppLogic_Step(&logic, &input, 0U);
    CHECK(!logic.safety.operator_armed); /* recovering alone cannot self-rearm */
    frame(50U, 1000U); AppLogic_Step(&logic, &input, 0U);
    frame(60U, 2000U); AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.operator_armed && logic.safety.output_allowed);
    input.remote.channels[7] = 1000U; input.remote.sequence++; input.remote.received_ms = input.now_ms = 70U;
    AppLogic_Step(&logic, &input, 0U);
    CHECK(!logic.safety.output_allowed && logic.safety.reason == SAFETY_FAILSAFE);
    test_stage = 4U;
    /* Joint wheel operation requires the existing real wheel health. */
    input.backend_ready = input.feedback_ready = 0U; input.actuator_fault = 1U;
    AppLogic_CombineActuatorHealth(&input, &yaw, &pitch, 1U);
    CHECK(input.backend_ready && !input.feedback_ready && input.actuator_fault);
    input.backend_ready = input.feedback_ready = 1U; input.actuator_fault = 0U;
    AppLogic_CombineActuatorHealth(&input, &yaw, &pitch, 1U);
    CHECK(input.feedback_ready && !input.actuator_fault);
    test_done = 1U;
}
