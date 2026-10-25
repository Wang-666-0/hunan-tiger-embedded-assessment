#include "app_logic.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static AppLogic logic;
static SafetyInput input;

static void frame(uint32_t now)
{
    input.now_ms = input.remote.received_ms = now;
    input.remote.sequence++;
    AppLogic_Step(&logic, &input, 0U); /* 当前实际拨盘许可保持关闭。 */
}
static void reset(uint32_t now)
{
    uint32_t i;
    memset(&input, 0, sizeof(input)); AppLogic_Init(&logic);
    for (i = 0U; i < 14U; ++i) { input.remote.channels[i] = 1500U; }
    input.remote.has_frame = 1U;
    input.remote.channels[4] = input.remote.channels[5] = 1000U;
    input.backend_ready = input.feedback_ready = 1U;
    frame(now);
}
static void arm(uint32_t now)
{
    reset(now); input.remote.channels[4] = 1500U; frame(now + 7U);
    CHECK(logic.safety.operator_armed && logic.safety.output_allowed);
    CHECK(!logic.targets.request.friction_requested);
}
static void mapper_and_safety(void)
{
    RemoteCommand command;
    uint32_t i;
    test_stage = 1U;
    reset(100U);
    CHECK(APP_REMOTE_PROFILE_ASSESSMENT && !APP_REMOTE_PROFILE_LEGACY);
    CHECK(RC_FAILSAFE_CHANNEL == 7U && RC_TRIGGER_CHANNEL == 7U);
    CHECK(logic.command.neutral && logic.command.failsafe_ok);
    for (i = 0U; i < 8U; ++i) {
        input.remote.channels[i] = 799U;
        RemoteMapper_Map(&input.remote, &command); CHECK(!command.valid);
        input.remote.channels[i] = 2201U;
        RemoteMapper_Map(&input.remote, &command); CHECK(!command.valid);
        input.remote.channels[i] = i == 4U || i == 5U ? 1000U : 1500U;
    }
    input.remote.channels[8] = input.remote.channels[9] = 0U;
    RemoteMapper_Map(&input.remote, &command); CHECK(command.valid);
    input.remote.channels[8] = input.remote.channels[9] = 1500U;
    input.remote.channels[7] = 1300U; frame(110U);
    CHECK(logic.safety.reason == SAFETY_FAILSAFE && !logic.safety.off_seen);
    input.remote.channels[7] = 1301U;
    input.remote.channels[4] = 1500U; frame(117U);
    CHECK(!logic.safety.operator_armed); /* RF恢复不能把预设OFF当作人工解锁。 */
    input.remote.channels[4] = 1000U; frame(124U);
    input.remote.channels[4] = 1500U; frame(131U);
    CHECK(logic.safety.operator_armed);
    input.remote.channels[7] = 1000U; frame(138U);
    CHECK(!logic.safety.operator_armed && !logic.safety.output_allowed);
    input.remote.channels[4] = input.remote.channels[5] = 1000U; frame(145U);
    CHECK(!logic.safety.off_seen);
    input.remote.channels[7] = 1500U; input.remote.channels[4] = 1500U; frame(152U);
    CHECK(!logic.safety.operator_armed);
    arm(200U);
    input.now_ms = input.remote.received_ms + 99U;
    AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.operator_armed);
    input.now_ms++; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_REMOTE && !logic.safety.operator_armed);
    reset(400U); input.remote.channels[5] = 1500U;
    input.remote.channels[4] = 1500U; frame(407U);
    CHECK(logic.safety.reason == SAFETY_NOT_NEUTRAL);
    reset(500U); input.remote.channels[7] = 2000U;
    input.remote.channels[4] = 1500U; frame(507U);
    CHECK(logic.safety.reason == SAFETY_NOT_NEUTRAL);
    reset(600U); input.remote.channels[2] = 1600U;
    input.remote.channels[4] = 1500U; frame(607U);
    CHECK(logic.safety.reason == SAFETY_NOT_NEUTRAL);
    arm(700U); input.stop_sequence++;
    AppLogic_Step(&logic, &input, 0U); CHECK(logic.safety.reason == SAFETY_STOP);
    input.remote.channels[4] = 1000U; input.remote.sequence++;
    AppLogic_Step(&logic, &input, 0U); CHECK(!logic.safety.off_seen);
    frame(714U); input.remote.channels[4] = 1500U; frame(721U);
    CHECK(logic.safety.operator_armed);
    input.uart_epoch++; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_UART && !logic.safety.operator_armed);
    arm(800U); input.can_epoch++; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_CAN);
    arm(900U); input.control_late = 1U; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_TIMING);
    arm(1000U); input.actuator_fault = 1U; AppLogic_Step(&logic, &input, 0U);
    CHECK(logic.safety.reason == SAFETY_ACTUATOR);
    arm(0xFFFFFFF0U); input.remote.channels[5] = 1500U; frame(5U);
    CHECK(logic.targets.request.friction_requested);
}
static void wheels_and_test_interfaces(void)
{
    uint32_t sequence;
    test_stage = 2U;
    arm(100U); input.remote.channels[0] = 2000U; frame(114U);
    CHECK(logic.targets.request.yaw_rate_dps > 0.0f && !logic.targets.request.friction_requested);
    input.remote.channels[0] = 1500U; input.remote.channels[5] = 1500U; frame(121U);
    CHECK(logic.targets.request.friction_requested);
    CHECK(logic.targets.request.flywheel_rpm == 6000.0f && logic.targets.request.fire_mode == FIRE_SINGLE);
    input.remote.channels[0] = 1000U; input.remote.channels[5] = 2000U; frame(128U);
    CHECK(logic.targets.request.friction_requested && logic.targets.request.fire_mode == FIRE_CONTINUOUS);
    input.remote.channels[7] = 2000U; frame(135U);
    CHECK(!logic.targets.request.shot_event && logic.targets.request.shot_sequence == 0U);
    input.remote.channels[7] = 1500U; input.remote.channels[4] = 2000U; frame(142U);
    CHECK(logic.targets.request.gimbal_mode == GIMBAL_TEST && !logic.targets.request.friction_requested);
    input.remote.channels[3] = 2000U; input.remote.channels[6] = 2000U; frame(149U);
    CHECK(logic.targets.request.step_event && logic.targets.request.step_axis == GIMBAL_STEP_YAW);
    CHECK(logic.targets.request.step_delta_deg == 30.0f);
    sequence = logic.targets.request.step_sequence;
    frame(156U); CHECK(!logic.targets.request.step_event && logic.targets.request.step_sequence == sequence);
    input.remote.channels[6] = 1500U; frame(163U);
    CHECK(logic.targets.request.step_delta_deg == 30.0f); /* 按钮释放不取消已提交阶跃。 */
    input.remote.channels[3] = 1000U; input.remote.channels[6] = 2000U; frame(170U);
    CHECK(logic.targets.request.step_event && logic.targets.request.step_delta_deg == -30.0f);
    input.remote.channels[6] = 1500U; input.remote.channels[3] = 1500U; frame(177U);
    input.remote.channels[2] = 2000U; input.remote.channels[6] = 2000U; frame(184U);
    CHECK(logic.targets.request.step_event && logic.targets.request.step_axis == GIMBAL_STEP_PITCH);
    input.remote.channels[6] = 1500U; frame(191U);
    input.remote.channels[3] = 2000U; input.remote.channels[6] = 2000U; frame(198U);
    CHECK(!logic.targets.request.step_event); /* 左摇杆斜向拒绝，不能猜测测试轴。 */
    input.remote.channels[6] = 1000U; frame(205U);
    CHECK(logic.targets.request.step_return_event && logic.targets.request.step_delta_deg == -30.0f);
    sequence = logic.targets.request.step_sequence;
    frame(212U); CHECK(!logic.targets.request.step_return_event && logic.targets.request.step_sequence == sequence);
    input.remote.channels[4] = 1500U; frame(219U);
    CHECK(!logic.targets.request.friction_requested); /* 离开测试不恢复旧CH6 ON。 */
    input.remote.channels[5] = 1000U; frame(226U);
    input.remote.channels[5] = 1500U; frame(233U);
    CHECK(logic.targets.request.friction_requested);
    input.remote.channels[5] = 1000U; frame(240U);
    CHECK(logic.safety.operator_armed && !logic.targets.request.friction_requested);
}
static void fire_interfaces(void)
{
    TargetManager target;
    RemoteCommand command;
    uint32_t sequence;
    test_stage = 3U;
    reset(100U); input.remote.channels[5] = 1500U;
    RemoteMapper_Map(&input.remote, &command); TargetManager_Init(&target);
    command.wheel_switch = RC_LOW; command.friction_on = 0U; command.fire_mode = FIRE_SAFE;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 100U);
    command.wheel_switch = RC_MIDDLE; command.friction_on = 1U; command.fire_mode = FIRE_SINGLE;
    command.trigger = RC_HIGH;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 107U); CHECK(!target.request.shot_event);
    command.trigger = RC_LOW;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 114U);
    command.trigger = RC_HIGH;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 121U); CHECK(target.request.shot_event);
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 128U); CHECK(!target.request.shot_event);
    command.fire_mode = FIRE_CONTINUOUS;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 135U); CHECK(!target.request.shot_event);
    command.trigger = RC_LOW;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 142U);
    command.trigger = RC_HIGH;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 149U); CHECK(target.request.shot_event);
    TargetManager_Step(&target, &command, 0U, 1U, 1U, 198U); CHECK(!target.request.shot_event);
    TargetManager_Step(&target, &command, 0U, 1U, 1U, 199U); CHECK(target.request.shot_event);
    sequence = target.request.shot_sequence;
    TargetManager_Step(&target, &command, 0U, 1U, 1U, 1000U);
    CHECK(target.request.shot_sequence == sequence + 1U);
    TargetManager_Step(&target, &command, 1U, 1U, 0U, 1001U);
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 1002U); CHECK(!target.request.shot_event);
    command.trigger = RC_LOW;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 1009U); CHECK(!target.firing);
    command.failsafe_ok = 0U;
    TargetManager_Step(&target, &command, 1U, 1U, 1U, 1016U);
    CHECK(!target.request.friction_requested && !target.request.shot_event);
}
void test_run(void)
{
    mapper_and_safety(); wheels_and_test_interfaces(); fire_interfaces(); test_done = 1U;
}
