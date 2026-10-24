#include "target_manager.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static TargetManager target;
static RemoteCommand command;

static void step(uint32_t now, uint8_t ready)
{ TargetManager_Step(&target, &command, 1U, 1U, ready, now); }

static void start(uint32_t now)
{
    TargetManager_Init(&target); memset(&command, 0, sizeof(command));
    command.valid = command.failsafe_ok = 1U;
    command.gimbal_mode = GIMBAL_MANUAL; command.trigger = RC_LOW;
    command.wheel_switch = RC_LOW; command.fire_mode = FIRE_SAFE;
    step(now - 4U, 1U);
    command.wheel_switch = RC_HIGH; command.fire_mode = FIRE_CONTINUOUS;
    command.friction_on = 1U; step(now - 2U, 1U);
    command.trigger = RC_HIGH; step(now, 1U);
    CHECK(target.request.shot_event && target.request.shot_sequence == 1U);
    CHECK(target.request.continuous_fire && target.next_shot_ms == now + 50U);
}

static void jitter(uint32_t base)
{
    uint32_t i, due, late;
    start(base);
    for (i = 1U; i <= 200U; ++i) {
        due = base + i * APP_FIRE_INTERVAL_MS;
        step(due - 1U, 1U); CHECK(!target.request.shot_event);
        late = i % (APP_CONTROL_PERIOD_MS + 1U);
        step(due + late, 1U);
        CHECK(target.request.shot_event && target.request.shot_sequence == i + 1U);
        CHECK(target.next_shot_ms == due + APP_FIRE_INTERVAL_MS);
        step(due + late, 1U); CHECK(!target.request.shot_event);
    }
}

void test_run(void)
{
    uint32_t i, count, now;
    test_stage = 1U; CHECK(APP_FIRE_INTERVAL_MS == 50U && APP_CONTROL_PERIOD_MS == 2U);
    jitter(100U); jitter(0xFFFFFF00U);
    test_stage = 2U;
    /* Outside ordinary 2ms jitter, re-anchor instead of following up a late
     * pulse with a second pulse a few milliseconds later. */
    for (i = 3U; i <= 149U; ++i) {
        start(100U); now = 150U + i; step(now, 1U);
        CHECK(target.request.shot_event && target.request.shot_sequence == 2U);
        CHECK(target.next_shot_ms == now + 50U);
        step(now + 1U, 1U); CHECK(!target.request.shot_event);
        step(now + 49U, 1U); CHECK(!target.request.shot_event);
        step(now + 50U, 1U); CHECK(target.request.shot_event && target.request.shot_sequence == 3U);
    }
    test_stage = 3U;
    start(100U); count = target.request.shot_sequence;
    step(149U, 0U); CHECK(!target.firing && !target.request.continuous_fire);
    step(150U, 1U); step(500U, 1U);
    CHECK(!target.request.shot_event && target.request.shot_sequence == count);
    command.trigger = RC_LOW; step(502U, 1U);
    command.trigger = RC_HIGH; step(504U, 1U);
    CHECK(target.request.shot_event && target.next_shot_ms == 554U);
    command.trigger = RC_LOW; step(552U, 1U); step(554U, 1U);
    CHECK(!target.request.shot_event && !target.firing && !target.request.continuous_fire);
    test_done = 1U;
}
