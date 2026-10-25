#include "remote_mapper.h"
#include "app_config.h"
#include <string.h>
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

void test_run(void)
{
    RemoteFrame frame;
    RemoteCommand command;
    uint32_t i;
    const uint8_t channels[] = {0U, 4U, 5U};
    test_stage = 1U;
    CHECK(APP_REMOTE_PROFILE_LEGACY == 1U);
    memset(&frame, 0, sizeof(frame));
    frame.has_frame = 1U;
    frame.channels[0] = 1500U; frame.channels[4] = 1000U; frame.channels[5] = 2000U;
    /* 未设置CH7..14时，无效或极端辅助值不能影响旧配置的保护。 */
    RemoteMapper_Map(&frame, &command);
    CHECK(command.valid && command.neutral && command.failsafe_ok && command.arm == RC_LOW);
    CHECK(command.pitch_rate_dps == 0.0f && !command.friction_on && command.trigger == RC_LOW);
    CHECK(command.fire_mode == FIRE_SAFE && command.gimbal_mode == GIMBAL_MANUAL && command.step == RC_MIDDLE);
    for (i = 0U; i < 14U; ++i) {
        uint16_t saved = frame.channels[i];
        if (i == 0U || i == 4U || i == 5U) { continue; }
        frame.channels[i] = 65535U;
        RemoteMapper_Map(&frame, &command); CHECK(command.valid && command.neutral);
        frame.channels[i] = saved;
    }
    for (i = 0U; i < sizeof(channels); ++i) {
        uint16_t saved = frame.channels[channels[i]];
        frame.channels[channels[i]] = 799U;
        RemoteMapper_Map(&frame, &command); CHECK(!command.valid);
        frame.channels[channels[i]] = 2201U;
        RemoteMapper_Map(&frame, &command); CHECK(!command.valid);
        frame.channels[channels[i]] = saved;
    }
    frame.channels[0] = 2000U; frame.channels[4] = 2000U;
    RemoteMapper_Map(&frame, &command);
    CHECK(command.valid && !command.neutral && command.arm == RC_HIGH);
    CHECK(command.yaw_rate_dps == APP_YAW_RATE_LIMIT_DPS && command.pitch_rate_dps == 0.0f);
    frame.channels[5] = 1000U;
    RemoteMapper_Map(&frame, &command); CHECK(!command.failsafe_ok);
    frame.has_frame = 0U;
    RemoteMapper_Map(&frame, &command); CHECK(!command.valid);
    test_done = 1U;
}
