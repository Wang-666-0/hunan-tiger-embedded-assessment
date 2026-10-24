#define test_run feeder_speed_backend_baseline_test_run
#include "test_feeder_backend.c"
#undef test_run
#include <stdlib.h>
void test_run(void)
{
    char *cursor, *end;
    float value[9];
    float angle[39];
    FeederBackendState before, after;
    unsigned i;
    test_stage = 1U;
    Reset(10U);
    rotor_rpm = -3568; Feed(11U); now_ms += 5U; Read();
    FeederConsole_SpeedTelemetry(response, sizeof(response));
    CHECK(strncmp(response, "feeder_speed:", 13U) == 0); cursor = response + 13U;
    for (i = 0U; i < 9U; ++i) {
        value[i] = strtof(cursor, &end); CHECK(end != cursor);
        if (i < 8U) { CHECK(*end == ','); cursor = end + 1U; } else { CHECK(strcmp(end, "\r\n") == 0); }
    }
    CHECK(value[0] == -3568 && value[1] == 0 && value[2] == -100 && value[3] == 0);
    CHECK(value[4] == 35.68f && value[5] == 1 && value[6] == 0 && value[7] == 0 && value[8] == 5);
    CHECK(state.controller.raw_rpm == -3568 && !nonzero_calls);
    test_stage = 2U;
    StartNormal(100U, 2000U, 1U);
    rotor_counts = 227; rotor_rpm = 600; Tick(now_ms + 1U);
    before = state;
    FeederConsole_Telemetry(response, sizeof(response));
    CHECK(strncmp(response, "feeder:", 7U) == 0); cursor = response + 7U;
    for (i = 0U; i < 39U; ++i) {
        angle[i] = strtof(cursor, &end); CHECK(end != cursor);
        if (i < 38U) { CHECK(*end == ','); cursor = end + 1U; }
        else { CHECK(strcmp(end, "\r\n") == 0); }
    }
    CHECK(angle[0] > 9.97f && angle[0] < 9.98f && angle[1] == 40.0f);
    CHECK(angle[2] == 600.0f && angle[18] == 1 && angle[21] == 1);
    Read(); after = state; CHECK(!memcmp(&before, &after, sizeof(before)));
    test_stage = 3U;
    Reset(1000U); Feed(1001U); Feed(1002U); Feed(1003U);
    CHECK(FeederBackend_Zero());
    rotor_counts = -227; rotor_rpm = -600; Feed(1004U);
    FeederConsole_Telemetry(response, sizeof(response));
    angle[0] = strtof(response + 7U, &end); CHECK(*end == ',');
    angle[1] = strtof(end + 1U, &end);
    CHECK(angle[0] < -0.27f && angle[0] > -0.29f && angle[1] == 0.0f);
    CHECK(state.controller.config.direction == -1 && state.controller.config.gear_ratio == 35.68f);
    test_done = 1U;
}
