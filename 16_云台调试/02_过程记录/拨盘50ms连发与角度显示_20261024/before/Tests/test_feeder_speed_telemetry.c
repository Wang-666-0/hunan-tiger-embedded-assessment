#define test_run feeder_speed_backend_baseline_test_run
#include "test_feeder_backend.c"
#undef test_run
#include <stdlib.h>
void test_run(void)
{
    char *cursor, *end;
    float value[9];
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
    test_done = 1U;
}
