#include "app_logic.h"
#include <string.h>
void AppLogic_Init(AppLogic *logic)
{
    if (logic == 0) { return; }
    memset(logic, 0, sizeof(*logic));
    SafetyManager_Init(&logic->safety);
    TargetManager_Init(&logic->targets);
}
void AppLogic_Step(AppLogic *logic, const SafetyInput *input, uint8_t flywheels_ready)
{
    uint8_t new_frame;
    if (logic == 0 || input == 0) { return; }
    new_frame = input->remote.has_frame && (!logic->safety.have_sequence ||
        logic->safety.last_sequence != input->remote.sequence);
    RemoteMapper_Map(&input->remote, &logic->command);
    SafetyManager_Step(&logic->safety, input, &logic->command);
    TargetManager_Step(&logic->targets, &logic->command, new_frame,
        logic->safety.operator_armed, flywheels_ready && logic->safety.output_allowed,
        input->now_ms);
}
