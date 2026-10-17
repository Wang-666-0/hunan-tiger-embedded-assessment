#include "app_logic.h"
#include <string.h>
void AppLogic_CombineActuatorHealth(SafetyInput *input, const AppAxisHealth *yaw,
                                   const AppAxisHealth *pitch, uint8_t wheels_wanted)
{
    uint8_t requested = 0U, healthy = 1U, faulted = 0U;
    const AppAxisHealth *axes[2] = { yaw, pitch };
    uint32_t i;
    if (!input) { return; }
    for (i = 0U; i < 2U; ++i) {
        const AppAxisHealth *axis = axes[i];
        if (!axis || !axis->requested) { continue; }
        requested = 1U;
        healthy = (uint8_t)(healthy && axis->online && !axis->stop_pending && !axis->faulted);
        faulted = (uint8_t)(faulted || axis->faulted);
    }
    if (requested) {
        input->feedback_ready = (uint8_t)(healthy &&
            (!wheels_wanted || (input->backend_ready && input->feedback_ready)));
        input->actuator_fault = (uint8_t)(faulted || (wheels_wanted && input->actuator_fault));
        input->backend_ready = 1U;
    }
}
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
