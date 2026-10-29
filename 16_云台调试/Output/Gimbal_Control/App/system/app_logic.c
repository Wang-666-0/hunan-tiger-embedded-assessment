/**
 * @file app_logic.c
 * 作用：将遥控通道转换为业务命令，统一判定运行许可并生成电机目标请求。
 * 对外函数：
 *   AppLogic_Step / AppLogic_CombineActuatorHealth / AppLogic_Init
 * 内部函数：
 *   无。
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "app_logic.h"
#include "app_config.h"
#include <string.h>

void AppLogic_Step(AppLogic *logic, const SafetyInput *input, uint8_t flywheels_ready)
/*
 * 作用：把遥控原始值转换为操作命令，推进安全状态机，再生成各电机的本轮目标请求。
 * 参数 logic：应用状态，包含遥控命令、安全状态机及目标管理器。
 * 参数 input：本轮遥控、设备健康、时间及故障代次快照。
 * 参数 flywheels_ready：双摩擦轮是否满足稳速条件。
 * 流程：先判断是否出现新的遥控帧，边沿动作只消费一次。
 * 流程：安全状态机决定是否已解锁；电机后端还会根据输出许可和设备状态决定能否执行。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint8_t new_frame = input->remote.has_frame && (!logic->safety.have_sequence ||
        logic->safety.last_sequence != input->remote.sequence);
    RemoteMapper_Map(&input->remote, &logic->command);
    SafetyManager_Step(&logic->safety, input, &logic->command);
    TargetManager_Step(&logic->targets, &logic->command, new_frame,
        logic->safety.operator_armed, flywheels_ready && logic->safety.output_allowed,
        input->now_ms);
}

void AppLogic_CombineActuatorHealth(SafetyInput *input, const AppAxisHealth *yaw,
                                   const AppAxisHealth *pitch, uint8_t wheels_wanted)
/*
 * 作用：组合已请求云台轴与必要轮子的健康状态，形成统一运行许可输入。
 * 参数 input：本轮遥控、设备健康、时间及故障代次快照。
 * 参数 yaw：Yaw 控制器实例。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 wheels_wanted：是否要求摩擦轮健康参与整机就绪判定；独立拨盘模式会忽略该要求。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint8_t requested = 0U, healthy = 1U, faulted = 0U;
    const AppAxisHealth *axes[2] = { yaw, pitch };
    uint32_t i;
    if (!input) { return; }
#if APP_FEEDER_STANDALONE_TEST_ENABLED
    /* 独立拨盘测试时摩擦轮不参与动力许可；已请求的云台轴仍逐项检查。
     * 不把缺失电调的在线/稳速状态伪装为正常。 */
    wheels_wanted = 0U;
#endif
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
/*
 * 作用：初始化应用、安全管理和目标节拍状态。
 * 参数 logic：应用状态，包含遥控命令、安全状态机及目标管理器。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (logic == 0) { return; }
    memset(logic, 0, sizeof(*logic));
    SafetyManager_Init(&logic->safety);
    TargetManager_Init(&logic->targets);
}
