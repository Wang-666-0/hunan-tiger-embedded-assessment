/**
 * @file target_manager.c
 * 作用：手动/阶跃目标与 50 ms 发射节拍
 * 对外函数：
 *   TargetManager_Step / TargetManager_RecordAcceptedTestAxis / TargetManager_Init
 * 内部函数：
 *   AssessmentStep / RepeatShot / Cancel
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "target_manager.h"
#include "app_config.h"
#include <string.h>

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static void Cancel(TargetManager *manager);
static void RepeatShot(TargetManager *manager, uint32_t now_ms);
static void AssessmentStep(TargetManager *manager, const RemoteCommand *command,
    uint8_t new_frame, uint8_t operator_armed, uint8_t flywheels_ready, uint32_t now_ms);

void TargetManager_Step(TargetManager *manager, const RemoteCommand *command,
    uint8_t new_frame, uint8_t operator_armed, uint8_t flywheels_ready, uint32_t now_ms)
/*
 * 作用：校验输入指针并推进现用遥控目标和发射节拍状态机。
 * 参数 manager：调用方保存的安全状态机或目标管理状态。
 * 参数 command：通道映射后的操作命令。
 * 参数 new_frame：是否刚收到一帧新的遥控数据，边沿动作仅在新帧处理。
 * 参数 operator_armed：是否已完成操作员解锁。
 * 参数 flywheels_ready：双摩擦轮是否满足稳速条件。
 * 参数 now_ms：当前毫秒时间。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (manager == 0 || command == 0) { return; }
    AssessmentStep(manager, command, new_frame, operator_armed, flywheels_ready, now_ms);
}

static void AssessmentStep(TargetManager *manager, const RemoteCommand *command,
    uint8_t new_frame, uint8_t operator_armed, uint8_t flywheels_ready, uint32_t now_ms)
/*
 * 作用：按现用遥控模式生成手动、单次阶跃、单发或连发请求及边沿锁存。
 * 参数 manager：调用方保存的安全状态机或目标管理状态。
 * 参数 command：通道映射后的操作命令。
 * 参数 new_frame：是否刚收到一帧新的遥控数据，边沿动作仅在新帧处理。
 * 参数 operator_armed：是否已完成操作员解锁。
 * 参数 flywheels_ready：双摩擦轮是否满足稳速条件。
 * 参数 now_ms：当前毫秒时间。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    TargetRequest *request = &manager->request;
    uint8_t feed_requested;
    request->shot_event = request->step_event = request->step_return_event = request->continuous_fire = 0U;
    if (!operator_armed || !command->valid || !command->failsafe_ok) { Cancel(manager); return; }
    if (command->gimbal_mode != manager->last_gimbal_mode) {
        request->step_delta_deg = 0.0f;
        request->step_axis = GIMBAL_MANUAL;
        manager->step_center_seen = manager->trigger_low_seen = manager->firing = 0U;
        manager->wheel_off_seen = 0U;
        manager->last_step_switch = command->step;
        manager->last_gimbal_mode = command->gimbal_mode;
    }
    request->gimbal_mode = command->gimbal_mode;
    request->fire_mode = command->fire_mode;
    request->yaw_rate_dps = request->pitch_rate_dps = 0.0f;
    request->friction_requested = 0U;
    request->flywheel_rpm = 0.0f;
    if (command->gimbal_mode == GIMBAL_TEST) {
        manager->wheel_off_seen = manager->trigger_low_seen = manager->firing = 0U;
        if (new_frame) {
            if (command->step == RC_MIDDLE) { manager->step_center_seen = 1U; }
            else if (command->step == RC_LOW) {
                if (manager->last_step_switch != RC_LOW) {
                    /* Physical CH7 up (1000): one relative -30 request.
                     * Centered selection uses the last accepted axis. */
                    if (command->step_valid || command->step_selection_centered) {
                        request->step_return_event = request->step_event = 1U;
                        request->step_delta_deg = -APP_STEP_DEGREES;
                        request->step_axis = command->step_valid ? command->step_axis : manager->last_test_axis;
                        request->step_sequence++;
                    }
                }
                manager->step_center_seen = 0U;
            } else if (manager->step_center_seen) {
                if (command->step_valid) {
                    /* 未选轴时不消耗确认；保持CH7高位后选好单轴即可。
                     * 一旦发布即消耗，长按不会重复发起，下一次仍需新中位。 */
                    manager->step_center_seen = 0U;
                    request->step_axis = command->step_axis;
                    request->step_delta_deg = command->step_delta_deg;
                    request->step_event = 1U;
                    request->step_sequence++;
                } else if (!command->step_selection_centered) {
                    /* 斜向仍拒绝并消耗确认，改为单轴后须重新经过中位。 */
                    manager->step_center_seen = 0U;
                }
            }
            manager->last_step_switch = command->step;
        }
        return; /* 此模块只发布单次请求，由相应轴后端执行，测试禁止射击。 */
    }
    request->yaw_rate_dps = command->yaw_rate_dps;
    request->pitch_rate_dps = command->pitch_rate_dps;
    if (new_frame && command->wheel_switch == RC_LOW) { manager->wheel_off_seen = 1U; }
#if APP_FEEDER_STANDALONE_TEST_ENABLED
    /* CH6 中档单发/高档连发；摩擦轮始终为零请求。
     * 仍需本轮解锁后的 CH6 OFF，以及选好模式后的新鲜 CH8 释放/扣动。 */
    feed_requested = manager->wheel_off_seen && command->friction_on;
    (void)flywheels_ready;
#else
    request->friction_requested = manager->wheel_off_seen && command->friction_on;
    feed_requested = request->friction_requested && flywheels_ready;
#endif
    request->flywheel_rpm = request->friction_requested ? 6000.0f : 0.0f;
    if (command->fire_mode != manager->last_fire_mode) {
        manager->trigger_low_seen = manager->firing = 0U;
        manager->last_fire_mode = command->fire_mode;
    }
    if (!feed_requested || command->fire_mode == FIRE_SAFE) {
        manager->trigger_low_seen = manager->firing = 0U;
        return;
    }
    if (new_frame && command->trigger == RC_LOW) {
        manager->trigger_low_seen = 1U; manager->firing = 0U;
    } else if (new_frame && command->trigger == RC_HIGH && manager->trigger_low_seen) {
        request->shot_event = 1U; request->shot_sequence++;
        manager->trigger_low_seen = 0U;
        manager->firing = command->fire_mode == FIRE_CONTINUOUS;
        manager->next_shot_ms = now_ms + APP_FIRE_INTERVAL_MS;
    } else if (manager->firing && command->trigger == RC_HIGH &&
               (int32_t)(now_ms - manager->next_shot_ms) >= 0) {
        RepeatShot(manager, now_ms);
    }
    /* Observation of the existing release/trigger latch; no new shot source. */
    request->continuous_fire = manager->firing && command->trigger == RC_HIGH;
}

static void RepeatShot(TargetManager *manager, uint32_t now_ms)
/*
 * 作用：发布一次连发事件，并推进下一次 50 ms 发射节拍。
 * 参数 manager：调用方保存的安全状态机或目标管理状态。
 * 参数 now_ms：当前毫秒时间。
 * 流程：小幅迟到保留原节拍相位；明显迟到只发布一次事件并重新计时，不突发补发。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint32_t late_ms = now_ms - manager->next_shot_ms;
    manager->request.shot_event = 1U;
    manager->request.shot_sequence++;
    /* Keep the 50ms phase through normal one-control-tick jitter. A genuinely
     * late control call emits only one event and starts a new interval; it
     * must never squeeze missed requests into a catch-up burst. */
    manager->next_shot_ms = late_ms <= APP_CONTROL_PERIOD_MS ?
        manager->next_shot_ms + APP_FIRE_INTERVAL_MS : now_ms + APP_FIRE_INTERVAL_MS;
}

void TargetManager_RecordAcceptedTestAxis(TargetManager *manager, GimbalMode axis)
/*
 * 作用：只在轴后端接受阶跃后更新最近测试轴，供 CH7 回退使用。
 * 参数 manager：调用方保存的安全状态机或目标管理状态。
 * 参数 axis：本次真正接受测试的轴。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (manager != 0 && (axis == GIMBAL_STEP_YAW || axis == GIMBAL_STEP_PITCH)) {
        manager->last_test_axis = axis;
    }
}

void TargetManager_Init(TargetManager *manager)
/*
 * 作用：初始化目标管理器、发射节拍和最近一次测试轴。
 * 参数 manager：调用方保存的安全状态机或目标管理状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (manager != 0) {
        memset(manager, 0, sizeof(*manager));
        manager->last_test_axis = GIMBAL_STEP_YAW;
    }
}

static void Cancel(TargetManager *manager)
/*
 * 作用：清除当前运动与发射请求及触发锁存，保留已经发布的事件序号。
 * 参数 manager：调用方保存的安全状态机或目标管理状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint32_t shots = manager->request.shot_sequence, steps = manager->request.step_sequence;
    memset(&manager->request, 0, sizeof(manager->request));
    manager->request.shot_sequence = shots;
    manager->request.step_sequence = steps;
    manager->trigger_low_seen = 0U;
    manager->step_center_seen = 0U;
    manager->firing = 0U;
    manager->last_fire_mode = FIRE_SAFE;
    manager->last_gimbal_mode = GIMBAL_MANUAL;
    manager->wheel_off_seen = 0U;
    manager->last_step_switch = RC_MIDDLE;
}
