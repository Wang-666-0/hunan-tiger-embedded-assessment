/*
 * 文件功能：管理电机输出请求，并周期检查条件、发送控制报文。
 * 1. 接受限幅范围内的给定值，处理停止请求和 K 命令的有效期续期。
 * 2. 周期检查反馈在线状态、温度、命令有效期以及 CAN 发送状态；
 *    条件不满足时清零输出请求，并记录停止原因。
 * 3. 调用 GM6020_BuildCommand 打包，通过 can_transport.c 提交发送；
 *    发送异常时取消待发报文，避免旧给定持续重发。
 * 4. 提供控制状态快照，方便串口遥测和 Keil Watch 验收。
 * 模块关系：motor_console.c 更新请求，can_tasks.c 周期调用控制步骤；
 * CAN 发送与取消由控制任务统一执行，串口任务只修改请求状态。
 * 阅读重点：本文件没有转速或位置 PID，给定值不是目标 rpm；
 * 停止表示清零给定，不表示机械抱闸，线路断开时也无法保证零帧送达。
 */
#include "motor_control.h"
#include "can_transport.h"
#include "can_debug.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

volatile MotorControl_State motor_control = { // 未明确初始化的请求、给定、使能均为零。
    .mode = GM6020_CONTROL_MODE, // 记录当前编译选择的电压/电流协议。
    .motor_id = GM6020_MOTOR_ID, // 记录当前编译选择的目标电机编号。
    .stop_reason = MOTOR_STOP_POWER_ON // 初始没有非零请求，上电状态为停止。
};

static int32_t MotorControl_Limit(void)//获取控制限值
{
    return (GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE) ? // 根据当前模式选择教学上限，防止实验给定过大。
        MOTOR_VOLTAGE_DEMO_LIMIT : MOTOR_CURRENT_DEMO_LIMIT; // 电压 ±2000，电流 ±1000，均小于协议范围。
}

void MotorControl_Stop(MotorStopReason reason)//停止电机
{
    /* 本函数只更新请求状态，HAL 发送和取消都由唯一的控制任务执行。 */
    taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
    motor_control.enabled = 0U; // 锁定为禁用状态，恢复反馈也不会自动启动。
    motor_control.requested_raw = 0; // 清除用户的旧非零请求。
    motor_control.output_raw = 0; // 清零对外显示的本轮给定，实际发送由控制任务处理。
    motor_control.stop_reason = (uint32_t)reason; // 保存撤销请求的具体原因，供串口遥测返回。
    taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
}

uint8_t MotorControl_SetOutput(int16_t raw)//设置电机输出
{
    GM6020_Feedback feedback; // 局部反馈快照，检查条件时不直接读取零散共享字段。
    uint32_t now_ms; // HAL 毫秒时间，用于反馈年龄和命令有效期。
    int32_t limit = MotorControl_Limit(); // 取得当前协议的教学给定上限。
    if ((int32_t)raw < -limit || (int32_t)raw > limit) // 超教学限幅直接拒绝，不修改原来的输出请求。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }
    if (raw == 0) // 零给定统一按用户停止处理，不要求电机在线。
    {
        MotorControl_Stop(MOTOR_STOP_USER); // 请求停止；不会在串口调用路径里直接发送 CAN。
        return 1U; // 通知调用方本次处理成功或输入已被接受。
    }
    GM6020_GetFeedback(&feedback); // 先复制一致反馈，再读当前时间，避免抢占引起负年龄。
    /* 先取快照再读时间，避免被接收任务抢占后 now 比反馈时间更早。 */
    now_ms = HAL_GetTick(); // 在取得反馈之后记录当前毫秒时间。
    if (can_ready == 0U || GM6020_IsOnline(&feedback, now_ms) == 0U || // CAN 未启动或反馈过期，不接受非零给定。
        feedback.temperature >= MOTOR_TEMPERATURE_LIMIT_C) // 温度达到教学阈值 80℃ 也拒绝启用。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }

    taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
    motor_control.requested_raw = raw; // 保存用户请求，稍后由周期任务选择并发送。
    motor_control.enabled = 1U; // 允许周期任务采用这个非零请求。
    motor_control.last_command_ms = now_ms; // 新的有效给定或续期，从当前时刻重新计算三秒期限。
    motor_control.stop_reason = MOTOR_STOP_NONE; // 请求已启用，清除之前的停止原因。
    taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
    return 1U; // 通知调用方本次处理成功或输入已被接受。
}

uint8_t MotorControl_KeepAlive(void)//续期命令有效期
{
    uint8_t accepted = 0U; // 默认拒绝续期，只在原请求仍有效时返回接受。
    uint32_t now_ms = HAL_GetTick(); // 记录 K 命令被处理时的毫秒时间。
    taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
    /* K 只续期已启用的输出，不会让停下的电机重新启动。 */
    if (motor_control.enabled != 0U && // K 只能续期正在启用的请求，不能重新启动电机。
        // 已经达到三秒期限的旧请求不能用 K 恢复。
        (uint32_t)(now_ms - motor_control.last_command_ms) < MOTOR_COMMAND_TIMEOUT_MS)
    {
        motor_control.last_command_ms = now_ms; // 新的有效给定或续期，从当前时刻重新计算三秒期限。
        accepted = 1U; // 本次续期成功，通知串口命令处理模块。
    }
    taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
    return accepted; // 返回 K 是否实际更新了有效期。
}

void MotorControl_GetState(MotorControl_State *snapshot)//获取控制状态快照
{
    if (snapshot != NULL) // 防止向空指针复制控制状态。
    {
        taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
        *snapshot = motor_control;// 复制整个结构体，保证一致性
        taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
    }
}

void MotorControl_Step(void)//执行控制步骤
{
    GM6020_Feedback feedback; // 局部反馈快照，检查条件时不直接读取零散共享字段。
    uint32_t now_ms; // HAL 毫秒时间，用于反馈年龄和命令有效期。
    uint32_t id; // 组帧后输出的标准 CAN 报文 ID。
    uint8_t data[8]; // 本轮待发送的八字节命令缓冲。
    int16_t raw; // 本轮最终选出的给定，禁用时为零。
    uint8_t online; // 当前快照的在线判定结果。
    GM6020_GetFeedback(&feedback);// 获取电机反馈快照
    now_ms = HAL_GetTick(); // 在取得反馈之后记录当前毫秒时间。
    online = GM6020_IsOnline(&feedback, now_ms); // 依据接收时间判断反馈是否仍新鲜。

    /* 条件失效后锁定为停机，需要一条新的非零命令才能再次启用。 */
    taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
    motor_control.cycle_count++; // 统计执行过多少轮控制检查。
    motor_control.online = online; // 发布本轮在线状态，供遥测读取。
    if (motor_control.enabled != 0U) // 只对已启用请求判断是否需要撤销。
    {
        MotorStopReason reason = MOTOR_STOP_NONE; // 默认条件正常，按顺序检查故障原因。
        if (online == 0U) // 离线时不提交新控制帧，避免没有 ACK 时占用邮箱。
        {
            reason = MOTOR_STOP_FEEDBACK; // 记录反馈超时原因。
        }
        else if (feedback.temperature >= MOTOR_TEMPERATURE_LIMIT_C) // 反馈在线后检查温度。
        {
            reason = MOTOR_STOP_TEMPERATURE; // 记录温度超限原因。
        }
        // 最后检查请求是否三秒没有更新或续期。
        else if ((uint32_t)(now_ms - motor_control.last_command_ms) >= MOTOR_COMMAND_TIMEOUT_MS)
        {
            reason = MOTOR_STOP_COMMAND_TIMEOUT; // 记录命令期限耗尽原因。
        }
        if (reason != MOTOR_STOP_NONE) // 任一条件失败都撤销旧请求，需新给定才能再启用。
        {
            motor_control.enabled = 0U; // 锁定为禁用状态，恢复反馈也不会自动启动。
            motor_control.requested_raw = 0; // 清除用户的旧非零请求。
            motor_control.stop_reason = (uint32_t)reason; // 保存撤销请求的具体原因，供串口遥测返回。
        }
    }
    // 请求有效用请求值，否则本轮选择零给定。
    raw = (motor_control.enabled != 0U) ? motor_control.requested_raw : 0;
    motor_control.output_raw = raw; // 这只是准备发送的值，不代表电机实测输出。
    taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。

    if (can_ready == 0U) // CAN 尚未启动时不访问发送流程。
    {
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }

    /* 正常 1 Mbps 下，一帧的发送时间远小于 2 ms。
     * 下轮仍有挂起邮箱时，不堆积给定：取消旧报文并撤销输出。
     * HAL 的自动重发因此不能在断线恢复后继续发送旧的非零给定。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U) // 正常两毫秒周期内应已发完，上轮仍挂起视为发送异常。
    {
        can1_tx_busy_count++; // 记录出现挂起邮箱的轮次。
        MotorControl_Stop(MOTOR_STOP_CAN_TX); // 发送异常撤销给定，防止旧非零命令继续有效。
        CanTransport_AbortPending(); // 请求硬件取消旧报文，清除自动重发中的给定。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    /* 尚未收到反馈或反馈超时，不提交控制帧，避免无 ACK 时占满邮箱。 */
    if (online == 0U) // 离线时不提交新控制帧，避免没有 ACK 时占用邮箱。
    {
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }

    // 按当前 ID 和协议编码本轮给定，编码失败进入异常处理。
    if (GM6020_BuildCommand(GM6020_MOTOR_ID, GM6020_CONTROL_MODE, raw, &id, data) == 0U ||
        CanTransport_Send(id, data) != HAL_OK) // 编码成功后提交 CAN；失败同样撤销请求。
    {
        MotorControl_Stop(MOTOR_STOP_CAN_TX); // 发送异常撤销给定，防止旧非零命令继续有效。
        CanTransport_AbortPending(); // 请求硬件取消旧报文，清除自动重发中的给定。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
    motor_control.last_sent_raw = raw; // 记录最近成功提交邮箱的值，不保证已被电机执行。
    motor_control.last_tx_id = id; // 保存本轮提交的组报文 ID。
    for (uint8_t i = 0U; i < 8U; i++) // 复制完整八字节报文，供状态快照读取。
    {
        motor_control.last_tx_data[i] = data[i]; // 保留每个槽位的实际编码结果。
    }
    taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
}
