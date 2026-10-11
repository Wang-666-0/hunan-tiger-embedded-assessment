/*
 * 文件功能：按照 GM6020 协议解码反馈、判断在线状态、编码控制命令。
 * 1. 筛选 CAN1 上目标电机的 8 字节反馈，解析编码值、角度、转速、
 *    电流原始值和温度；反馈 ID 为 0x204 + 电机 ID。
 * 2. 根据最后接收时间和反馈超时参数，判断电机是否在线。
 * 3. 根据电机 ID、控制模式和给定值，选择命令 ID 与双字节槽位，
 *    生成完整的 8 字节控制数据；本工程只控制一台电机，其余槽位填零。
 * 模块关系：调用 can_protocol.c 的字节转换工具；供 gm6020.c 和
 * motor_control.c 使用，不直接访问 CAN 硬件或更新共享反馈。
 * 阅读重点：角度是单圈机械角度；反馈电流原始值没有在此换算为安培；
 * 控制给定值不是转速，电压/电流命令模式必须与电机的实际设置匹配。
 */
#include "gm6020.h"
#include <stddef.h>

/* message 是输入报文，result 是输出反馈；返回 1 成功、0 拒绝。
 * 所有报文校验在写 result 之前完成，失败不会覆盖旧结果。 */
uint8_t GM6020_DecodeFeedback(const CanRxMessage *message, GM6020_Feedback *result)//解码反馈信息
{
    uint16_t encoder; // 临时编码值；先校验，成功后才填写输出结构体。
    if (message == NULL || result == NULL) // 检查输入和输出指针，避免访问不存在的内存。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }
    if (message->source != 1U || message->id != GM6020_FEEDBACK_ID) // 只处理 CAN1 上当前配置电机的反馈。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }
    if (message->dlc != 8U) // GM6020 反馈固定八字节，长度不对不能按协议解码。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }

    encoder = CanProtocol_ReadU16BE(&message->data[0]); // 反馈第 0、1 字节是无符号大端编码值。
    if (encoder > 8191U) // 一圈共 8192 个编码位置，合法范围为 0～8191。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }

    result->encoder = encoder;//单圈编码位置，0～8191
    result->angle_deg = (float)encoder * 360.0f / 8192.0f;// 单圈绝对机械角度，[0,360)，没有做零点校准
    result->speed_rpm = CanProtocol_ReadI16BE(&message->data[2]);// 带符号转速，rpm
    result->current_raw = CanProtocol_ReadI16BE(&message->data[4]);// 手册没有提供反馈电流的安培换算比例
    result->temperature = message->data[6];// 摄氏度
    /* data[7] 是保留字节，不作为序号或校验和。 */
    result->rx_count = 0U; // 纯解码不累计帧数；由 gm6020.c 发布反馈时更新。
    result->last_rx_ms = 0U; // 纯解码不读取硬件时间；发布时使用报文接收时间。
    return 1U; // 通知调用方本次处理成功或输入已被接受。
}

/* snapshot 是完整反馈快照，now_ms 是取快照之后读取的 HAL 毫秒时间。 */
uint8_t GM6020_IsOnline(const GM6020_Feedback *snapshot, uint32_t now_ms)//判断电机是否在线
{
    uint32_t elapsed_ms; // 保存距离上一条有效反馈的毫秒时间差。
    if (snapshot == NULL || snapshot->rx_count == 0U) // 没有有效快照或从未收到反馈，不能判为在线。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }
    /* 无符号减法支持 HAL 毫秒计数跨越 0xFFFFFFFF 后的短期超时判断。 */
    elapsed_ms = (uint32_t)(now_ms - snapshot->last_rx_ms); // 无符号减法使短期超时判断兼容计数器回绕。
    return (uint8_t)(elapsed_ms < MOTOR_FEEDBACK_TIMEOUT_MS); // 反馈年龄小于配置阈值为在线；本阶段为 20 ms，等于阈值即离线。
}

/* 输入 motor_id=1～7、mode=电压/电流协议、raw=有符号原始给定。
 * 输出 can_id 和 data[8]；返回 1 表示组帧成功，0 表示输入不合法。 */
uint8_t GM6020_BuildCommand(uint8_t motor_id, uint8_t mode, int16_t raw,
                           uint32_t *can_id, uint8_t data[8])//构建命令
{
    uint8_t slot; // 目标电机在组报文里的槽位编号，每槽占两个字节。
    int32_t limit; // 手册规定的协议上限；与更小的教学限幅不同。
    // 输出缓冲必须存在，电机编号只支持 1～7。
    if (can_id == NULL || data == NULL || motor_id < 1U || motor_id > 7U)
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }
    if (mode == GM6020_MODE_VOLTAGE) // 电压协议使用 ±25000 的原始给定范围。
    {
        limit = GM6020_VOLTAGE_RAW_LIMIT; // 手册 V1.4 的电压给定上限。
    }
    else if (mode == GM6020_MODE_CURRENT) // 电流协议使用 ±16384 的原始给定范围。
    {
        limit = GM6020_CURRENT_RAW_LIMIT; // 电流命令全量程对应 ±3 A，不是反馈电流换算。
    }
    else
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }
    if ((int32_t)raw < -limit || (int32_t)raw > limit) // 超出协议范围直接拒绝，不截断或溢出。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }

    /* 每帧四个槽位，每个槽位两个字节，未使用槽位始终发零。
     * 本工程针对一台电机，同组其他 GM6020 也会收到零给定。 */
    for (uint8_t i = 0U; i < 8U; i++)// 初始化所有槽位为零
    {
        data[i] = 0U; // 清除全部槽位，避免旧缓冲内容控制其他电机。
    }
    if (motor_id <= 4U) // 电机 1～4 使用第一组控制 ID。
    {
        *can_id = (mode == GM6020_MODE_VOLTAGE) ? 0x1FFU : 0x1FEU; // 根据模式选择第一组的电压或电流报文。
        slot = (uint8_t)(motor_id - 1U); // ID 1～4 映射为零起点槽位 0～3。
    }
    else
    {
        *can_id = (mode == GM6020_MODE_VOLTAGE) ? 0x2FFU : 0x2FEU; // 电机 5～7 使用第二组电压或电流报文。
        slot = (uint8_t)(motor_id - 5U); // ID 5～7 映射为第二组槽位 0～2。
    }
    CanProtocol_WriteI16BE(&data[slot * 2U], raw); // 槽位乘二得到字节偏移，将给定写入对应高低字节。
    return 1U; // 通知调用方本次处理成功或输入已被接受。
}
