/**
 * @file vesc_diag.h
 * @brief VESC 只读诊断协议
 * 职责：定义参数接收缓存，声明查询、分片拼接和 CRC 结果接口。
 * 主要接口：VescDiag_BuildRead, VescDiag_Begin, VescDiag_Reset, VescDiag_OnFrame。
 * 实现/调用：app_diagnostics.c；长度及节点/命令白名单验证保持原逻辑。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef VESC_DIAG_H
#define VESC_DIAG_H
#include "app_config.h"

#include "can_frame.h"

/* 只允许一笔查询在途。分片帧不携带发送节点，不能并行查询两个电调。 */
typedef struct {
    uint8_t buffer[VESC_DIAG_MAX_PAYLOAD];
    uint16_t length;
    uint16_t next_offset;
    uint8_t expected_node;
    uint8_t expected_command;
    uint8_t active;
} VescDiagReceiver;

/* 仅构造固件版本、运行量、电机配置、应用配置及临时配置的读取请求。
 * 此接口不能构造使能、转速、电流、写参数或检测电机命令。 */
uint8_t VescDiag_BuildRead(uint8_t node, uint8_t command, CanFrame *out);
void VescDiag_Begin(VescDiagReceiver *receiver, uint8_t node, uint8_t command);
void VescDiag_Reset(VescDiagReceiver *receiver);

/* 返回1：完整响应已存入buffer/length；0：忽略或等待后续分片；
 * 返回-1：匹配的响应损坏，本次查询结束。超时由上层统一处理。 */
int VescDiag_OnFrame(VescDiagReceiver *receiver, const CanFrame *frame);

#endif
