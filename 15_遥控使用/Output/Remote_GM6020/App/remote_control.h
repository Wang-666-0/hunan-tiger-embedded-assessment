#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include <stdint.h>

/* 纯状态机，不依赖 HAL/FreeRTOS；通道编号为人看的 CH1/CH5/CH6。 */
typedef enum {
    REMOTE_FAULT_NONE = 0, REMOTE_FAULT_POWER_ON = 1,
    REMOTE_FAULT_DISARM = 2, REMOTE_FAULT_LINK = 3,
    REMOTE_FAULT_FAILSAFE = 4, REMOTE_FAULT_CAN = 5,
    REMOTE_FAULT_TEMPERATURE = 6, REMOTE_FAULT_UART = 7,
    REMOTE_FAULT_CONTROL_TIMING = 8, REMOTE_FAULT_CAN_TX = 9,
    REMOTE_FAULT_STOP = 10, REMOTE_FAULT_NOT_CENTERED = 11
} RemoteControl_Fault;

typedef struct {
    uint16_t channels[14];
    uint32_t now_ms, received_ms, frame_sequence;
    uint8_t has_frame, can_online, can_temperature;
    uint8_t uart_fault, can_tx_fault, control_late;
} RemoteControl_Input;

typedef struct {
    float target_rpm;
    uint32_t fault, last_sequence;
    uint8_t armed, off_seen, previous_arm, have_sequence;
} RemoteControl;

void RemoteControl_Init(RemoteControl *control);
void RemoteControl_ForceStop(RemoteControl *control, RemoteControl_Fault fault);
void RemoteControl_Step(RemoteControl *control, const RemoteControl_Input *input);
float RemoteControl_MapSpeed(uint16_t channel);
#endif
