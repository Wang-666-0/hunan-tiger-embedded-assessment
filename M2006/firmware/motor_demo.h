#ifndef MOTOR_DEMO_H
#define MOTOR_DEMO_H
#include "stm32f4xx_hal.h"

/* Watch: edit g_control fields, then write command LAST. All speeds are rotor rpm. */
typedef struct {
    uint32_t command;             /* 0=none, 1=one timed run, 2=stop, 3=clear fault */
    uint32_t motor_id;            /* 0=auto-select the only online ID; 1..8=manual */
    int32_t target_rpm;           /* rotor rpm; 360 rotor rpm ~= 10 output rpm */
    uint32_t duration_ms;         /* 100..2000 ms; no continuous-run command */
    uint32_t current_limit_raw;   /* 1..500 raw; 500 raw = 0.5 A request */
    float kp;
    float ki;
} DemoControl;

typedef struct {
    uint32_t rx_count;
    uint32_t last_rx_ms;
    uint32_t angle_raw;
    int32_t rotor_rpm;
    int32_t torque_raw;           /* feedback torque raw, NOT calibrated amperes */
    int32_t total_counts;
} MotorFeedback;

typedef struct {
    uint32_t version;            /* 0x20060102: degree telemetry channels */
    uint32_t tick_ms;
    uint32_t selected_id;
    uint32_t online_mask;
    uint32_t active;
    uint32_t fault;              /* see README: bit flags */
    uint32_t remaining_ms;
    uint32_t feedback_age_ms;
    uint32_t rx_total;
    uint32_t tx_ok;
    uint32_t can_esr;
    uint32_t can_error;
    uint32_t uart_drop;
    uint32_t command_result;     /* 0=accepted; 1=busy; 2=invalid; 3=not ready */
    int32_t target_rpm;
    int32_t measured_rpm;
    int32_t current_raw;
    int32_t torque_raw;
    int32_t angle_raw;
    int32_t total_counts;
    float output_rpm;
    float integral;
    float rotor_angle_deg;      /* rotor single-turn angle, [0, 360) degrees */
    float output_angle_deg;     /* relative output angle, nominal 36:1, degrees */
} DemoDebug;

extern volatile DemoControl g_control;
extern volatile MotorFeedback g_motor[8];
extern volatile DemoDebug g_debug;
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart3;
void Error_Handler(void);
#endif
