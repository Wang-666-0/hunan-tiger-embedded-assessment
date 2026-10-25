#include "motor_demo.h"
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart3;
volatile DemoControl g_control = {0, 0, 360, 1000, 500, 0.8f, 2.0f};
volatile MotorFeedback g_motor[8];
volatile DemoDebug g_debug;

enum { FAULT_FEEDBACK=1, FAULT_SPEED=2, FAULT_CAN=4, FAULT_LOOP=8,
       FAULT_TX=16, FAULT_OTHER_NODE=32, FAULT_UART=64, FAULT_STALL=128 };
static uint32_t end_ms, last_control_ms, selected, motion_start_ms;
static uint32_t run_limit, stuck_since;
static float run_kp, run_ki, integral;
static int32_t run_target;
static uint8_t uart_byte;
static volatile uint32_t emergency_stop;
static char rx_line[64];
static uint32_t rx_used;
static char tx_line[256];

static void SystemClock_Config(void);
static void Peripherals_Init(void);
static void control_step(uint32_t now);

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }
static uint32_t age_ms(uint32_t now, uint32_t then)
{
    /* An RX IRQ may have sampled the following millisecond after main read now. */
    int32_t delta = (int32_t)(now - then);
    return delta < 0 ? 0U : (uint32_t)delta;
}

static void abort_tx(void)
{
    /* Cancel any queued old nonzero command before requesting zero. */
    CAN1->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
}

static uint32_t send_current(int32_t current)
{
    CAN_TxHeaderTypeDef h = {0};
    uint8_t d[8] = {0};
    uint32_t mailbox, slot;
    if (selected == 0) return 1;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) return 0;
    h.StdId = selected <= 4 ? 0x200 : 0x1FF;
    h.IDE = CAN_ID_STD;
    h.RTR = CAN_RTR_DATA;
    h.DLC = 8;
    slot = ((selected - 1) % 4) * 2;
    d[slot] = (uint8_t)((uint16_t)current >> 8);
    d[slot + 1] = (uint8_t)current;
    if (HAL_CAN_AddTxMessage(&hcan1, &h, d, &mailbox) == HAL_OK) {
        g_debug.tx_ok++;
        return 1;
    }
    return 0;
}

static void stop_motor(uint32_t fault)
{
    g_debug.active = 0;
    g_debug.current_raw = 0;
    g_debug.target_rpm = 0;
    g_debug.remaining_ms = 0;
    g_debug.fault |= fault;
    integral = 0;
    abort_tx();
    /* Abort completion can take a few cycles; control_step retries zero each ms. */
    (void)send_current(0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
    CAN_RxHeaderTypeDef h;
    uint8_t d[8];
    uint32_t index, angle;
    int32_t delta;
    volatile MotorFeedback *m;
    while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0)) {
        if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &h, d) != HAL_OK) break;
        if (h.IDE != CAN_ID_STD || h.RTR != CAN_RTR_DATA || h.DLC != 8 ||
            h.StdId < 0x201 || h.StdId > 0x208) continue;
        angle = ((uint32_t)d[0] << 8) | d[1];
        if (angle > 8191) continue;
        index = h.StdId - 0x201;
        m = &g_motor[index];
        if (m->rx_count) {
            delta = (int32_t)angle - (int32_t)m->angle_raw;
            if (delta > 4096) delta -= 8192;
            if (delta < -4096) delta += 8192;
            /* Diagnostic displacement only; do not integrate across an outage. */
            if (HAL_GetTick() - m->last_rx_ms <= 20) m->total_counts += delta;
        }
        m->angle_raw = angle;
        m->rotor_rpm = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
        m->torque_raw = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
        m->last_rx_ms = HAL_GetTick();
        m->rx_count++;
        g_debug.rx_total++;
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can)
{
    g_debug.can_error |= HAL_CAN_GetError(can);
    if (g_debug.active) stop_motor(FAULT_CAN);
    HAL_CAN_ResetError(can);
}

static void serial_command(void)
{
    int id, rpm, duration, consumed = 0;
    if (!strcmp(rx_line, "STOP")) emergency_stop = 1;
    else if (!strcmp(rx_line, "CLEAR")) g_control.command = 3;
    else if (sscanf(rx_line, "RUN %d %d %d%n", &id, &rpm, &duration, &consumed) == 3 &&
             rx_line[consumed] == 0 && id >= 1 && id <= 8 && duration >= 100 &&
             duration <= 2000 && rpm >= -720 && rpm <= 720 && rpm != 0) {
        if (!g_debug.active && !g_control.command) {
            g_control.motor_id = (uint32_t)id;
            g_control.target_rpm = rpm;
            g_control.duration_ms = (uint32_t)duration;
            __DMB();
            g_control.command = 1;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint8_t c = uart_byte;
    if (uart != &huart3) return;
    if (c == '!') { emergency_stop = 1; rx_used = 0; }
    else if (c == '\n' || c == '\r') {
        if (rx_used) { rx_line[rx_used] = 0; serial_command(); rx_used = 0; }
    } else if (c >= 32 && c <= 126) {
        if (rx_used < sizeof(rx_line)-1) rx_line[rx_used++] = (char)c;
        else { emergency_stop = 1; rx_used = 0; }
    }
    (void)HAL_UART_Receive_IT(&huart3, &uart_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    g_debug.uart_drop++;
    emergency_stop = 1;
    (void)HAL_UART_Receive_IT(uart, &uart_byte, 1);
}

static void control_step(uint32_t now)
{
    uint32_t i, mask = 0, only = 0, count = 0, command;
    float error, proposed, output;
    int32_t rpm;
    g_debug.tick_ms = now;
    g_debug.can_esr = CAN1->ESR;
    for (i = 0; i < 8; i++) {
        if (g_motor[i].rx_count && age_ms(now, g_motor[i].last_rx_ms) <= 20) {
            mask |= 1U << i; only = i + 1; count++;
        }
    }
    g_debug.online_mask = mask;
    /* When stopped, selecting an ID enables periodic ZERO frames for that group. */
    if (!g_debug.active) {
        uint32_t choice = g_control.motor_id;
        if (choice == 0 && count == 1) choice = only;
        if (choice >= 1 && choice <= 8) selected = choice;
    }
    g_debug.selected_id = selected;
    if (selected) {
        /* A short critical section gives Watch/telemetry a consistent snapshot. */
        __disable_irq();
        g_debug.feedback_age_ms = age_ms(now, g_motor[selected-1].last_rx_ms);
        g_debug.measured_rpm = g_motor[selected-1].rotor_rpm;
        g_debug.torque_raw = g_motor[selected-1].torque_raw;
        g_debug.angle_raw = (int32_t)g_motor[selected-1].angle_raw;
        g_debug.total_counts = g_motor[selected-1].total_counts;
        __enable_irq();
        g_debug.output_rpm = (float)g_debug.measured_rpm / 36.0f;
        g_debug.rotor_angle_deg = (float)g_debug.angle_raw * (360.0f / 8192.0f);
        g_debug.output_angle_deg = (float)g_debug.total_counts * (360.0f / (8192.0f * 36.0f));
    }
    command = g_control.command;
    g_control.command = 0;
    if (emergency_stop || command == 2) {
        emergency_stop = 0;
        stop_motor(0);
        command = 0;
    }
    if (command == 3 && !g_debug.active) {
        g_debug.fault = 0;
        g_debug.can_error = 0;
        g_debug.command_result = 0;
        HAL_CAN_ResetError(&hcan1);
    }
    if (command == 1) {
        if (g_debug.active) g_debug.command_result = 1;
        else if (g_control.motor_id > 8 || g_control.target_rpm < -720 ||
                 g_control.target_rpm > 720 || g_control.target_rpm == 0 ||
                 g_control.duration_ms < 100 || g_control.duration_ms > 2000 ||
                 g_control.current_limit_raw < 1 || g_control.current_limit_raw > 500 ||
                 !(g_control.kp >= 0 && g_control.kp <= 3) ||
                 !(g_control.ki >= 0 && g_control.ki <= 10)) g_debug.command_result = 2;
        else if (g_debug.fault || count != 1 || !selected || only != selected ||
                 g_motor[selected-1].rx_count < 100 ||
                 abs32(g_debug.measured_rpm) > 180 ||
                 (CAN1->ESR & (CAN_ESR_EPVF|CAN_ESR_BOFF))) g_debug.command_result = 3;
        else {
            run_target = g_control.target_rpm;
            run_limit = g_control.current_limit_raw;
            run_kp = g_control.kp; run_ki = g_control.ki;
            end_ms = now + g_control.duration_ms;
            motion_start_ms = now; stuck_since = 0; integral = 0;
            g_debug.command_result = 0;
            g_debug.active = 1;
        }
    }
    if (g_debug.active) {
        if ((int32_t)(now - end_ms) >= 0) stop_motor(0);
        else if (g_debug.feedback_age_ms > 20) stop_motor(FAULT_FEEDBACK);
        else if (count != 1 || only != selected) stop_motor(FAULT_OTHER_NODE);
        else if (now - last_control_ms > 10) stop_motor(FAULT_LOOP);
        else if (abs32(g_debug.measured_rpm) > 1800) stop_motor(FAULT_SPEED);
        else if (CAN1->ESR & (CAN_ESR_EPVF | CAN_ESR_BOFF)) stop_motor(FAULT_CAN);
    }
    if (g_debug.active) {
        rpm = g_debug.measured_rpm;
        error = (float)run_target - (float)rpm;
        proposed = integral + run_ki * error * 0.001f;
        output = run_kp * error + proposed;
        /* Conditional integration prevents accumulating against the current limit. */
        if ((output <= (float)run_limit && output >= -(float)run_limit) ||
            (output > (float)run_limit && error < 0) ||
            (output < -(float)run_limit && error > 0)) integral = proposed;
        output = run_kp * error + integral;
        if (output > (float)run_limit) output = (float)run_limit;
        if (output < -(float)run_limit) output = -(float)run_limit;
        g_debug.current_raw = (int32_t)output;
        g_debug.target_rpm = run_target;
        g_debug.remaining_ms = end_ms - now;
        if (now - motion_start_ms > 300 && abs32(rpm) < 30 &&
            abs32(g_debug.current_raw) >= 400) {
            if (!stuck_since) stuck_since = now;
            if (now - stuck_since >= 300) stop_motor(FAULT_STALL);
        } else stuck_since = 0;
    }
    if (!g_debug.active) {
        g_debug.current_raw = 0;
        g_debug.target_rpm = 0;
        g_debug.remaining_ms = 0;
    }
    g_debug.integral = integral;
    if (!send_current(g_debug.active ? g_debug.current_raw : 0) && g_debug.active)
        stop_motor(FAULT_TX);
    last_control_ms = now;
}

static void vofa_send(void)
{
    int len;
    if (huart3.gState != HAL_UART_STATE_READY) { g_debug.uart_drop++; return; }
    /* FireWater: 12 channels, 50 Hz. I10=rotor deg; I11=relative output deg. */
    len = snprintf(tx_line, sizeof(tx_line), "motor:%ld,%ld,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%.2f,%.2f\r\n",
        (long)g_debug.target_rpm, (long)g_debug.measured_rpm,
        (long)g_debug.current_raw, (long)g_debug.torque_raw, (long)g_debug.angle_raw,
        (unsigned long)g_debug.selected_id, (unsigned long)g_debug.online_mask,
        (unsigned long)g_debug.active, (unsigned long)g_debug.fault,
        (unsigned long)g_debug.feedback_age_ms,
        (double)g_debug.rotor_angle_deg, (double)g_debug.output_angle_deg);
    if (len > 0 && len < (int)sizeof(tx_line))
        (void)HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx_line, (uint16_t)len);
}

int main(void)
{
    uint32_t now, last_vofa = 0;
    HAL_Init();
    SystemClock_Config();
    Peripherals_Init();
    g_debug.version = 0x20060102;
    last_control_ms = HAL_GetTick();
    while (1) {
        now = HAL_GetTick();
        if (now != last_control_ms) control_step(now);
        if (now - last_vofa >= 20) { last_vofa = now; vofa_send(); }
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25; osc.PLL.PLLN = 168;
    osc.PLL.PLLP = RCC_PLLP_DIV2; osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void Peripherals_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    CAN_FilterTypeDef filter = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    /* PC15 drives the laser transistor. Explicitly hold it OFF. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_15; gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    __HAL_RCC_CAN1_CLK_ENABLE(); __HAL_RCC_USART3_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11|GPIO_PIN_12; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10|GPIO_PIN_11; gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3; /* APB1 42 MHz / (3 * (1+11+2)) = 1 Mbit/s */
    hcan1.Init.Mode = CAN_MODE_NORMAL; hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ; hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.AutoBusOff = ENABLE; hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE; hcan1.Init.TransmitFifoPriority = ENABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();
    filter.FilterBank = 0; filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x200 << 5; filter.FilterMaskIdHigh = 0x7F0 << 5;
    filter.FilterMaskIdLow = 6; /* Only standard data frames. */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0; filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK || HAL_CAN_Start(&hcan1) != HAL_OK)
        Error_Handler();
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0); HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 1, 0); HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING |
          CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
          CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR) != HAL_OK) Error_Handler();
    huart3.Instance = USART3; huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B; huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE; huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0); HAL_NVIC_EnableIRQ(USART3_IRQn);
    (void)HAL_UART_Receive_IT(&huart3, &uart_byte, 1);
}

void SysTick_Handler(void) { HAL_IncTick(); }
void CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
void CAN1_SCE_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }
void HardFault_Handler(void) { NVIC_SystemReset(); }
void MemManage_Handler(void) { NVIC_SystemReset(); }
void BusFault_Handler(void) { NVIC_SystemReset(); }
void UsageFault_Handler(void) { NVIC_SystemReset(); }
void Error_Handler(void)
{
    g_debug.fault |= FAULT_CAN;
    __disable_irq();
    NVIC_SystemReset();
    while (1) {}
}
