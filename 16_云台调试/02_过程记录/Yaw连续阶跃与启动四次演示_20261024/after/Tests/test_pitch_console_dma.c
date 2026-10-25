/* Exercise production USART command routing with DMA bursts, not byte IRQs.
 * Reuse the existing transport/app snapshots; real app_console.c is linked. */
#define PITCH_TELEMETRY_TEST_ENTRY telemetry_baseline_test_run
#include "test_pitch_telemetry.c"
static uint32_t dma_rearms;
static uint8_t dma_rearm_failure;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size);
HAL_StatusTypeDef ConsoleUart_StartReceive(uint8_t *data, uint16_t length)
{
    CHECK(length == APP_CONSOLE_RX_DMA_BYTES && length == 128U && data != 0);
    rx_data = data; ++dma_rearms;
    if (!dma_rearm_failure) { huart1.error = 0U; }
    return dma_rearm_failure ? HAL_ERROR : HAL_OK;
}
static void Burst(const char *text)
{
    uint16_t size = (uint16_t)strlen(text);
    CHECK(size && size <= APP_CONSOLE_RX_DMA_BYTES);
    memcpy(rx_data, text, size);
    HAL_UARTEx_RxEventCallback(&huart1, size);
}
void test_run(void)
{
    uint32_t before, epoch;
    char full[129];
    test_stage = 1U;
    CHECK(APP_CONSOLE_RX_DMA_ENABLED && APP_PITCH_ENABLED && APP_YAW_ENABLED && APP_CONSOLE_BAUDRATE == 921600U);
    AppConsole_CreateQueue(); AppConsole_Start(); clear_output(); command_line("YAW SIM OFF"); clear_output(); command_line("PITCH SIM OFF");
    CHECK(dma_rearms == 1U && rearms == 0U && rx_data);
    HAL_UART_RxCpltCallback(&huart4); HAL_UART_ErrorCallback(&huart4);
    CHECK(remote_rx == 1U && remote_errors == 1U);
#if APP_REMOTE_RX_DMA_ENABLED
    HAL_UARTEx_RxEventCallback(&huart4, 32U);
    CHECK(remote_events == 1U && remote_event_size == 32U && pitch_commands == 0U);
#endif
    /* Hardware DMA can finish the burst while task/IRQ handling is delayed. */
    memcpy(rx_data, "PITCH COORD 1\n\r", 15U); now += 2U;
    CHECK(pitch_commands == 0U);
    HAL_UARTEx_RxEventCallback(&huart1, 15U); AppConsole_Poll();
    CHECK(pitch_commands == 1U && strstr(output, "P|MOCK\r\n") && AppConsole_GetFaultEpoch() == 0U);
    CHECK(dma_rearms == 2U && rearms == 0U);
    clear_output(); Burst("PITCH COORD "); AppConsole_Poll();
    CHECK(pitch_commands == 1U);
    Burst("1\r\n"); AppConsole_Poll(); CHECK(pitch_commands == 2U);
    clear_output(); Burst("PITCH STATUS\r\nPITCH GUARD\n\r"); AppConsole_Poll();
    CHECK(pitch_commands == 4U && strstr(output, "P|MOCK"));
    test_stage = 2U;
    memset(full, 'A', 128U); full[128] = 0;
    Burst(full); epoch = AppConsole_GetFaultEpoch(); Burst(full);
    CHECK(AppConsole_GetFaultEpoch() == epoch + 1U);
    clear_output(); before = pitch_commands; AppConsole_Poll();
    CHECK(pitch_commands == before && strstr(output, "C|UART_FAULT|") && strstr(output, "DROP|1"));
    Burst("\r\n"); AppConsole_Poll(); clear_output();
    Burst("PITCH COORD 1\r\n"); AppConsole_Poll(); CHECK(pitch_commands == before + 1U);
    test_stage = 3U;
    Burst("PITCH CO"); AppConsole_Poll();
    huart1.error = 8U; epoch = AppConsole_GetFaultEpoch(); HAL_UART_ErrorCallback(&huart1);
    CHECK(AppConsole_GetFaultEpoch() == epoch + 1U);
    clear_output(); AppConsole_Poll(); CHECK(strstr(output, "LAST|0x00000008") && strstr(output, "RX_ERR|1"));
    before = pitch_commands; Burst("ORD 1\r\n"); AppConsole_Poll(); CHECK(pitch_commands == before);
    Burst("PITCH COORD 1\n\r"); AppConsole_Poll(); CHECK(pitch_commands == before + 1U);
    dma_rearm_failure = 1U; epoch = AppConsole_GetFaultEpoch();
    Burst("PITCH COORD 1\r\n"); CHECK(AppConsole_GetFaultEpoch() == epoch + 1U);
    dma_rearm_failure = 0U; before = pitch_commands; AppConsole_Poll(); CHECK(pitch_commands == before);
    Burst("\r\n"); AppConsole_Poll(); Burst("PITCH STATUS\r\n"); AppConsole_Poll();
    CHECK(pitch_commands == before + 1U && rearms == 0U);
    clear_output(); Burst("REMOTE RX\r\n"); AppConsole_Poll();
    CHECK(strstr(output, "R|RX|EPOCH|0|LAST|0x00000000|RX_ERR|0|DROP|0|REARM_ERR|0"));
#if APP_REMOTE_RX_DMA_ENABLED
    CHECK(strstr(output, "RX_DMA|1"));
#else
    CHECK(strstr(output, "RX_DMA|0"));
#endif
    test_stage = 4U;
    clear_output(); Burst("TELEM OFF\r\nPITCH VIEW\r\n"); AppConsole_Poll();
    CHECK(strstr(output, "C|OK|TELEM|OFF") && strstr(output, "GROUP|pitch|COLS|65"));
    now += 100U; clear_output(); AppConsole_Poll(); CHECK(!output_length);
    before = pitch_commands;
    Burst("PITCH PID\r\n"); AppConsole_Poll(); CHECK(pitch_commands == before + 1U && strstr(output, "P|MOCK"));
    clear_output(); Burst("TELEM ON\r\nYAW VIEW\r\n"); AppConsole_Poll();
    CHECK(strstr(output, "C|OK|TELEM|ON") && strstr(output, "Y|VIEW|YAW|GROUP|gimbal"));
    test_done = 1U;
}
