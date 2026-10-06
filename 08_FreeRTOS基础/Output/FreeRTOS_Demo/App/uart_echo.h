#ifndef UART_ECHO_H
#define UART_ECHO_H

/* 任务入口：收一个字节，按 rx:<数值> 的 FireWater 帧回复。 */
void UartEcho_Task(void *argument);

#endif /* UART_ECHO_H */
