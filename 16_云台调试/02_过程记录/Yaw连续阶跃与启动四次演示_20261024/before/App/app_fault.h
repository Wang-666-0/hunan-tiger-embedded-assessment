#ifndef APP_FAULT_H
#define APP_FAULT_H
typedef enum {
    APP_FAULT_QUEUE = 1, APP_FAULT_TASK = 2, APP_FAULT_CAN_START = 3,
    APP_FAULT_UART_START = 4, APP_FAULT_STACK = 5, APP_FAULT_MALLOC = 6
} AppFault;
void AppFault_Fatal(AppFault fault);
#endif
