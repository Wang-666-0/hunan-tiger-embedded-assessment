#include "app_fault.h"
#include "main.h"
void AppFault_Fatal(AppFault fault)
{
    (void)fault;
    /* 致命错误进入Error_Handler后无法调度停机重试。电调自身通信超时
     * 是最后兜底；此处不能在堆损坏/ISR环境依赖RTOS或阻塞发送。 */
    Error_Handler();
}
