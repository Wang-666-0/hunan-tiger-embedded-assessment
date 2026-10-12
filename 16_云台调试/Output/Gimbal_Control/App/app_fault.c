#include "app_fault.h"
#include "main.h"
void AppFault_Fatal(AppFault fault)
{
    (void)fault;
    /* 基础版没有动力输出；资源或初始化失败后不继续运行不完整的系统。 */
    Error_Handler();
}
