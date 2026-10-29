/**
 * @file app_fault.c
 * 作用：应用致命故障处置
 * 对外函数：
 *   AppFault_Fatal
 * 内部函数：
 *   无。
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "app_fault.h"
#include "main.h"

void AppFault_Fatal(AppFault fault)
/*
 * 作用：处理不可恢复的应用初始化故障，使系统进入原致命故障路径。
 * 参数 fault：无法继续初始化/运行的系统故障代码。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    (void)fault;
    /* 致命错误进入Error_Handler后无法调度停机重试。电调自身通信超时
     * 是最后兜底；此处不能在堆损坏/ISR环境依赖RTOS或阻塞发送。 */
    Error_Handler();
}
