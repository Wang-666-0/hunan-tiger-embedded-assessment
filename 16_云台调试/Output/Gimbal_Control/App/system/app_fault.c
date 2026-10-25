/**
 * @file app_fault.c
 * @brief 应用致命故障处置
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（1）：
 *   AppFault_Fatal
 * 内部函数（0）：
 *   无。
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 致命故障入口：交由 Error_Handler，避免在异常环境依赖 RTOS。 */
#include "app_fault.h"
#include "main.h"
void AppFault_Fatal(AppFault fault)
/* 功能：处理不可恢复的应用初始化故障，使系统进入原致命故障路径。 */
{
    (void)fault;
    /* 致命错误进入Error_Handler后无法调度停机重试。电调自身通信超时
     * 是最后兜底；此处不能在堆损坏/ISR环境依赖RTOS或阻塞发送。 */
    Error_Handler();
}
