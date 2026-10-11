#include "bmi088_port.h"
#include "main.h"
#include "spi.h"
#include "cmsis_os2.h"

/* 一次事务只传几个字节；2 ms 超时足够，禁止用 1000 ms 拖住 5 ms 周期任务。 */
#define BMI088_SPI_TIMEOUT_MS 2U

static uint32_t cycles_per_us; /* DWT 每微秒计数次数，是延时状态，不是调试 Watch 变量。 */

static void ReleaseBothChipSelects(void)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_SET); /* PC4 高：释放加速度计。 */
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);    /* PA4 高：释放陀螺仪。 */
}

bool BMI088_PortInit(void)
{
    uint32_t previous;
    uint32_t i;

    cycles_per_us = 0U; /* 初始化失败时，不允许之后使用未就绪的微秒延时。 */
    ReleaseBothChipSelects(); /* 两个器件共享 SPI，总线空闲时必须都不被选中。 */
    if (osKernelGetState() != osKernelRunning)
    {
        return false; /* 后续初始化使用 osDelay，不能从 main 的调度器启动前调用。 */
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 打开 Cortex-M4 跟踪单元。 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;           /* 启用 CPU 周期计数器，不修改 HAL 或 RTOS 时基。 */
    previous = DWT->CYCCNT;                       /* 不清零共享 DWT，以免影响其他模块的时间测量。 */
    for (i = 0U; i < 32U; ++i)
    {
        __NOP(); /* 留出足够的 CPU 周期，检查计数器确实在运行。 */
    }
    if (DWT->CYCCNT == previous || SystemCoreClock < 1000000U)
    {
        return false; /* 若 DWT 不运行，直接报错，而不是陷入无限延时。 */
    }
    cycles_per_us = SystemCoreClock / 1000000U; /* 当前 168 MHz 时得到 168 周期/微秒。 */
    return true;
}

void BMI088_PortDelayUs(uint32_t microseconds)
{
    if (cycles_per_us == 0U)
    {
        return; /* 防御未初始化的调用；正式初始化会检查 PortInit 返回值。 */
    }
    while (microseconds != 0U)
    {
        uint32_t chunk = microseconds > 1000U ? 1000U : microseconds; /* 分段避免乘法溢出。 */
        uint32_t target = chunk * cycles_per_us;
        uint32_t begin = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - begin) < target)
        {
            __NOP(); /* 无符号减法在 CYCCNT 回绕时仍然正确；不关闭中断。 */
        }
        microseconds -= chunk;
    }
}

void BMI088_PortDelayMs(uint32_t milliseconds)
{
    uint32_t frequency;
    uint32_t ticks;

    if (milliseconds == 0U)
    {
        return;
    }
    frequency = osKernelGetTickFreq(); /* 不硬编码 osDelay 的单位；本工程为 1000 Hz。 */
    ticks = (uint32_t)(((uint64_t)milliseconds * frequency + 999U) / 1000U); /* 向上取整。 */
    (void)osDelay(ticks); /* 毫秒等待让出 CPU，避免初始化期间一直占用任务运行时间。 */
}

bool BMI088_PortTransfer(bmi088_port_device_t device,
                         const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    HAL_StatusTypeDef status;

    ReleaseBothChipSelects(); /* 先释放两个片选，确保同一时刻只有一个器件驱动 MISO。 */
    if (tx == 0 || rx == 0 || length == 0U || cycles_per_us == 0U ||
        (device != BMI088_PORT_ACCEL && device != BMI088_PORT_GYRO))
    {
        return false; /* 参数错误也保证片选处于安全的高电平。 */
    }
    if (device == BMI088_PORT_ACCEL)
    {
        HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_RESET); /* 选中 PC4。 */
    }
    else
    {
        HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET); /* 选中 PA4。 */
    }
    BMI088_PortDelayUs(1U); /* 满足 CS 建立时间，整个地址/占位/数据过程保持 CS 为低。 */
    status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, length, BMI088_SPI_TIMEOUT_MS);
    BMI088_PortDelayUs(1U); /* 满足最后一个时钟后的片选保持时间。 */
    ReleaseBothChipSelects(); /* HAL 超时、BUSY、ERROR 和成功都必须释放片选。 */
    return status == HAL_OK; /* 错误上传给驱动和任务，不悄悄使用不完整的数据。 */
}
