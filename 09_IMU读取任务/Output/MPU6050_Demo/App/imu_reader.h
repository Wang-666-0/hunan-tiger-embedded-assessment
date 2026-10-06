#ifndef IMU_READER_H
#define IMU_READER_H

/* 外设初始化之后调用，完成 DMP 初始化和量程灵敏度读取。 */
void ImuReader_Init(void);
/* 读取当前 FIFO 包，换算物理量，再调用互补滤波。 */
void ImuReader_Process(void);

#endif /* IMU_READER_H */
