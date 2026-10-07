/*
 * 六个 PID 增益的掉电保存接口；不保存目标、零点或 PID 历史。
 * 两个工程使用互不重叠的日志区：
 * SpeedPID: sector 8/9；AnglePID: sector 10/11。
 * 两工程的应用链接区均须限制为前 512 KB，下载仅擦应用涉及扇区。
 */
#ifndef PID_STORE_H
#define PID_STORE_H

#include <stdint.h>
#include "app_config.h"

#ifndef PID_STORE_APP_KIND
#error PID_STORE_APP_KIND_must_be_defined_in_app_config
#endif

#if PID_STORE_APP_KIND == 1U
#define PID_STORE_SECTOR_A_NUMBER 8U
#define PID_STORE_SECTOR_B_NUMBER 9U
#define PID_STORE_SECTOR_A_ADDRESS 0x08080000U
#define PID_STORE_SECTOR_B_ADDRESS 0x080A0000U
#define PID_STORE_RECORD_MAGIC 0x53504944U /* 独立的 SpeedPID 格式标识。 */
#elif PID_STORE_APP_KIND == 2U
#define PID_STORE_SECTOR_A_NUMBER 10U
#define PID_STORE_SECTOR_B_NUMBER 11U
#define PID_STORE_SECTOR_A_ADDRESS 0x080C0000U
#define PID_STORE_SECTOR_B_ADDRESS 0x080E0000U
#define PID_STORE_RECORD_MAGIC 0x41504944U /* 独立的 AnglePID 格式标识。 */
#else
#error Invalid_PID_STORE_APP_KIND
#endif

#define PID_STORE_SECTOR_SIZE_BYTES 0x00020000U
#define PID_STORE_RECORD_SIZE_BYTES 64U
#define PID_STORE_RECORD_WORDS 16U
#define PID_STORE_RECORDS_PER_SECTOR (PID_STORE_SECTOR_SIZE_BYTES / PID_STORE_RECORD_SIZE_BYTES)
#define PID_STORE_FORMAT_VERSION 1U
#define PID_STORE_COMMIT_WORD 0x434F4D54U

typedef struct
{
    float speed_kp, speed_ki, speed_kd;
    float angle_kp, angle_ki, angle_kd;
} PidStore_Params;

typedef enum
{
    PID_STORE_SAVED = 0,
    PID_STORE_UNCHANGED = 1,
    PID_STORE_INVALID = 2,
    PID_STORE_FLASH_ERROR = 3,
    PID_STORE_NOT_READY = 4 // 调用层：仍在运行、保存忙或尚未确认零帧；本模块不返回。
} PidStore_Result;

/* 1=找到最新完整且增益范围有效的记录；0=无有效记录或硬件不支持。
 * 失败不修改 params。启动时可在 FreeRTOS 初始化前调用。
 */
uint8_t PidStore_Load(PidStore_Params *params);

/* 保存一个一致的参数快照。相同的六个数值不产生任何擦写。
 * 调用者必须保证闭环已停止、保存期间不能启动或并发调用本模块。
 * 单 bank Flash 擦写会阻塞取指；保存后须重新确认反馈与多圈零点。
 * 完整新记录的 commit 写入后才成为有效；绝不擦最新有效记录所在扇区。
 */
PidStore_Result PidStore_Save(const PidStore_Params *params);

/* 有限、非负且未超过当前工程的六个增益上界。
 * SpeedPID 仅使用内环，三个 angle_* 必须为 0。
 */
uint8_t PidStore_Validate(const PidStore_Params *params);

/* 仅软件测试构建启用，实际硬件后端完全不编入。
 * address/sector 保留真实逻辑编号，由 mock 映射到数组，禁止访问板卡。
 * 除 ReadWord 外均返回 1 成功、0 失败；可模拟中途掉电、部分擦写和错误。
 */
#ifdef PID_STORE_TEST_BACKEND
uint32_t PidStore_Backend_ReadWord(uint32_t address);
uint8_t PidStore_Backend_Unlock(void);
uint8_t PidStore_Backend_Lock(void);
uint8_t PidStore_Backend_ProgramWord(uint32_t address, uint32_t value);
uint8_t PidStore_Backend_EraseSector(uint32_t sector);
#endif

#endif /* PID_STORE_H */

