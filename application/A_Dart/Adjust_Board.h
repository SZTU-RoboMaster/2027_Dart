//
// 淘晶驰串口屏接收与参数备份模块头文件
//

#ifndef DART_2026_ADJUST_BOARD_H
#define DART_2026_ADJUST_BOARD_H

#include "stdbool.h"
#include "string.h"
#include "usart.h"
#include "dart.h"

// 调参屏 FreeRTOS 任务入口
extern void adjust_task(void const *pvParameters);

// 前哨站和基地共八个触发距离
// 下标 0～3 为前哨站，下标 4～7 为基地
extern fp32 outpost_base_value[8];


#define BUFF_SIZE                         128U

// 串口屏启动后等待一段时间再访问掉电存储区
#define SCREEN_STARTUP_DELAY_MS           500U

// 等待 get sys0 返回数值帧的最长时间
#define SCREEN_READ_TIMEOUT_MS            150U

// 每次 wepo 后留给串口屏执行掉电存储写入的时间
#define SCREEN_WRITE_DELAY_MS             8U

// 串口屏备份失败后的重试间隔
#define SCREEN_RETRY_INTERVAL_MS          3000U

// 同一组参数最多自动重试三次，防止通信异常时不断擦写
#define SCREEN_MAX_RETRY_COUNT            3U

// 浮点距离转换为定点整数时保留四位小数
#define SCREEN_DISTANCE_SCALE             10000.0

// 串口屏备份格式版本
#define SCREEN_BACKUP_VERSION             1U

// “DRT1”的十六进制表示，用于判断存储区是否包含有效数据
#define SCREEN_BACKUP_MAGIC               0x44525431UL

// CRC 最高位清零，保证写入 sys0 时始终位于正整数范围
#define SCREEN_CRC_MASK                   0x7FFFFFFFUL

// 串口屏单份备份记录的起始地址
#define SCREEN_BACKUP_ADDRESS            0U

// 备份记录中各个字段的相对地址
#define SCREEN_MAGIC_OFFSET               0U
#define SCREEN_VERSION_OFFSET             4U
#define SCREEN_SEQUENCE_OFFSET            8U
#define SCREEN_DISTANCE_OFFSET            12U
#define SCREEN_CRC_OFFSET                 44U

#endif // DART_2026_ADJUST_BOARD_H
