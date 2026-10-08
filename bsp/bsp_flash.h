#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>

#include "../application/A_Dart/struct_typedef.h"

/*
 * STM32F427 双存储体扇区起始地址。
 * 当前参数区固定使用第一存储体第十一扇区；链接脚本必须同时把该扇区排除在程序区外。
 */
#define ADDR_FLASH_SECTOR_0  ((uint32_t)0x08000000) /* 第零扇区，十六千字节。 */
#define ADDR_FLASH_SECTOR_1  ((uint32_t)0x08004000) /* 第一扇区，十六千字节。 */
#define ADDR_FLASH_SECTOR_2  ((uint32_t)0x08008000) /* 第二扇区，十六千字节。 */
#define ADDR_FLASH_SECTOR_3  ((uint32_t)0x0800C000) /* 第三扇区，十六千字节。 */
#define ADDR_FLASH_SECTOR_4  ((uint32_t)0x08010000) /* 第四扇区，六十四千字节。 */
#define ADDR_FLASH_SECTOR_5  ((uint32_t)0x08020000) /* 第五扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_6  ((uint32_t)0x08040000) /* 第六扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_7  ((uint32_t)0x08060000) /* 第七扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_8  ((uint32_t)0x08080000) /* 第八扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_9  ((uint32_t)0x080A0000) /* 第九扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* 第十扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* 第十一扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_12 ((uint32_t)0x08100000) /* 第十二扇区，十六千字节。 */
#define ADDR_FLASH_SECTOR_13 ((uint32_t)0x08104000) /* 第十三扇区，十六千字节。 */
#define ADDR_FLASH_SECTOR_14 ((uint32_t)0x08108000) /* 第十四扇区，十六千字节。 */
#define ADDR_FLASH_SECTOR_15 ((uint32_t)0x0810C000) /* 第十五扇区，十六千字节。 */
#define ADDR_FLASH_SECTOR_16 ((uint32_t)0x08110000) /* 第十六扇区，六十四千字节。 */
#define ADDR_FLASH_SECTOR_17 ((uint32_t)0x08120000) /* 第十七扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_18 ((uint32_t)0x08140000) /* 第十八扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_19 ((uint32_t)0x08160000) /* 第十九扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_20 ((uint32_t)0x08180000) /* 第二十扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_21 ((uint32_t)0x081A0000) /* 第二十一扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_22 ((uint32_t)0x081C0000) /* 第二十二扇区，一百二十八千字节。 */
#define ADDR_FLASH_SECTOR_23 ((uint32_t)0x081E0000) /* 第二十三扇区，一百二十八千字节。 */

#define FLASH_USER_START_ADDR 0x080E0000U /* 飞镖参数块固定起始地址。 */
#define FLASH_END_ADDR        0x081FFFFFU /* 芯片内部存储器末地址。 */

/* 在不违反严格别名规则的情况下查看浮点数的原始三十二位表示。 */
typedef union {
    float floatValue;
    uint32_t uintValue;
} float_to_uint32;

/* 以下两个接口是旧浮点数组存储兼容层，新参数系统不应继续使用。 */
void Flash_Write_Data(float *data, uint32_t length);
void Flash_Read_Data(float *data, uint32_t length);

/* 从指定地址开始擦除连续扇区。 */
void flash_erase_address(uint32_t address, uint16_t sector_count);

/* 在当前地址所属扇区内写入若干个三十二位字，成功返回零。 */
int8_t flash_write_single_address(uint32_t start_address, uint32_t *data, uint32_t word_count);

/* 在显式地址范围内写入若干个三十二位字，成功返回零。 */
int8_t flash_write_muli_address(uint32_t start_address,
                                uint32_t end_address,
                                uint32_t *data,
                                uint32_t word_count);

/* 从存储器映射地址读取若干个三十二位字。 */
void flash_read(uint32_t address, uint32_t *data, uint32_t word_count);

/* 把存储器地址换算成硬件抽象层使用的扇区编号。 */
uint32_t ger_sector(uint32_t address);

/* 返回当前扇区之后的第一个地址，用作单扇区写入上界。 */
uint32_t get_next_flash_address(uint32_t address);

#endif /* BSP_FLASH_H */
