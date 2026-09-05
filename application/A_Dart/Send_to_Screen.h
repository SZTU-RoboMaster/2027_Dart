//
// 淘晶驰串口屏发送模块头文件
//

#ifndef DART_2026_SEND_TO_SCREEN_H
#define DART_2026_SEND_TO_SCREEN_H

#include <stdint.h>

// 发送任意 ASCII 指令，函数会自动添加 0xFF 0xFF 0xFF 结束符
void Send_to_Screen(const char *msg);

// 发送 8 个触发距离到屏幕，前哨站 4 个、基地 4 个
void Send_to_Screen_Data(const float *outpost, const float *base);

// 设置数值控件，例如 Send_to_Screen_Number("main.n0", 123)
void Send_to_Screen_Number(const char *comp_name, int32_t value);

// 切换页面，例如 Send_to_Screen_Page(0)
void Send_to_Screen_Page(uint8_t page_id);

// 将一个 32 位整数写入串口屏掉电存储区
void Send_to_Screen_Eeprom_Write(uint16_t address, int32_t value);

// 请求串口屏读取一个 32 位整数，结果通过 0x71 标准数值返回帧送回 MCU
void Send_to_Screen_Eeprom_Read_Request(uint16_t address);

#endif // DART_2026_SEND_TO_SCREEN_H