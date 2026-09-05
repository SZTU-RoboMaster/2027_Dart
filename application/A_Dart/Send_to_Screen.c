//
// 淘晶驰串口屏发送模块
// 通信协议：ASCII 指令 + 0xFF 0xFF 0xFF 结束符
// 串口：UART7（PE8 TX / PE7 RX），9600 8N1（与 Dart.HMI 的 baud=9600 一致）
//

#include "Send_to_Screen.h"

#include <stdio.h>
#include <string.h>

#include "usart.h"

// 淘晶驰 ASCII 指令结束符
static const uint8_t TJC_END[3] = {0xFF, 0xFF, 0xFF};

/**
 * @brief 发送任意 ASCII 指令到串口屏
 * @param msg 指令字符串，例如 page 0
 *
 * @note
 * 这里使用阻塞发送，避免 DMA 继续访问已经失效的局部数组。
 * 如果以后需要重新改成 DMA，必须使用生命周期足够长的静态缓冲区，
 * 并且需要等待上一次 DMA 发送完成。
 */
void Send_to_Screen(const char *msg)
{
    uint8_t cmd[200];
    size_t len;

    if (msg == NULL)
    {
        return;
    }

    len = strlen(msg);

    // 为三个结束字节预留空间
    if (len > sizeof(cmd) - sizeof(TJC_END))
    {
        len = sizeof(cmd) - sizeof(TJC_END);
    }

    memcpy(cmd, msg, len);
    memcpy(cmd + len, TJC_END, sizeof(TJC_END));

    (void)HAL_UART_Transmit(&huart7,
                            cmd,
                            (uint16_t)(len + sizeof(TJC_END)),
                            100);
}

/**
 * @brief 发送触发距离到串口屏
 * @param outpost 前哨站的四个触发距离
 * @param base 基地的四个触发距离
 *
 * @note
 * 对应屏幕控件：
 * 前哨站为 Outpost.t6 到 Outpost.t9；
 * 基地为 Base.t6 到 Base.t9。
 */
void Send_to_Screen_Data(const float *outpost, const float *base)
{
    char cmd[80];
    int written;

    if (outpost == NULL || base == NULL)
    {
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        int text_id = 6 + i;

        written = snprintf(cmd,
                           sizeof(cmd),
                           "Outpost.t%d.txt=\"%.4f\"",
                           text_id,
                           outpost[i]);
        if (written > 0 && (size_t)written < sizeof(cmd))
        {
            Send_to_Screen(cmd);
        }

        written = snprintf(cmd,
                           sizeof(cmd),
                           "Base.t%d.txt=\"%.4f\"",
                           text_id,
                           base[i]);
        if (written > 0 && (size_t)written < sizeof(cmd))
        {
            Send_to_Screen(cmd);
        }
    }
}

/**
 * @brief 设置屏幕数值控件
 * @param comp_name 控件完整名称，例如 main.n0
 * @param value 要设置的数值
 */
void Send_to_Screen_Number(const char *comp_name, int32_t value)
{
    char cmd[80];

    if (comp_name == NULL)
    {
        return;
    }

    (void)snprintf(cmd,
                   sizeof(cmd),
                   "%s.val=%ld",
                   comp_name,
                   (long)value);
    Send_to_Screen(cmd);
}

/**
 * @brief 切换串口屏页面
 * @param page_id 页面编号
 */
void Send_to_Screen_Page(uint8_t page_id)
{
    char cmd[24];

    (void)snprintf(cmd, sizeof(cmd), "page %u", (unsigned int)page_id);
    Send_to_Screen(cmd);
}

/**
 * @brief 将一个 32 位整数写入串口屏掉电存储区
 * @param address 掉电存储区地址
 * @param value 要保存的整数
 *
 * @note
 * sys0 在这里作为临时寄存器使用，因此屏幕工程不要同时修改 sys0。
 * wepo 每次保存四个字节，调用者应保证地址按四字节对齐。
 */
void Send_to_Screen_Eeprom_Write(uint16_t address, int32_t value)
{
    char cmd[48];

    // 先把待保存的数据放入系统变量 sys0
    (void)snprintf(cmd, sizeof(cmd), "sys0=%ld", (long)value);
    Send_to_Screen(cmd);

    // 再把 sys0 的四个字节写入指定掉电存储地址
    (void)snprintf(cmd,
                   sizeof(cmd),
                   "wepo sys0,%u",
                   (unsigned int)address);
    Send_to_Screen(cmd);
}

/**
 * @brief 请求读取串口屏掉电存储区中的一个 32 位整数
 * @param address 掉电存储区地址
 *
 * @note
 * repo 先把数据恢复到 sys0，get sys0 再让屏幕返回标准数值帧：
 * 0x71 + 四字节小端整数 + 0xFF 0xFF 0xFF。
 */
void Send_to_Screen_Eeprom_Read_Request(uint16_t address)
{
    char cmd[48];

    (void)snprintf(cmd,
                   sizeof(cmd),
                   "repo sys0,%u",
                   (unsigned int)address);
    Send_to_Screen(cmd);

    Send_to_Screen("get sys0");
}
