#include "dart_platform.h"

#include "dart_hw_legacy.h"
#include "bsp_flash.h"
#include "bsp_usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/*
 * STM32F427 板级实现
 * -----------------
 * 这是应用层中唯一把持久化存储和字节流绑定到具体 STM32/USB 驱动的文件。电机、GPIO
 * 和 PWM 的历史绑定暂时保留在 dart.c，并通过同一张平台操作表向业务层提供服务。
 */
/**
 * @brief 从固定参数扇区读取一个完整字节块。
 * @param data 接收数据的输出缓冲区。
 * @param size 读取字节数，必须为四字节整数倍。
 * @return true 表示完成读取；false 表示地址或长度参数无效。
 */
static bool stm32_parameters_read(void *data, uint32_t size)
{
    /* Flash 驱动以 32 位字为单位，拒绝无法整除的字节长度。 */
    if (data == NULL || size == 0U || (size % sizeof(uint32_t)) != 0U) return false;
    flash_read(FLASH_USER_START_ADDR, data, size / sizeof(uint32_t));
    return true;
}

/**
 * @brief 擦除参数扇区并写入一个完整参数块。
 * @param data 待写入数据首地址。
 * @param size 写入字节数，必须为四字节整数倍。
 * @return true 表示全部字写入成功；false 表示参数无效或底层写入失败。
 */
static bool stm32_parameters_write(const void *data, uint32_t size)
{
    /* 链接脚本已把扇区 11 从程序区剥离，专门保存参数。 */
    if (data == NULL || size == 0U || (size % sizeof(uint32_t)) != 0U) return false;
    flash_erase_address(FLASH_USER_START_ADDR, 1U);
    return flash_write_single_address(FLASH_USER_START_ADDR,
                                      (uint32_t *)(uintptr_t)data,
                                      size / sizeof(uint32_t)) == 0;
}

/**
 * @brief 初始化当前板卡的 USB 设备栈。
 * @return true 表示初始化调用已经完成。
 */
static bool stm32_usb_init(void)
{
    /* 只有 UsbTask 调用本函数，避免 USB 设备被重复初始化。 */
    MX_USB_DEVICE_Init();
    return true;
}

/**
 * @brief 尝试向 USB 虚拟串口提交一帧数据。
 * @param data 待发送数据首地址。
 * @param length 有效字节数。
 * @return 发送已接受、端点忙或底层错误三种平台无关结果之一。
 */
static dart_stream_result_t stm32_usb_write(const uint8_t *data, uint16_t length)
{
    uint8_t result = CDC_Transmit_FS((uint8_t *)(uintptr_t)data, length);
    if (result == USBD_OK) return DART_STREAM_OK;
    if (result == USBD_BUSY) return DART_STREAM_BUSY;
    return DART_STREAM_ERROR;
}

/**
 * @brief 配置裁判串口直接存储器访问接收。
 * @param buffer 长期有效的接收缓冲区。
 * @param length 缓冲区容量。
 * @return true 表示接收已经启动；false 表示参数无效。
 */
static bool stm32_referee_rx_start(uint8_t *buffer, uint16_t length)
{
    /* DMA 和中断配置属于板级细节，协议解析仍在 DecodeTask 中完成。 */
    if (buffer == NULL || length == 0U) return false;
    usart6_init(buffer, length);
    return true;
}

static const dart_platform_ops_t stm32_ops = {
    .init = dart_hw_legacy_init,
    .now_ms = dart_hw_legacy_now_ms,
    .sample_feedback = dart_hw_legacy_sample_feedback,
    .apply = dart_hw_legacy_apply,
    .service = dart_hw_legacy_service,
    .reset_control_state = dart_hw_legacy_reset_control_state,
    .parameters_read = stm32_parameters_read,
    .parameters_write = stm32_parameters_write,
    .usb_init = stm32_usb_init,
    .usb_write = stm32_usb_write,
    .referee_rx_start = stm32_referee_rx_start,
    .zero_axis = dart_hw_legacy_zero_axis,
    .gate_set = dart_hw_legacy_gate_set,
    .gate_close_all = dart_hw_legacy_gate_close_all,
};

/**
 * @brief 获取当前 STM32F427 板卡的完整操作表。
 * @return 静态只读操作表地址，程序运行期间始终有效。
 */
const dart_platform_ops_t *dart_platform_stm32_get(void)
{
    return &stm32_ops;
}
