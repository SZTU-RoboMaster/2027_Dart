#ifndef STM32_FREERTOS_DECODE_H
#define STM32_FREERTOS_DECODE_H

#include <stdint.h>

/*
 * DecodeTask 的外部接口
 * --------------------
 * USB CDC 与裁判串口中断只负责复制字节并唤醒任务，不在中断中执行 CRC、结构体解析或
 * Topic 发布。所有协议处理都在 DecodeTask 上下文完成，避免中断执行时间不可控。
 */

/**
 * @brief 初始化 USB 接收流缓冲和裁判串口接收链路。
 *
 * 函数使用静态存储并保持幂等，可在系统初始化和 DecodeTask 启动阶段重复调用。它只
 * 建立传输资源，不解析协议数据。
 */
void decode_transport_init(void);

/**
 * @brief 从中断上下文唤醒 DecodeTask。
 *
 * 任务尚未创建时函数安全返回；任务存在时发送直接任务通知，并按需请求中断退出后调度。
 */
void decode_task_wake_from_isr(void);

/**
 * @brief 把一段 USB 接收数据复制到 DecodeTask 的静态流缓冲。
 *
 * 本函数供 USB 接收回调在中断相关上下文调用，不执行循环冗余校验和协议分发。
 *
 * @param buffer 本次收到的数据首地址，返回后可由 USB 驱动复用。
 * @param length 本次收到的有效字节数。
 */
void usb_receiver(uint8_t *buffer, uint32_t length);

/**
 * @brief 运行 USB 视觉与裁判系统的事件驱动解析任务。
 *
 * 每次唤醒先排空两个输入源，再阻塞等待下一次通知。函数不会返回。
 *
 * @param argument 静态任务表保留参数，当前实现忽略该值。
 */
void decode_task(void const *argument);

#endif /* STM32_FREERTOS_DECODE_H */
