#ifndef PROJECT_USB_TASK_H
#define PROJECT_USB_TASK_H

#include <stdbool.h>
#include <stdint.h>

#define USB_TX_MAX_FRAME_SIZE 128U

/**
 * @brief 创建 USB 发送静态队列。
 *
 * 函数允许重复调用，但只会实际创建一次。正常情况下在调度器启动前调用，UsbTask 启动
 * 时还会防御性调用一次。队列控制块和数据区均为静态存储。
 */
void usb_task_queue_init(void);

/**
 * @brief 将一帧完整数据复制到后台 USB 发送队列。
 *
 * 本函数不会直接调用 USB 端点，数据实际由 UsbTask 串行发送。函数会复制有效字节，
 * 因此返回后调用方可以立即复用原缓冲区。
 *
 * @param data 待发送帧首地址。
 * @param length 有效字节数，必须大于零且不超过单帧上限。
 * @param timeout_ms 队列满时允许等待的毫秒数，零表示不等待。
 *
 * @return
 * - true：完整帧已经进入发送队列；
 * - false：参数无效、队列未初始化或超时后队列仍满。
 */
bool usb_tx_enqueue(const uint8_t *data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief 初始化 USB 设备并持续消费发送队列。
 *
 * 端点忙时任务每毫秒退让一次并重试当前帧，保证帧顺序不被改变。函数不会返回。
 *
 * @param argument 静态任务表保留参数，当前实现忽略该值。
 */
void usb_task(void const *argument);

#endif //PROJECT_USB_TASK_H
