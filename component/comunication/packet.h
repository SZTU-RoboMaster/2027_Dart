#ifndef DART_PACKET_H
#define DART_PACKET_H

#include <stdint.h>

/**
 * @brief 把业务负载封装为完整协议帧并提交给 UsbTask。
 *
 * 函数生成帧头、序列号、命令字、两级循环冗余校验和帧尾，然后把完整帧复制进静态发送
 * 队列。它不会直接访问 USB 外设。当前兼容接口不返回错误，非法长度或队列满时丢弃本帧。
 *
 * @param command_id 业务命令字。
 * @param buffer 待封装负载首地址；函数返回后可立即复用。
 * @param length 负载有效字节数，不能超过协议单帧容量。
 */
void rm_queue_data(uint16_t command_id, const void *buffer, uint16_t length);

#endif /* DART_PACKET_H */
