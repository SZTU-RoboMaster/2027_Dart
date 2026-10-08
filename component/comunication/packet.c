#include "packet.h"

#include <string.h>

#include "CRC8_CRC16.h"
#include "usb_task.h"
#include "../../application/A_Dart/protocol_shaob.h"

/*
 * USB 协议序列化器
 * =================
 *
 * 输出布局：
 *   [5 字节帧头(含 CRC8)] [2 字节命令 ID] [N 字节负载]
 *   [2 字节 CRC16] [0x0D 0x0A]
 *
 * 本模块只构造一帧并复制到 UsbTask 静态队列，不直接调用 CDC 驱动。因此多个业务
 * 调用者不会并发操作 USB 端点，实际发送长度也随队列元素一起保存。
 */

/**
 * @brief 按机器人通信协议封装一帧数据并提交给 UsbTask。
 *
 * 本函数依次生成帧头、CRC8、命令字、负载、CRC16 和帧尾，然后把实际帧长连同数据
 * 一起复制到静态 USB 发送队列。函数不会直接访问 USB CDC 端点，因此调用者不会和
 * UsbTask 发生端点并发。非法长度或队列拥塞时直接放弃当前帧。
 *
 * @param command_id 协议命令字，按现有协议的小端格式写入。
 * @param payload 待发送负载的首地址，必须与 `payload_length` 对应。
 * @param payload_length 负载字节数，不包含帧头、命令字、校验和帧尾。
 */
static void encode_and_enqueue(uint16_t command_id,
                               const void *payload,
                               uint16_t payload_length)
{
    /* 最大长度同时考虑协议开销和 UsbTask 单帧存储上限。 */
    const uint16_t protocol_overhead =
        (uint16_t)(REF_HEADER_CRC_CMDID_LEN + sizeof(msg_end_info));
    if (payload == NULL || payload_length > (USB_TX_MAX_FRAME_SIZE - protocol_overhead)) {
        return;
    }

    uint8_t frame[USB_TX_MAX_FRAME_SIZE] = {0};
    uint16_t index = 0U;
    static uint8_t sequence;

    /* 帧头必须按 1 字节对齐协议结构生成，CRC8 覆盖整个固定帧头。 */
    frame_header_struct_t header = {
        .SOF = HEADER_SOF,
        .data_length = payload_length,
        .seq = sequence++,
    };
    append_CRC8_check_sum((uint8_t *)&header, sizeof(header));
    memcpy(frame + index, &header, sizeof(header));
    index += (uint16_t)sizeof(header);

    /* uint16_t 命令字按协议原有小端格式复制，避免未对齐写入。 */
    memcpy(frame + index, &command_id, sizeof(command_id));
    index += (uint16_t)sizeof(command_id);

    memcpy(frame + index, payload, payload_length);
    index += payload_length;

    /* append 函数把最后两个预留字节写为 CRC16。 */
    const uint16_t crc_frame_length =
        (uint16_t)(REF_HEADER_CRC_CMDID_LEN + payload_length);
    append_CRC16_check_sum(frame, crc_frame_length);
    index += (uint16_t)sizeof(uint16_t);

    const msg_end_info end_marker = { .end1 = END1_SOF, .end2 = END2_SOF };
    memcpy(frame + index, &end_marker, sizeof(end_marker));
    index += (uint16_t)sizeof(end_marker);

    /* 50 ms 是队列拥塞上限；队列满时丢弃本帧，不阻塞控制任务。 */
    (void)usb_tx_enqueue(frame, index, 50U);
}

/**
 * @brief 向统一 USB 发送通道提交一条协议消息。
 *
 * 该函数保留原工程的调用名称，内部已经改为“封装后入队”的非直接发送方式。它不保证
 * 数据在返回前已经上总线；UsbTask 会在 USB 空闲时依次发送队列中的完整帧。
 *
 * @param command_id 协议命令字。
 * @param buffer 待发送负载地址；为空时本次请求被忽略。
 * @param length 负载字节数，超过单帧容量时本次请求被忽略。
 */
void rm_queue_data(uint16_t command_id, const void *buffer, uint16_t length)
{
    encode_and_enqueue(command_id, buffer, length);
}
