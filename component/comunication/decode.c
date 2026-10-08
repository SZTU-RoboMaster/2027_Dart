#include "decode.h"

#include <stddef.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "fifo.h"
#include "main.h"
#include "stream_buffer.h"
#include "topic_bus.h"
#include "../../application/A_Dart/dart.h"
#include "../../application/A_Dart/protocol_shaob.h"
#include "../../application/Referee_system/Referee.h"

/*
 * USB 视觉与裁判数据解析任务
 * ==========================
 *
 * 接收链路分为两个阶段：
 *
 *   USB CDC ISR -> 静态 StreamBuffer -> 字节 FIFO -> USB 帧解析状态机
 *   裁判 UART ISR -> 裁判静态帧队列 ----------------> 裁判帧解析器
 *
 * ISR 只复制数据并发送任务通知，不做 CRC、结构体复制或 Topic 发布。DecodeTask 被任一
 * 来源唤醒后先排空当前积压数据，再进入阻塞，从而兼顾响应速度和中断实时性。
 *
 * USB 协议帧先用 CRC8 校验固定帧头，再用 CRC16 校验命令 ID 与负载。只有完整且合法
 * 的视觉控制帧才会更新 TOPIC_VISION_TARGET。
 */

#define USB_DECODE_BUFFER_SIZE 512U
#define USB_DECODE_CHUNK_SIZE   64U
#define Q_SERIES_YAW_OFFSET     0.5f

/*
 * usb_fifo 由 DecodeTask 独占，用于保存跨 StreamBuffer 分片的不完整协议帧。
 * StreamBuffer 的写端在 USB ISR，读端只有 DecodeTask，符合单写者/单读者约束。
 */
static fifo_s_t usb_fifo;
static uint8_t usb_fifo_storage[USB_DECODE_BUFFER_SIZE];
static StreamBufferHandle_t usb_rx_stream;
static StaticStreamBuffer_t usb_rx_stream_control;
static uint8_t usb_rx_stream_storage[USB_DECODE_BUFFER_SIZE];

/* ISR 通过该句柄发送直接任务通知；任务创建之前允许为 NULL。 */
static TaskHandle_t decode_task_handle;

/* 增量解包上下文跨多次任务唤醒保存，支持任意位置拆包。 */
static unpack_data_t usb_unpack;
static frame_header_struct_t received_header;

/*
 * 兼容旧观察代码的视觉结构体。它只在 DecodeTask 中写入；新的 Dart 业务只读取
 * TOPIC_VISION_TARGET，不直接依赖该全局对象。
 */
robot_ctrl_info_t robot_ctrl;

static void decode_usb_fifo(void);
static uint16_t dispatch_usb_frame(const uint8_t *frame);

/**
 * @brief 持续解析 USB 视觉数据和裁判串口数据。
 *
 * 每次被通知后先排空流缓冲和裁判帧队列，再重新阻塞。解析过程在任务上下文完成，不占用
 * 外设中断时间。函数不会返回。
 *
 * @param argument 任务保留参数，当前实现不使用。
 */
void decode_task(void const *argument)
{
    (void)argument;
    decode_task_handle = xTaskGetCurrentTaskHandle();
    fifo_s_init(&usb_fifo, usb_fifo_storage, USB_DECODE_BUFFER_SIZE);
    decode_transport_init();

    for (;;) {
        uint8_t chunk[USB_DECODE_CHUNK_SIZE];
        size_t received_length;

        /*
         * StreamBuffer 可能把一帧分成多段，也可能一次包含多帧。先全部搬入持久 FIFO，
         * 再由状态机逐字节处理，任务栈上的 chunk 可在每轮安全复用。
         */
        while ((received_length = xStreamBufferReceive(usb_rx_stream,
                                                        chunk,
                                                        sizeof(chunk),
                                                        0U)) > 0U) {
            fifo_s_puts(&usb_fifo, (char *)chunk, (uint16_t)received_length);
        }

        decode_usb_fifo();
        referee_decode_pending();

        /*
         * pdTRUE 清除已经累计的通知计数。如果 ISR 在排空数据与此调用之间到达，本调用
         * 会立刻返回而不会丢失唤醒；无新数据时则永久阻塞，不消耗 CPU。
         */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

/**
 * @brief 创建静态 USB 流缓冲并启动裁判接收链路。
 *
 * 函数保持幂等，只建立资源，不消费任何协议数据。
 */
void decode_transport_init(void)
{
    /* 系统初始化和 DecodeTask 都可调用，静态缓冲只创建一次。 */
    if (usb_rx_stream == NULL) {
        usb_rx_stream = xStreamBufferCreateStatic(sizeof(usb_rx_stream_storage),
                                                  1U,
                                                  usb_rx_stream_storage,
                                                  &usb_rx_stream_control);
    }

    /* 裁判串口拥有自己的 DMA/队列，接口在 Referee 模块内部保持幂等。 */
    referee_transport_init();
}

/**
 * @brief 从中断上下文向 DecodeTask 发送一次直接任务通知。
 *
 * 任务句柄尚未建立时安全返回；否则按需触发中断退出后的任务切换。
 */
void decode_task_wake_from_isr(void)
{
    /* 任务尚未创建时没有可唤醒对象；来源队列仍负责保存已接收数据。 */
    if (decode_task_handle == NULL) return;

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(decode_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 从 USB 回调向静态流缓冲复制一段接收数据。
 *
 * @param buffer 本次接收缓冲区，函数返回后可由 USB 驱动复用。
 * @param length 本次有效字节数；零长度会被忽略。
 */
void usb_receiver(uint8_t *buffer, uint32_t length)
{
    /*
     * USB CDC 回调运行在中断相关上下文，只允许使用 FromISR API。StreamBuffer 会复制
     * 数据，因此回调返回后 USB 驱动可以立即复用原 buffer。
     */
    if (usb_rx_stream == NULL || buffer == NULL || length == 0U) return;

    BaseType_t higher_priority_task_woken = pdFALSE;
    (void)xStreamBufferSendFromISR(usb_rx_stream,
                                   buffer,
                                   length,
                                   &higher_priority_task_woken);
    if (decode_task_handle != NULL) {
        vTaskNotifyGiveFromISR(decode_task_handle, &higher_priority_task_woken);
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/*
 * USB 增量解包状态机
 * ------------------
 * 每次只消费一个字节，因此天然支持粘包、拆包和帧前噪声。usb_unpack 中的 step、index
 * 和 protocol_packet 在调用之间持续保存。长度或 CRC 错误只丢弃当前候选帧，然后
 * 重新搜索 0xA5，不会清空后续已经进入 FIFO 的字节。
 */
/**
 * @brief 逐字节推进 USB 增量解包状态机。
 *
 * 函数排空当前先进先出缓冲，支持粘包、拆包和帧前噪声。校验失败只丢弃当前候选帧，
 * 不会清空后续已经接收的数据。
 */
static void decode_usb_fifo(void)
{
    unpack_data_t *unpack = &usb_unpack;

    while (fifo_s_used(&usb_fifo)) {
        const uint8_t byte = fifo_s_get(&usb_fifo);

        switch (unpack->unpack_step) {
            case STEP_HEADER_SOF:
                /* 非 SOF 字节属于前导噪声，直接忽略。 */
                if (byte == HEADER_SOF) {
                    unpack->index = 0U;
                    unpack->protocol_packet[unpack->index++] = byte;
                    unpack->unpack_step = STEP_LENGTH_LOW;
                } else {
                    unpack->index = 0U;
                }
                break;

            case STEP_LENGTH_LOW:
                unpack->data_len = byte;
                unpack->protocol_packet[unpack->index++] = byte;
                unpack->unpack_step = STEP_LENGTH_HIGH;
                break;

            case STEP_LENGTH_HIGH:
                unpack->data_len |= (uint16_t)((uint16_t)byte << 8U);
                unpack->protocol_packet[unpack->index++] = byte;

                /* 为固定帧头、命令 ID 和尾部 CRC16 留出空间，防止数组越界。 */
                if (unpack->data_len <
                    (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
                    unpack->unpack_step = STEP_FRAME_SEQ;
                } else {
                    unpack->unpack_step = STEP_HEADER_SOF;
                    unpack->index = 0U;
                }
                break;

            case STEP_FRAME_SEQ:
                /* 序列号属于 CRC8 覆盖的固定帧头。 */
                unpack->protocol_packet[unpack->index++] = byte;
                unpack->unpack_step = STEP_HEADER_CRC8;
                break;

            case STEP_HEADER_CRC8:
                unpack->protocol_packet[unpack->index++] = byte;
                if (unpack->index == REF_PROTOCOL_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(unpack->protocol_packet,
                                              REF_PROTOCOL_HEADER_SIZE)) {
                        unpack->unpack_step = STEP_DATA_CRC16;
                    } else {
                        /* 帧头不可信，不能继续采用其中的长度字段。 */
                        unpack->unpack_step = STEP_HEADER_SOF;
                        unpack->index = 0U;
                    }
                }
                break;

            case STEP_DATA_CRC16: {
                const uint16_t frame_length =
                    (uint16_t)(REF_HEADER_CRC_CMDID_LEN + unpack->data_len);
                if (unpack->index < frame_length) {
                    unpack->protocol_packet[unpack->index++] = byte;
                }

                if (unpack->index >= frame_length) {
                    /* 在分发前复位解析状态，使下一帧即使紧邻当前帧也能正常识别。 */
                    unpack->unpack_step = STEP_HEADER_SOF;
                    unpack->index = 0U;
                    if (verify_CRC16_check_sum(unpack->protocol_packet, frame_length)) {
                        (void)dispatch_usb_frame(unpack->protocol_packet);
                    }
                }
                break;
            }

            default:
                /* 内存破坏或非法枚举的防御出口。 */
                unpack->unpack_step = STEP_HEADER_SOF;
                unpack->index = 0U;
                break;
        }
    }
}

/* 将已通过 CRC8/CRC16 的 USB 帧转换为强类型 Topic。 */
/**
 * @brief 分发一帧已经通过两级循环冗余校验的 USB 数据。
 *
 * 当前只把视觉控制帧转换为视觉目标主题，未知命令字安全忽略。
 *
 * @param frame 完整协议帧首地址。
 * @return 按帧头计算出的完整帧字节数，供后续诊断统计使用。
 */
static uint16_t dispatch_usb_frame(const uint8_t *frame)
{
    uint16_t index = 0U;
    uint16_t command_id = 0U;

    /* memcpy 避免 Cortex-M 对未对齐结构体和 uint16_t 的直接访问。 */
    memcpy(&received_header, frame, sizeof(received_header));
    index += (uint16_t)sizeof(received_header);
    memcpy(&command_id, frame + index, sizeof(command_id));
    index += (uint16_t)sizeof(command_id);

    switch (command_id) {
        case CHASSIS_CTRL_CMD_ID: {
            /* 长度不足时不得读取跨越协议帧边界的数据。 */
            if (received_header.data_length < sizeof(robot_ctrl)) break;

            memcpy(&robot_ctrl, frame + index, sizeof(robot_ctrl));
            if (robot_ctrl.target_lock == 0) {
                robot_ctrl.yaw = 0.0f;
            } else {
                /* 当前 Q 系列四发共用同一机械零偏；以后应迁移到标定参数。 */
                robot_ctrl.yaw += Q_SERIES_YAW_OFFSET;
            }

            /* DartTask 使用接收时间执行 200 ms 视觉新鲜度保护。 */
            const vision_target_t target = {
                .yaw_error = robot_ctrl.yaw,
                .target_locked = robot_ctrl.target_lock != 0,
                .timestamp_ms = HAL_GetTick(),
            };
            (void)topic_publish(TOPIC_VISION_TARGET, &target);
            break;
        }

        default:
            /* 未支持的命令已经通过 CRC，可安全忽略。 */
            break;
    }

    /* 返回整帧长度，保留给未来统计/诊断使用。 */
    index = (uint16_t)(index + received_header.data_length + sizeof(uint16_t));
    return index;
}
