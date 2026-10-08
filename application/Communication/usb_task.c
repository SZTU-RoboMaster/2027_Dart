#include "usb_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "dart_platform.h"

/*
 * USB 发送任务
 * ------------
 * 业务模块只负责把“数据指针 + 实际长度”提交到静态队列，UsbTask 串行调用平台层。
 * 这样可以避免多个任务同时进入 CDC_Transmit_FS，也修复旧代码固定长度发送导致的
 * 尾部脏数据问题。整个路径没有动态内存。
 */
#define USB_TX_QUEUE_DEPTH 8U

typedef struct {
    /* 每个队列元素自带有效长度，data 中 length 之后的内容不会被发送。 */
    uint16_t length;
    uint8_t data[USB_TX_MAX_FRAME_SIZE];
} usb_tx_frame_t;

static QueueHandle_t tx_queue;
static StaticQueue_t tx_queue_control;
static uint8_t tx_queue_storage[USB_TX_QUEUE_DEPTH * sizeof(usb_tx_frame_t)];

/**
 * @brief 使用静态控制块和静态数据区创建 USB 发送队列。
 *
 * 已创建时直接返回，因此系统初始化和任务入口可以安全重复调用。
 */
void usb_task_queue_init(void)
{
    /* FreeRTOS 静态队列由本文件永久持有存储和控制块。 */
    if (tx_queue == NULL) {
        tx_queue = xQueueCreateStatic(USB_TX_QUEUE_DEPTH,
                                     sizeof(usb_tx_frame_t),
                                     tx_queue_storage,
                                     &tx_queue_control);
    }
}

/**
 * @brief 复制一帧数据到 USB 后台发送队列。
 * @return true 表示已入队；false 表示参数无效、队列未初始化或等待超时。
 */
bool usb_tx_enqueue(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    /* 严格拒绝空帧和超长帧，防止 memcpy 越界。 */
    if (data == NULL || length == 0U || length > USB_TX_MAX_FRAME_SIZE || tx_queue == NULL) {
        return false;
    }
    usb_tx_frame_t frame = { .length = length };
    /* 队列保存 frame 的完整副本，因此调用者返回后可立即复用 data。 */
    memcpy(frame.data, data, length);
    return xQueueSend(tx_queue, &frame, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

/**
 * @brief 初始化 USB 并按先进先出顺序持续发送队列中的完整帧。
 *
 * 端点忙时保留当前帧并延时重试，函数不会返回。
 */
void usb_task(void const *argument)
{
    (void)argument;
    const dart_platform_ops_t *platform = dart_platform_stm32_get();
    usb_task_queue_init();
    /* USB 初始化失败属于系统配置错误，不能在没有传输能力时静默运行。 */
    configASSERT(platform->usb_init != NULL && platform->usb_write != NULL &&
                 platform->usb_init());

    for (;;) {
        usb_tx_frame_t frame;
        /* 没有待发数据时永久阻塞，不占用控制任务的 CPU 时间。 */
        if (xQueueReceive(tx_queue, &frame, portMAX_DELAY) == pdTRUE) {
            /* CDC 端点忙是正常短暂状态；每毫秒退让一次，避免忙等。 */
            while (platform->usb_write(frame.data, frame.length) == DART_STREAM_BUSY) {
                vTaskDelay(pdMS_TO_TICKS(1U));
            }
        }
    }
}
