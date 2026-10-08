#ifndef DART_PLATFORM_H
#define DART_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#include "dart_topics.h"

typedef enum {
    DART_STREAM_OK = 0,  /* 本次字节流发送已经被底层接受。 */
    DART_STREAM_BUSY,   /* 底层仍在发送上一帧，调用方稍后重试。 */
    DART_STREAM_ERROR   /* 外设未就绪或发生不可重试的底层错误。 */
} dart_stream_result_t;

/*
 * 完整的板级移植接口
 * ------------------
 * 业务模块只依赖这张操作表。移植到其他 STM32 时，只需重新映射 HAL 句柄、引脚、CAN
 * 标识符、Flash 和字节流，dart_sm 与换弹策略无需修改。除非实现另有明确说明，所有
 * 回调都在任务上下文执行。
 */
typedef struct {
    /* 初始化当前板卡使用的电机控制器和运行时资源，不创建业务任务。 */
    bool (*init)(void);
    /* 返回单调递增的毫秒时钟；允许自然回绕。 */
    uint32_t (*now_ms)(void);
    /* 在同一控制周期内生成一份完整反馈快照。 */
    void (*sample_feedback)(dart_feedback_t *feedback);
    /* 把状态机输出写入板级控制目标，不应在此阻塞等待电机到位。 */
    void (*apply)(const dart_actuator_command_t *command);
    /* 执行 PID 计算并发送一周期 CAN/PWM 输出。 */
    void (*service)(void);
    /* 完成回零或恢复事务后清除 PID、滤波器和旧目标等运行历史。 */
    void (*reset_control_state)(void);
    /* 从板载非易失存储读取完整参数块；读取失败时返回假。 */
    bool (*parameters_read)(void *data, uint32_t size);
    /* 原子保存完整参数块；长度必须是 32 位字的整数倍。 */
    bool (*parameters_write)(const void *data, uint32_t size);
    /* 只初始化一次 USB 设备栈；重复调用应保持幂等。 */
    bool (*usb_init)(void);
    /* 提交一帧 USB 数据，忙时由 UsbTask 保留队首并重试。 */
    dart_stream_result_t (*usb_write)(const uint8_t *data, uint16_t length);
    /* 启动裁判串口的中断或 DMA 接收。 */
    bool (*referee_rx_start)(uint8_t *buffer, uint16_t length);
    /* 把指定轴当前编码器位置登记为机械零点。 */
    void (*zero_axis)(dart_axis_t axis);
    /* 设置当前换弹机构的一扇舵机门；拆机版本必须安全忽略。 */
    void (*gate_set)(uint8_t index, bool open);
    /* 关闭全部换弹舵机门；拆机版本不得访问不存在的外设。 */
    void (*gate_close_all)(void);
} dart_platform_ops_t;

/**
 * @brief 获取当前 STM32F427 板卡的完整平台操作表。
 *
 * 操作表保存在静态只读存储中，集中映射系统时钟、执行器、反馈、参数存储和通信外设。
 * 业务层不得绕过该表直接持有 HAL 句柄。移植到其他芯片时提供同类型的新操作表即可。
 *
 * @return STM32F427 平台操作表地址，程序运行期间始终有效且无需释放。
 */
const dart_platform_ops_t *dart_platform_stm32_get(void);

#endif
