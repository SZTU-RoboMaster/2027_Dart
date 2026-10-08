#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "../application/A_Dart/struct_typedef.h"

/**
 * @brief CAN 接收链路的全局故障诊断快照。
 *
 * 在调试器的全局变量窗口查看 dart_can_diag。初始化结果与接收计数持续更新；
 * fault_ 开头的字段只在反馈离线故障刚出现时锁存，避免故障之后的总线变化
 * 覆盖真正发生故障时的证据。所有计数和寄存器值都使用原始整数，便于直接查看。
 */
typedef struct {
    volatile uint32_t init_calls;             /* CAN 过滤器初始化执行次数，正常为一。 */
    volatile uint32_t can1_filter_result;     /* CAN1 过滤器配置结果，零表示成功。 */
    volatile uint32_t can1_start_result;      /* CAN1 启动结果，零表示成功。 */
    volatile uint32_t can1_notify_result;     /* CAN1 接收中断使能结果，零表示成功。 */
    volatile uint32_t can2_filter_result;     /* CAN2 过滤器配置结果，零表示成功。 */
    volatile uint32_t can2_start_result;      /* CAN2 启动结果，零表示成功。 */
    volatile uint32_t can2_notify_result;     /* CAN2 接收中断使能结果，零表示成功。 */
    volatile uint32_t can1_rx_count;          /* CAN1 收到的帧数，用于对照两条总线。 */
    volatile uint32_t can2_irq_count;         /* CAN2 接收回调进入次数。 */
    volatile uint32_t can2_rx_count;          /* CAN2 成功读出的帧数。 */
    volatile uint32_t can2_rx_error_count;    /* CAN2 从 FIFO0 取帧失败次数。 */
    volatile uint32_t can2_unknown_count;     /* CAN2 收到但未匹配电机标识的帧数。 */
    volatile uint32_t can2_last_id;           /* CAN2 最近收到的标准帧标识。 */
    volatile uint32_t fault_missing_axis_mask;/* 故障时离线轴位图，位号等于轴编号。 */
    volatile uint32_t fault_can2_state;       /* 故障时 HAL CAN2 状态，二表示监听中。 */
    volatile uint32_t fault_can2_error;       /* 故障时 HAL CAN2 错误码。 */
    volatile uint32_t fault_can2_esr;         /* 故障时 CAN2 错误状态寄存器。 */
    volatile uint32_t fault_can2_rf0r;        /* 故障时 CAN2 FIFO0 状态寄存器。 */
    volatile uint32_t fault_can2_ier;         /* 故障时 CAN2 中断使能寄存器。 */
    volatile uint32_t fault_can1_fmr;         /* 故障时共用过滤器的分界寄存器。 */
    volatile uint32_t fault_can1_fa1r;        /* 故障时共用过滤器的激活位图。 */
} dart_can_diag_t;

extern volatile dart_can_diag_t dart_can_diag;

/**
 * @brief 配置过滤器、启动 CAN1 和 CAN2，并开启 FIFO0 接收中断。
 *
 * 两路 CAN 共用 STM32F4 的过滤器资源。本函数负责固定过滤器组分界，并为两路
 * 总线安装全接收过滤器。调用前必须已经完成 CAN 外设和对应 GPIO 的初始化。
 *
 * @return 无返回值。
 */
extern void can_filter_init(void);

/**
 * @brief 在首次核心反馈离线故障时锁存 CAN2 硬件状态。
 *
 * @param missing_axis_mask 缺失轴位图；扳机、左推板、右推板和 Yaw 分别使用
 *                          DART_AXIS_TRIGGER、DART_AXIS_PUSH_LEFT、
 *                          DART_AXIS_PUSH_RIGHT、DART_AXIS_YAW 对应的位。
 * @return 无返回值，结果保存在全局变量 dart_can_diag 的 fault_ 字段中。
 */
void dart_can_diag_capture(uint32_t missing_axis_mask);

#endif
