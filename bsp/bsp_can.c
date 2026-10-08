#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* 调试器可以直接读取此全局变量，不依赖任务局部变量或串口输出。 */
volatile dart_can_diag_t dart_can_diag;

/**
 * @brief 配置并启动两路 CAN 总线的全接收滤波器。
 *
 * 本工程在 CAN1 上接收达妙电机等设备反馈，在 CAN2 上接收扳机、左右推板和
 * Yaw 电机反馈。当前过滤规则允许全部标准帧进入 FIFO0，具体报文标识由接收
 * 回调函数继续分发。
 *
 * STM32F4 的 CAN1 与 CAN2 共用过滤器组，因此必须在配置任意一路 CAN 之前先
 * 确定过滤器组分界。本函数把 0～13 号过滤器组分配给 CAN1，从 14 号开始分配
 * 给 CAN2，避免未初始化的分界值导致 CAN2 收不到核心电机反馈。
 *
 * @note 本函数必须在 MX_CAN1_Init() 和 MX_CAN2_Init() 成功之后、FreeRTOS
 *       任务开始之前调用。
 *
 * @return 无返回值。HAL 初始化失败时，相关 CAN 总线不会产生有效反馈，业务
 *         状态机会在反馈超时后进入故障锁定状态。
 */
void can_filter_init(void)
{
    /*
     * HAL 会读取结构中的 SlaveStartFilterBank。原代码在配置 CAN1 时尚未给该字段赋值，
     * 栈中的随机值可能把滤波器组错误地划分给 CAN1/CAN2，导致 CAN2 电机反馈完全收不到。
     * 先整体清零并统一指定 14 号为 CAN2 起始滤波器组，再分别配置两条总线。
     */
    dart_can_diag.init_calls++;
    CAN_FilterTypeDef can_filter_st = {0};
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;
    dart_can_diag.can1_filter_result = HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    dart_can_diag.can1_start_result = HAL_CAN_Start(&hcan1);
    dart_can_diag.can1_notify_result = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.FilterBank = 14;
    dart_can_diag.can2_filter_result = HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    dart_can_diag.can2_start_result = HAL_CAN_Start(&hcan2);
    dart_can_diag.can2_notify_result = HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}

/**
 * @brief 保存反馈离线瞬间的 CAN2 状态，供调试器事后读取。
 *
 * CAN 错误寄存器和接收 FIFO 状态会随总线活动变化，因此只在状态机首次锁定
 * 反馈离线故障时调用。寄存器读取不会清除错误或消费接收帧。
 *
 * @param missing_axis_mask 故障瞬间四个核心轴的离线位图。
 * @return 无返回值；快照写入全局 dart_can_diag。
 */
void dart_can_diag_capture(uint32_t missing_axis_mask)
{
    dart_can_diag.fault_missing_axis_mask = missing_axis_mask;
    dart_can_diag.fault_can2_state = hcan2.State;
    dart_can_diag.fault_can2_error = hcan2.ErrorCode;
    dart_can_diag.fault_can2_esr = CAN2->ESR;
    dart_can_diag.fault_can2_rf0r = CAN2->RF0R;
    dart_can_diag.fault_can2_ier = CAN2->IER;
    dart_can_diag.fault_can1_fmr = CAN1->FMR;
    dart_can_diag.fault_can1_fa1r = CAN1->FA1R;
}
