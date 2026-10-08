#include "remote.h"

#include <stdlib.h>

#include "main.h"
#include "bsp_usart.h"
#include "usart.h"

/*
 * DJI DBUS/SBUS 遥控器接收适配器
 * =============================
 * USART1 使用双缓冲 DMA 接收 18 字节遥控器帧，空闲中断确认本次长度后解析。该模块只
 * 更新 rc_ctrl 和 remote_last_update_ms；DartTask 再把拨杆边沿转换成 dart_command_t，
 * 因此遥控器不会直接修改业务状态机。
 */
#define RC_CHANNEL_ERROR_VALUE 700

RC_ctrl_t rc_ctrl;
volatile uint32_t remote_last_update_ms;

/* DMA ping-pong 缓冲：硬件写一个缓冲时，CPU 解析另一个。 */
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

static int16_t RC_abs(int16_t value);

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
 * @返回值         无
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

/* 在外设初始化完成后启动 USART1 双缓冲 DMA。 */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}


uint8_t RC_data_is_error(void)
{
    /* 任一通道越过物理范围或拨杆编码为 0 时，将整帧置为安全中立值。 */
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNEL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNEL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNEL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNEL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    /* 错误帧不得保留上一帧的发射输入。 */
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

/*
 * USART1 空闲中断：冻结当前 DMA 缓冲、读取实际长度、切换到另一缓冲并立即恢复 DMA。
 * 只有长度恰好为 18 字节的 DBUS 帧才进入解析器。
 */
void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* DMA 当前写 Memory 0：停流后切换到 Memory 1。 */

            /* 必须先关闭 DMA，才能可靠修改 NDTR 和 CT 位。 */
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            /* 实收长度 = 配置长度 - DMA 剩余计数。 */
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            /* 为下一缓冲恢复完整接收长度。 */
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /* CT=1 后 DMA 写 Memory 1。 */
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* DMA 当前写 Memory 1：同样冻结后切换回 Memory 0。 */
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /* CT=0 后 DMA 写 Memory 0。 */
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);

            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

/* int16_t 的轻量绝对值，用于通道范围检查。 */
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
 * @返回值         无
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3

    /* DBUS 各通道跨字节紧密打包，先还原 11 位原始值。 */
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right

    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 /* 拨轮 */

    /* 将协议中心值 1024 归一化到 0。 */
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    RC_data_is_error();
    if(rc_ctrl->rc.ch[2]<=10&&rc_ctrl->rc.ch[2]>=-10)
    {
        rc_ctrl->rc.ch[2]=0;
    }

    /* 单帧跳变超过 500 视为串扰，保留上一帧值；正常值更新 last_ch。 */
    for (int i = 0; i < 4; ++i) {
        if(abs((rc_ctrl->rc.last_ch[i]-rc_ctrl->rc.ch[i]))>500)
            rc_ctrl->rc.ch[i] = rc_ctrl->rc.last_ch[i];
        else
            rc_ctrl->rc.last_ch[i] = rc_ctrl->rc.ch[i];
    }
    remote_last_update_ms = HAL_GetTick();
}



