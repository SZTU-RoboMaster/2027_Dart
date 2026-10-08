#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    /* 允许串口接收请求触发直接存储器访问。 */
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    /* 空闲中断用于识别一帧遥控器数据结束。 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    /* 修改地址和长度前必须确认接收通道已经停止。 */
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    /* 配置双缓冲区：解析当前帧时，外设可继续写入另一个缓冲区。 */
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    /* 两个缓冲区共用同一个固定接收长度。 */
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    /* 开启双缓冲模式并恢复接收。 */
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart1);
}

void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    hdma_usart1_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    __HAL_UART_ENABLE(&huart1);
}
