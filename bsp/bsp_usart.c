#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void usart1_init(uint8_t*rx_buf,uint16_t dma_buf_num)
{

    /* 允许串口一的收发请求触发直接存储器访问。 */
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); /* 使用空闲中断识别一帧结束。 */

    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while (hdma_usart1_rx.Instance->CR&DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_HISR_TCIF5);

    hdma_usart1_rx.Instance->PAR=(uint32_t)&(USART1->DR);
    hdma_usart1_rx.Instance->M0AR=(uint32_t)(rx_buf);
    hdma_usart1_rx.Instance->NDTR=dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);


    /* 初始化发送通道为空闲状态。 */
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{

    /* 更新发送地址和长度前先停止旧传输。 */
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



void usart6_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{

    /* 允许裁判串口的收发请求触发直接存储器访问。 */
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    /* 空闲中断用于冻结本批数据的实际接收长度。 */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    /* 修改裁判接收缓冲区前先停止接收通道。 */
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    /* 裁判数据使用单接收缓冲区，由上层在空闲中断中复制到静态队列。 */
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    /* 设置一次最多接收的字节数。 */
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    /* 恢复接收通道。 */
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

    /* 初始化裁判发送通道为空闲状态。 */
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    /* 每次发送前停止通道，以便安全更新数据地址和计数器。 */
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }
    /* 清除上一次发送完成标志。 */
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);
    /* 写入本次发送数据地址和长度，然后启动传输。 */
    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);
    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}


void uart6_tx_one_byte(uint8_t* data){
    while (!huart6.Instance->SR&UART_FLAG_TC);//判断串口6的TC标志位是否置1 置1则退出循环
    HAL_UART_Transmit(&huart6,data,1,1);
}

void uart6_tx_mul_byte_dma(uint8_t*data,uint16_t len){

    HAL_UART_Transmit_DMA(&huart6,data,len);
}
