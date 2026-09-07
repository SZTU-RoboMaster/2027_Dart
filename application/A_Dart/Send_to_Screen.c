//
// Created by 86134 on 2025/11/28.
//
#include "Send_to_Screen.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Adjust_Board.h"
#include "usart.h"

#define SCREEN_TX_BUFFER_SIZE 512U
#define SCREEN_TX_QUEUE_DEPTH 32U

/* Each queue slot remains valid until its DMA transfer has completed. */
static uint8_t screen_tx_queue[SCREEN_TX_QUEUE_DEPTH][SCREEN_TX_BUFFER_SIZE];
static uint16_t screen_tx_queue_len[SCREEN_TX_QUEUE_DEPTH];
static volatile uint8_t screen_tx_head;
static volatile uint8_t screen_tx_tail;
static volatile uint8_t screen_tx_count;
static volatile uint8_t screen_tx_busy;

static void Screen_Queue(const uint8_t *data, uint16_t len);
static uint8_t Screen_Append_Data_Command(uint8_t *buffer, uint16_t *len,
                                          const char *name, int index,
                                          float value);

static void Screen_EnterCritical(uint32_t *primask)
{
    *primask = __get_PRIMASK();
    __disable_irq();
}

static void Screen_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t Screen_NextIndex(uint8_t index)
{
    index++;
    if (index >= SCREEN_TX_QUEUE_DEPTH)
    {
        index = 0U;
    }
    return index;
}

void Send_to_Screen(const char *msg)
{
    uint8_t command[SCREEN_TX_BUFFER_SIZE];
    size_t msg_len;

    if (msg == NULL)
    {
        return;
    }

    msg_len = strlen(msg);
    if (msg_len == 0U || msg_len > (sizeof(command) - 3U))
    {
        return;
    }

    memcpy(command, msg, msg_len);
    command[msg_len] = 0xFFU;
    command[msg_len + 1U] = 0xFFU;
    command[msg_len + 2U] = 0xFFU;

    Screen_Queue(command, (uint16_t)(msg_len + 3U));
}

void Send_to_Screen_Data(const float *outpost, const float *base)
{
    uint8_t command[SCREEN_TX_BUFFER_SIZE];
    uint16_t len = 0U;

    if (outpost == NULL || base == NULL)
    {
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        if (!Screen_Append_Data_Command(command, &len, "outpost", i + 3,
                                        outpost[i]) ||
            !Screen_Append_Data_Command(command, &len, "base", i + 3,
                                        base[i]))
        {
            return;
        }
    }

    Screen_Queue(command, len);
}

static uint8_t Screen_Append_Data_Command(uint8_t *buffer, uint16_t *len,
                                          const char *name, int index,
                                          float value)
{
    int remaining = (int)SCREEN_TX_BUFFER_SIZE - *len;
    int written;

    written = snprintf((char *)&buffer[*len], (size_t)remaining,
                       "%s.t%d.txt=\"%.4f\"\xff\xff\xff",
                       name, index, value);
    if (written < 0 || written >= remaining)
    {
        return 0U;
    }

    *len += (uint16_t)written;
    return 1U;
}

static void Screen_Queue(const uint8_t *data, uint16_t len)
{
    uint32_t primask;
    uint8_t start_now = 0U;
    uint8_t dma_index = 0U;
    uint16_t dma_len = 0U;
    HAL_StatusTypeDef status;

    if (data == NULL || len == 0U || len > SCREEN_TX_BUFFER_SIZE)
    {
        return;
    }

    Screen_EnterCritical(&primask);
    if (screen_tx_count >= SCREEN_TX_QUEUE_DEPTH)
    {
        /* The UART has finite throughput; preserve queued data on overflow. */
        Screen_ExitCritical(primask);
        return;
    }

    memcpy(screen_tx_queue[screen_tx_tail], data, len);
    screen_tx_queue_len[screen_tx_tail] = len;
    screen_tx_tail = Screen_NextIndex(screen_tx_tail);
    screen_tx_count++;

    if (!screen_tx_busy)
    {
        screen_tx_busy = 1U;
        dma_index = screen_tx_head;
        dma_len = screen_tx_queue_len[dma_index];
        start_now = 1U;
    }
    Screen_ExitCritical(primask);

    if (start_now)
    {
        status = HAL_UART_Transmit_DMA(&huart7, screen_tx_queue[dma_index],
                                       dma_len);
        if (status != HAL_OK)
        {
            Screen_EnterCritical(&primask);
            screen_tx_head = screen_tx_tail;
            screen_tx_count = 0U;
            screen_tx_busy = 0U;
            Screen_ExitCritical(primask);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;
    uint8_t start_next = 0U;
    uint8_t dma_index = 0U;
    uint16_t dma_len = 0U;

    if (huart->Instance != UART7)
    {
        return;
    }

    Screen_EnterCritical(&primask);
    if (screen_tx_count > 0U)
    {
        screen_tx_head = Screen_NextIndex(screen_tx_head);
        screen_tx_count--;
    }

    if (screen_tx_count > 0U)
    {
        dma_index = screen_tx_head;
        dma_len = screen_tx_queue_len[dma_index];
        start_next = 1U;
    }
    else
    {
        screen_tx_busy = 0U;
    }
    Screen_ExitCritical(primask);

    if (start_next && HAL_UART_Transmit_DMA(&huart7,
                                            screen_tx_queue[dma_index],
                                            dma_len) != HAL_OK)
    {
        Screen_EnterCritical(&primask);
        screen_tx_head = screen_tx_tail;
        screen_tx_count = 0U;
        screen_tx_busy = 0U;
        Screen_ExitCritical(primask);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;
    uint8_t tx_error;

    if (huart->Instance != UART7)
    {
        return;
    }

    /* A UART7 RX error must not clear an unrelated TX DMA queue. */
    tx_error = (uint8_t)(screen_tx_busy &&
                         (huart->gState == HAL_UART_STATE_READY) &&
                         ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U));

    if (tx_error)
    {
        Screen_EnterCritical(&primask);
        screen_tx_head = screen_tx_tail;
        screen_tx_count = 0U;
        screen_tx_busy = 0U;
        Screen_ExitCritical(primask);
    }

    if (huart->RxState == HAL_UART_STATE_READY)
    {
        Adjust_Board_Restart_Receive();
    }
}
