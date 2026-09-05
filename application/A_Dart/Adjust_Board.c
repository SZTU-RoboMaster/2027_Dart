//
// Created by 86134 on 2025/11/27.
//
#include "Adjust_Board.h"

#include "bsp_flash.h"
#include "cmsis_os.h"
#include "Send_to_Screen.h"
#include "dart.h"

#define BUFF_SIZE  128
static uint8_t rx_buff[BUFF_SIZE];

static float Str_to_float (char *msg,int *len);
fp32 outpost_base_value[8];

void adjust_task(void const*pvParameters)
{
    Flash_Read_Data(outpost_base_value,8);
    for (int i=0;i<4;i++)
    {
        dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][i] = outpost_base_value[i];
    }
    for (int i=0;i<4;i++)
    {
        dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][i] = outpost_base_value[i+4];
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart1,rx_buff,BUFF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
    while (1)
    {
        osDelay(1);
    }
}

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size)
// {
//     if (huart->Instance==USART1)
//     {
//         if (size <= BUFF_SIZE)
//         {
//             if (rx_buff[0]==0x55&&rx_buff[1]==0x01)
//             {
//                 int len=2;
//                 uint8_t cnt=0;
            //     while (cnt<4)
            //     {
            //         if (rx_buff[len]==cnt+1)
            //         {
            //             len++;
            //             float temp=Str_to_float(rx_buff,&len);
            //             dart_goal_set.trigger_distance_set[1][cnt] = temp;
            //             cnt++;
            //         }
            //     }
            // }else
            // if (rx_buff[0]==0x55&&rx_buff[1]==0x02)
            // {
            //     int len=2;
            //     uint8_t cnt=0;
            //     while (cnt<4)
//                 {
//                     if (rx_buff[len]==cnt+1)
//                     {
//                         len++;
//                         float temp=Str_to_float(rx_buff,&len);
//                         dart_goal_set.trigger_distance_set[2][cnt] = temp;
//                         cnt++;
//                     }
//                 }
//             }
//             for (int i=0;i<4;i++)
//             {
//                 outpost_base_value[i] = dart_goal_set.trigger_distance_set[1][i];
//             }
//             for (int i=4;i<8;i++)
//             {
//                 outpost_base_value[i] = dart_goal_set.trigger_distance_set[2][i-4];
//             }
//             Flash_Write_Data(outpost_base_value,8);
//         }
//         memset(rx_buff,0,BUFF_SIZE);
//         HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buff,BUFF_SIZE);
//         __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
//     }
// }

static float Str_to_float (char *msg,int *len)
{
    float temp=0;
    uint8_t flag=0;
    float point_va=1.0f;
    if (msg[*len]!=0xff)
    {
        while (1)
        {
            if (msg[*len]!=0xff)
            {
                if (msg[*len]!='.'&&!flag)
                {
                    temp*=10;
                    temp+=msg[*len]-'0';
                    (*len)++;
                }else
                    if (msg[*len]=='.')
                    {
                        flag=1;
                        (*len)++;
                    }else
                        if (msg[*len]!='.'&&flag)
                        {
                            point_va*=10;
                            temp+=(float)(msg[*len]-'0')/(float)point_va;
                            (*len)++;
                        }
            }else if (msg[*len]==0xff)
            {
                if (msg[(*len)+1]==0xff)
                {
                    (*len)++;
                    (*len)++;
                    return temp;
                }
            }
        }
    }
}