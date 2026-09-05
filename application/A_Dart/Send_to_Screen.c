//
// Created by 86134 on 2025/11/28.
//
#include "Send_to_Screen.h"

#include <string.h>

#include "usart.h"
#include "stdio.h"

void Send_to_Screen(const char *msg)
{   char cmd[200]={0};
    int len=sprintf(cmd,"%s\xff\xff\xff",msg);
    HAL_UART_Transmit_DMA(&huart1,cmd,len);
}

void Send_to_Screen_Data(const float *outpost,const float *base)
{
    char cmd[200]={0};
    int t=3;
    int len=0;
    for (int i=0;i<4;i++)
    {
        len=sprintf(cmd,"outpost.t%d.txt=\"%.4f\"\xff\xff\xff",t,outpost[i]);
        HAL_UART_Transmit(&huart1,cmd,len,100);
        memset(cmd,0,200);
        len=sprintf(cmd,"base.t%d.txt=\"%.4f\"\xff\xff\xff",t,base[i]);
        HAL_UART_Transmit(&huart1,cmd,len,100);
        memset(cmd,0,200);
        t++;
    }
}