//
// Created by liang on 2025-04-08.
//

#ifndef OMNI_INFANTRY_DM_MOTOR_H
#define OMNI_INFANTRY_DM_MOTOR_H

#include "../application/A_Dart/struct_typedef.h"
#include "bsp_can.h"
#include "../application/Communication/can_receive.h"
#include "user_lib.h"

typedef struct {

    int p_int, v_int, t_int;                // 整型的电机位置、速度、扭矩数据
    fp32 position, velocity, torque;        // 浮点型的电机位置、速度、扭矩数据
    uint8_t  Tx_Data[8];					// 数据发送存储
    uint8_t  RxData[8];						// 数据接收存储
    CAN_RxHeaderTypeDef Rx_pHeader;         // 接收帧头定义

}DM_Motor_t;

extern DM_Motor_t YAW_Motor;
extern DM_Motor_t can_1;
extern fp32 DM_Velocity;
extern first_order_filter_type_t DM_Velocity_Filter;

extern uint8_t DM_Enable_CMD[8];
// DM电机失能
extern uint8_t DM_Disable_CMD[8];
// DM电机保存零点
extern uint8_t DM_Save_ZeroPoint_CMD[8];
// DM电机清错
extern uint8_t DM_Clear_Error_CMD[8];

extern void DM_Send_CMD(CAN_TYPE can_type, can_msg_id_e motor_id, uint8_t *cmd);
extern void DM_MIT_Ctrl_Motor(CAN_TYPE can_type, uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _troq);
extern void DM_Motor_Decode(DM_Motor_t *motor, CAN_TYPE can_type, uint32_t can_id, uint8_t *data);

#endif //OMNI_INFANTRY_DM_MOTOR_H