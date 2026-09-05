//
// Created by liang on 2025-04-08.
//

#include "DM_MOTOR.h"
#include "user_lib.h"
#include "../application/Referee_system/Detection.h"
#include "../application/Communication/can_receive.h"
#include "bsp_can.h"
#include "can.h"
#include "../application/A_Dart/dart.h"

// MIT控制帧相关常量定义
#define P_MIN 0
#define P_MAX 6.28
#define V_MIN -45
#define V_MAX 45
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10
#define T_MAX 10

// 内部函数声明
static int fp32_to_uint(fp32 x, fp32 x_min, fp32 x_max, int bits);
static fp32 uint_to_fp32(int x_int, fp32 x_min, fp32 x_max, int bits);

// 全局变量声明
DM_Motor_t  YAW_Motor;
DM_Motor_t  can_1;
fp32 DM_Velocity;
first_order_filter_type_t DM_Velocity_Filter;

// DM电机使能
uint8_t DM_Enable_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
// DM电机失能
uint8_t DM_Disable_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
// DM电机保存零点
uint8_t DM_Save_ZeroPoint_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
// DM电机清错
uint8_t DM_Clear_Error_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

/**
 * @brief 发送DM电机控制命令
 * @param[in] can_type      CAN通道
 * @param[in] motor_id  DM电机ID
 * @param[in] cmd       指令
 */
void DM_Send_CMD(CAN_TYPE can_type, can_msg_id_e motor_id, uint8_t *cmd) {
    if(can_type == CAN_1) {
        CANx_SendStdData(&hcan1, motor_id, cmd, 8);
    } else if(can_type == CAN_2) {
        CANx_SendStdData(&hcan2, motor_id, cmd, 8);
    }
}

/**
 * @brief 将浮点数转换为无符号整数
 * @param[in] x         要转换的浮点数
 * @param[in] x_min     最小值
 * @param[in] x_max     最大值
 * @param[in] bits      无符号整数的位宽
 * @return              转换后的无符号整数
 */
int fp32_to_uint(fp32 x, fp32 x_min, fp32 x_max, int bits) {
    fp32 span = x_max - x_min;
    fp32 offset = x_min;
    return (int)((x - offset)*((fp32)((1 << bits) - 1)) / span);
}

/**
 * @brief 将无符号整数转换为浮点数
 * @param x       要转换的无符号整数
 * @param x_min   目标浮点数的最小值
 * @param x_max   目标浮点数的最大值
 * @param bits    无符号整数的位宽
 * @return        转换后的浮点数
 */

fp32 uint_to_fp32(int x_int, fp32 x_min, fp32 x_max, int bits) {
    fp32 span = x_max - x_min;
    fp32 offset = x_min;
    return ((fp32)x_int) * span / ((fp32)((1 << bits) - 1)) + offset;
}

/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
void DM_MIT_Ctrl_Motor(CAN_TYPE can_type, uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq) {
    static CAN_TxHeaderTypeDef Tx_Header;
    uint16_t pos_tmp, vel_tmp, torq_tmp, kp_tmp, kd_tmp;
    pos_tmp = fp32_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = fp32_to_uint(_vel, V_MIN, V_MAX, 12);
    torq_tmp = fp32_to_uint(_torq, T_MIN, T_MAX, 12);
    kp_tmp = fp32_to_uint(_KP, KP_MIN, KP_MAX, 16);
    kd_tmp = fp32_to_uint(_KD, KD_MIN, KD_MAX, 16);

    Tx_Header.StdId = id;
    Tx_Header.IDE = CAN_ID_STD;
    Tx_Header.RTR = CAN_RTR_DATA;
    Tx_Header.DLC = 0x08;

    can_1.Tx_Data[0] = (pos_tmp >> 8);
    can_1.Tx_Data[1] = pos_tmp;
    can_1.Tx_Data[2] = (vel_tmp >> 4);
    can_1.Tx_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    can_1.Tx_Data[4] = kp_tmp;
    can_1.Tx_Data[5] = (kd_tmp >> 4);
    can_1.Tx_Data[6] = ((kd_tmp & 0xF) << 4) | (torq_tmp >> 8);
    can_1.Tx_Data[7] = torq_tmp;

    // 寻找空邮箱并发送数据，基本上就是把现有的邮箱都发一下看看成不成功
    if(can_type == CAN_1){
        if(HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, can_1.Tx_Data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
            if(HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, can_1.Tx_Data, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
                HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, can_1.Tx_Data, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
    }else if(can_type == CAN_2){
        if(HAL_CAN_AddTxMessage(&hcan2, &Tx_Header, can_1.Tx_Data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
            if(HAL_CAN_AddTxMessage(&hcan2, &Tx_Header, can_1.Tx_Data, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
                HAL_CAN_AddTxMessage(&hcan2, &Tx_Header, can_1.Tx_Data, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
    }

}

int status;
/**
 * @brief 解码DM电机数据帧
 * @param[in] motor     DM电机数据结构体指针
 * @param[in] hcan      CAN通道
 * @param[in] can_id    CAN数据帧ID
 * @param[in] data      CAN数据帧数据
 */
void DM_Motor_Decode(DM_Motor_t *motor, CAN_TYPE hcan, uint32_t can_id, uint8_t *data) {
     if(hcan == CAN_1) {
         if(CAN_DM4310_TURN == can_id) {
            status = data[0] & 0xF0;
            if(status == 0) {
                DM_Send_CMD(CAN_1, DM4310_TURN_MOTOR_ID, DM_Enable_CMD);
            }
            motor->p_int = (data[1] << 8) | data[2];
            motor->v_int = (data[3] << 4) | (data[4] >> 4);
            motor->t_int = ((data[4] & 0xF) << 8) | data[5];

            motor->position = uint_to_fp32(motor->p_int, P_MIN, P_MAX, 16);
            motor->velocity = uint_to_fp32(motor->v_int, V_MIN, V_MAX, 12);
            motor->torque = uint_to_fp32(motor->t_int, T_MIN, T_MAX, 12);

            //first_order_filter_cali(&DM_Velocity_Filter, YAW_Motor.velocity);
            DM_Velocity = DM_Velocity_Filter.out;
            //detect_handle(DETECT_GIMBAL_DM_YAW);
        }

         if(CAN_DM6006_TURN == can_id) {
             status = data[0] & 0xF0;
             if(status == 0) {
                 DM_Send_CMD(CAN_1, DM6006_TURN_MOTOR_ID, DM_Enable_CMD);
             }
             motor->p_int = (data[1] << 8) | data[2];
             motor->v_int = (data[3] << 4) | (data[4] >> 4);
             motor->t_int = ((data[4] & 0xF) << 8) | data[5];

             motor->position = uint_to_fp32(motor->p_int, P_MIN, P_MAX, 16);
             motor->velocity = uint_to_fp32(motor->v_int, V_MIN, V_MAX, 12);
             motor->torque = uint_to_fp32(motor->t_int, T_MIN, T_MAX, 12);

             //first_order_filter_cali(&DM_Velocity_Filter, YAW_Motor.velocity);
             DM_Velocity = DM_Velocity_Filter.out;
             //detect_handle(DETECT_GIMBAL_DM_YAW);
         }
    }
}
