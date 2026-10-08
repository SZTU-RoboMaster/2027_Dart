#include "DM_MOTOR.h"
#include "user_lib.h"
#include "../application/Communication/can_receive.h"
#include "bsp_can.h"
#include "can.h"
#include "../application/A_Dart/dart.h"

/* 达妙电机控制协议允许的物理量范围，编码和解码必须使用完全相同的上下限。 */
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

/* 协议定点数与浮点物理量之间的转换函数。 */
static int fp32_to_uint(fp32 x, fp32 x_min, fp32 x_max, int bits);
static fp32 uint_to_fp32(int x_int, fp32 x_min, fp32 x_max, int bits);

/* 水平轴反馈对象以及发送帧共用的暂存对象。 */
DM_Motor_t  YAW_Motor;
DM_Motor_t  can_1;
fp32 DM_Velocity;
first_order_filter_type_t DM_Velocity_Filter;

/* 达妙电机厂家协议规定的使能、失能、保存零点和清错命令。 */
uint8_t DM_Enable_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
uint8_t DM_Disable_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
uint8_t DM_Save_ZeroPoint_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
uint8_t DM_Clear_Error_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

/**
 * @说明 向指定 CAN 总线发送一组八字节达妙特殊命令。
 * @参数 can_type 目标物理总线。
 * @参数 motor_id 目标电机控制标识。
 * @参数 cmd 厂家协议规定的八字节命令。
 */
void DM_Send_CMD(CAN_TYPE can_type, can_msg_id_e motor_id, uint8_t *cmd) {
    if(can_type == CAN_1) {
        CANx_SendStdData(&hcan1, motor_id, cmd, 8);
    } else if(can_type == CAN_2) {
        CANx_SendStdData(&hcan2, motor_id, cmd, 8);
    }
}

/**
 * @说明 把给定范围内的浮点物理量线性映射为无符号定点整数。
 * @参数 x 待转换物理量。
 * @参数 x_min 物理量下限。
 * @参数 x_max 物理量上限。
 * @参数 bits 协议字段位数。
 * @返回值 编码后的无符号整数。
 */
int fp32_to_uint(fp32 x, fp32 x_min, fp32 x_max, int bits) {
    fp32 span = x_max - x_min;
    fp32 offset = x_min;
    return (int)((x - offset)*((fp32)((1 << bits) - 1)) / span);
}

/**
 * @说明 把协议无符号定点整数还原为浮点物理量。
 * @参数 x_int 待转换整数。
 * @参数 x_min 目标物理量下限。
 * @参数 x_max 目标物理量上限。
 * @参数 bits 协议字段位数。
 * @返回值 解码后的浮点物理量。
 */

fp32 uint_to_fp32(int x_int, fp32 x_min, fp32 x_max, int bits) {
    fp32 span = x_max - x_min;
    fp32 offset = x_min;
    return ((fp32)x_int) * span / ((fp32)((1 << bits) - 1)) + offset;
}

/**
 * @说明 按达妙 MIT 模式打包位置、速度、刚度、阻尼和转矩控制帧。
 * @参数 can_type 目标物理总线。
 * @参数 id 目标电机控制标识。
 * @参数 _pos 位置目标。
 * @参数 _vel 速度目标。
 * @参数 _KP 位置比例系数。
 * @参数 _KD 位置微分系数。
 * @参数 _torq 转矩目标。
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

    /* 依次尝试三个硬件发送邮箱，减少高频控制时因单个邮箱占用造成的丢帧。 */
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
 * @说明 解码一帧达妙电机反馈，并在电机未使能时请求使能。
 * @参数 motor 接收解码结果的电机对象。
 * @参数 hcan 反馈来自哪条物理总线。
 * @参数 can_id 反馈帧标识。
 * @参数 data 八字节反馈数据。
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

            DM_Velocity = DM_Velocity_Filter.out;
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

             DM_Velocity = DM_Velocity_Filter.out;
         }
    }
}
