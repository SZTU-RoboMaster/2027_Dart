//
// Created by xhuanc on 2021/9/27.
//

#include "can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "../Chassis/Chassis.h"
#include "math.h"
#include "../Referee_system/Detection.h"
#include "../Gimbal/launcher.h"
#include "../Chassis/Cap.h"
#include "bsp_cap.h"
#include "../A_Dart/dart.h"
#include "DM_MOTOR.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/******************** define *******************/

//电子数据解算,do while作为保护性代码，防止在展开时被错误编译
#define get_motor_measure(ptr, data)                                    \
    do{                                                                 \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    } while(0)

//电机总编码值的计算,do while作为保护性代码，防止在展开时被错误编译
#define get_motor_round_cnt(ptr)  \
    do{                            \
             if(ptr.ecd-ptr.last_ecd> 4192){ \
                ptr.round_cnt--;                    \
             }                   \
             else if(ptr.ecd-ptr.last_ecd< -4192)    \
             {                   \
                ptr.round_cnt++;            \
             }                   \
             ptr.total_ecd= ptr.round_cnt*8192+ptr.ecd-ptr.offset_ecd;\
                                 \
    }while(0)
//电机真实距离计算
#define get_motor_real_distance(ptr)\
{\
    get_motor_round_cnt(ptr);\
    ptr.torque_round_cnt=ptr.total_ecd/8192.f; \
    ptr.real_round_cnt=ptr.torque_round_cnt/19.f; \
    ptr.real_angle_deg=fmodf(ptr.real_round_cnt,1.0f);             \
    if(ptr.real_angle_deg<0.0f)ptr.real_angle_deg+=1.0f;                \
    ptr.real_angle_deg*=360.0f;     \
    ptr.total_dis=ptr.real_round_cnt*wheel_circumference;      \
}\

//3508减速比
#define motor_3508_reduction_ratio (3591.0f/187.0f)
//轮子周长
#define wheel_circumference (70*2*3.14f)
/******************** variable *******************/

motor_measure_t motor_3508_measure[4];//0-3 分别对应  RF,LF,LB,RB

motor_measure_t motor_yaw_measure;
motor_measure_t motor_pitch_measure;
motor_measure_t motor_turn_measure;

motor_measure_t motor_3508[5];
motor_measure_t motor_2006[3];
motor_measure_t motor_6020[2];


motor_measure_t motor_yaw_measure;
motor_measure_t motor_pitch_measure;
motor_measure_t motor_shoot_measure[4];//0:TRIGGER,建为数组方便以后添加
motor_measure_t motor_2006_measure[1];//TRIGGER
extern cap2_info_t cap2;

static CAN_TxHeaderTypeDef tx_message;
static uint8_t can_send_data[8];
int32_t cap_percentage;

int cap_can_cnt = 0;

void cap2_info_decode(cap2_info_t *cap,uint8_t *rx_data){
    cap->mode=rx_data[0];
//    cap->rec_cap_cmd=rx_data[1];
//    cap->cap_voltage=rx_data[2];
    cap->cap_voltage=(uint16_t)(rx_data[2]<<8|rx_data[3]);
//    cap->chassis_current=(uint16_t)(rx_data[4]<<8|rx_data[5])/1000;
    cap_percentage=cap->cap_voltage/22.f;

    cap_can_cnt++;
}

extern void dm8009_can_msg_unpack(uint32_t id, uint8_t data[]);


//车轮电机的发送函数
void CAN_cmd_motor(CAN_TYPE can_type, can_msg_id_e CMD_ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    uint32_t send_mail_box;
    tx_message.StdId = CMD_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    if (can_type == CAN_1) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    } else if (can_type == CAN_2) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;

    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1) {
        switch (rx_header.StdId) {
            case CAN_DM4310_TURN:
                DM_Motor_Decode(&YAW_Motor, CAN_1, rx_header.StdId,rx_data);
                break;

            case CAN_DM6006_TURN:
                DM_Motor_Decode(&turndish_dart.dm_turn_motor, CAN_1, rx_header.StdId, rx_data);
                break;

            default: {
                break;
            }
        }
    }
    if (hcan == &hcan2) {
        //dm8009_can_msg_unpack(rx_header.StdId,rx_data);
        switch (rx_header.StdId){
            case CAN_3508_DRIVE_RIGHT: get_motor_measure(&motor_3508[0], rx_data);
                get_motor_round_cnt(motor_3508[0]);
                break;

            case CAN_3508_DRIVE_LEFT: get_motor_measure(&motor_3508[1], rx_data);
                get_motor_round_cnt(motor_3508[1]);
                break;

            case CAN_3508_TRIGGER: get_motor_measure(&motor_3508[2], rx_data);
                get_motor_round_cnt(motor_3508[2]);
                break;

            case CAN_6020_YAW: get_motor_measure(&motor_6020[0], rx_data);
                get_motor_round_cnt(motor_6020[0]);
                break;
            case CAN_3508_TURN_LEFT: get_motor_measure(&motor_3508[3], rx_data);
                get_motor_round_cnt(motor_3508[3]);
                break;

            case CAN_3508_TURN_RIGHT: get_motor_measure(&motor_3508[4], rx_data);
                get_motor_round_cnt(motor_3508[4]);
                break;

            default: {
                break;
            }
        }
    }

}


fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd) {
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE) {
        relative_ecd -= ECD_RANGE;
    } else if (relative_ecd < -HALF_ECD_RANGE) {
        relative_ecd += ECD_RANGE;
    }

    return ((fp32)relative_ecd * MOTOR_ECD_TO_RAD);
}

uint8_t CANx_SendStdData(CAN_HandleTypeDef *hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
    static CAN_TxHeaderTypeDef   Tx_Header;

    Tx_Header.StdId=ID;
    Tx_Header.ExtId=0;
    Tx_Header.IDE=0;
    Tx_Header.RTR=0;
    Tx_Header.DLC=Len;

    /*找到空的发送邮箱，把数据发送出去*/
    if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
    {
        if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}


//计算距离零点的度数  -180-180
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd) {
    int32_t tmp = 0;
    if (offset_ecd >= 4096) {
        if (ecd > offset_ecd - 4096) {
            tmp = ecd - offset_ecd;
        } else {
            tmp = ecd + 8192 - offset_ecd;
        }
    } else {
        if (ecd > offset_ecd + 4096) {
            tmp = ecd - 8192 - offset_ecd;
        } else {
            tmp = ecd - offset_ecd;
        }
    }
    return (fp32) tmp / 8192.f * 360;
}

void CAN_cmd_cap2(cap2_info_t*cap) {
    uint32_t send_mail_box;
    tx_message.StdId = 0x002;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x06;
    for (uint8_t i = 0; i <=4 ; ++i) {
        can_send_data[i]=cap->send_data[i];
    }
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, cap->send_data, &send_mail_box);
}

uint32_t get_can1_free_mailbox() {
    if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET) {
        return CAN_TX_MAILBOX0;
    } else if ((hcan1.Instance->TSR & CAN_TSR_TME1)
               != RESET) { return CAN_TX_MAILBOX1; }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET) { return CAN_TX_MAILBOX2; }
    else { return 0; }
}

uint32_t get_can2_free_mailbox() {
    if ((hcan2.Instance->TSR & CAN_TSR_TME0) != RESET) {
        return CAN_TX_MAILBOX0;
    } else if ((hcan2.Instance->TSR & CAN_TSR_TME1)
               != RESET) { return CAN_TX_MAILBOX1; }
    else if ((hcan2.Instance->TSR & CAN_TSR_TME2) != RESET) { return CAN_TX_MAILBOX2; }
    else { return 0; }
}
