#ifndef OMNI_INFANTRY_DM_MOTOR_H
#define OMNI_INFANTRY_DM_MOTOR_H

#include "../application/A_Dart/struct_typedef.h"
#include "bsp_can.h"
#include "../application/Communication/can_receive.h"
#include "user_lib.h"

/* 达妙电机一帧反馈的原始定点值、解码物理量和通信暂存区。 */
typedef struct {
    int p_int, v_int, t_int;         /* 协议中的位置、速度和转矩定点整数。 */
    fp32 position, velocity, torque; /* 解码后的浮点物理量。 */
    uint8_t Tx_Data[8];              /* 八字节发送暂存区。 */
    uint8_t RxData[8];               /* 八字节接收暂存区。 */
    CAN_RxHeaderTypeDef Rx_pHeader;  /* 最近一帧接收头，供底层诊断使用。 */

}DM_Motor_t;

extern DM_Motor_t YAW_Motor;
extern DM_Motor_t can_1;
extern fp32 DM_Velocity;
extern first_order_filter_type_t DM_Velocity_Filter;

extern uint8_t DM_Enable_CMD[8];
extern uint8_t DM_Disable_CMD[8];
extern uint8_t DM_Save_ZeroPoint_CMD[8];
extern uint8_t DM_Clear_Error_CMD[8];

extern void DM_Send_CMD(CAN_TYPE can_type, can_msg_id_e motor_id, uint8_t *cmd);
extern void DM_MIT_Ctrl_Motor(CAN_TYPE can_type, uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _troq);
extern void DM_Motor_Decode(DM_Motor_t *motor, CAN_TYPE can_type, uint32_t can_id, uint8_t *data);

#endif /* OMNI_INFANTRY_DM_MOTOR_H */
