//
// Created by Shockley on 2022/12/5.
//
#include "decode.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "../../application/A_Dart/protocol_shaob.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "decode.h"
#include "../../application/Referee_system/Detection.h"
#include "../../application/A_Dart/dart.h"
#include "math.h"

void usb_fifo_init();
void decode_task(void const * arg);
extern robot_ctrl_info_t robot_ctrl;

//usb fifo 控制结构体
fifo_s_t usb_fifo;

//usb fifo环形缓存区
uint8_t usb_fifo_buf[512];
unpack_data_t decode_unpack;

//协议解包控制结构体
unpack_data_t decode_unpack_obj;
robot_ctrl_info_t robot_ctrl;

//反序列化函数
void decode_unpack_fifo_data(void);
uint16_t decode_data_solve(uint8_t *frame);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//协议序列化函数
void encode_send_data(uint16_t cmd_id, void* buf, uint16_t len);

frame_header_struct_t decode_receive_header;

void decode_task(void const * argument);

//协议解析函数，freertos调用
void decode_task(void const *arg)
{
    fifo_s_init(&usb_fifo, usb_fifo_buf, 512);
    while(1)
    {
        decode_unpack_fifo_data();
        osDelay(1);
    }
}
//USB FIFO 初始化
void usb_fifo_init(void)
{
    fifo_s_init(&usb_fifo, usb_fifo_buf, 512);
}
//usb 接受中断
void usb_receiver(uint8_t *buf, uint32_t len)
{
    fifo_s_puts(&usb_fifo, (char*)buf, len);
}
uint8_t byte = 0;
//反序列化
void decode_unpack_fifo_data()
{
    //uint8_t byte=0;                     // 单字节
    uint8_t SOF = HEADER_SOF;           // 帧头
    unpack_data_t *p = &decode_unpack;  // 解包指针
    while (fifo_s_used(&usb_fifo)){
        // 获取单字节数据
        byte = fifo_s_get(&usb_fifo);
        switch(p->unpack_step){
            case STEP_HEADER_SOF:{
                // 查找帧头
                if(byte == SOF){
                    p->unpack_step = STEP_LENGTH_LOW;
                    p->protocol_packet[p->index++] = byte;
                } else{
                    p->index = 0;
                }
            }break;
            case STEP_LENGTH_LOW:{
                /* 接收数据低字节 */
                p->data_len = byte;
                p->protocol_packet[p->index++] = byte;
                p->unpack_step = STEP_LENGTH_HIGH;
            }break;
            case STEP_LENGTH_HIGH:{
                /* 接收数据高字节 */
                p->data_len |= (byte << 8);             // 拼接数据长度
                p->protocol_packet[p->index++] = byte;
                // 数据长度小于 (128-帧头数据长度)
                if(p->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)){
                    /* 获取包序 */
                    p->unpack_step = STEP_FRAME_SEQ;
                } else{
                    /* 重新寻找帧头 */
                    p->unpack_step = STEP_HEADER_SOF;
                    p->index = 0;
                }
            }break;
            case STEP_FRAME_SEQ:{
                /* 记录协议序列号,进行CRC8验证 */
                p->protocol_packet[p->index++] = byte;
                p->unpack_step = STEP_HEADER_CRC8;
            }break;
            case STEP_HEADER_CRC8:{
                p->protocol_packet[p->index++] = byte;
                // 目前长度等于帧头长度
                if (p->index == REF_PROTOCOL_HEADER_SIZE){
                    /* 进行CRC8校验 */
                    if (verify_CRC8_check_sum(p->protocol_packet, REF_PROTOCOL_HEADER_SIZE)){
                        p->unpack_step = STEP_DATA_CRC16;
                    } else{
                        p->unpack_step = STEP_HEADER_SOF;
                        p->index = 0;
                    }
                }
            }break;
            case STEP_DATA_CRC16:{
                /* 循环获取数据 */
                if (p->index < (REF_HEADER_CRC_CMDID_LEN + p->data_len))
                    p->protocol_packet[p->index++] = byte;

                /* 进行CRC16校验 */
                if (p->index >= (REF_HEADER_CRC_CMDID_LEN + p->data_len)){
                    /* 重新设置为初始状态 */
                    p->unpack_step = STEP_HEADER_SOF;
                    p->index = 0;

                    /* 进行CRC16校验 */
                    if (verify_CRC16_check_sum(p->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p->data_len)){
                        // 开始解析数据
                        decode_data_solve(p->protocol_packet);

                    }
                }
            }break;
            default:
            {
                /* 解包失败,重新初始化 */
                p->unpack_step = STEP_HEADER_SOF;
                p->index = 0;
                break;
            }
        }
    }
}
//把frame的信息转到对应的结构体中
extern uint8_t dart_goal;
union
{
    uint8_t data[4];
    float yaw;
}yaw2;

uint16_t decode_data_solve(uint8_t *frame){
    uint8_t index = 0;
    uint16_t cmd_id = 0;
    /* 提取帧头数据 */
    memcpy(&decode_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);
    /* 提取CMD_ID */
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id){
        //接受控制码对应信息包
        case CHASSIS_CTRL_CMD_ID: {
            memcpy(&robot_ctrl, frame + index, sizeof(robot_ctrl_info_t));
                if (!robot_ctrl.target_lock)
                {
                    robot_ctrl.yaw = 0;
                }
            if(robot_ctrl.target_lock) {
                {
#if 1 //Q系列
                    robot_ctrl.yaw -=0.00;//越+越往右，越-越往左
                    if (dart_goal_set.launcherable_num == 0) {
                        robot_ctrl.yaw += 0.5;
                    }else if(dart_goal_set.launcherable_num == 1) {
                         robot_ctrl.yaw += 0.5;
                    }else if (dart_goal_set.launcherable_num == 2) {
                        robot_ctrl.yaw += 0.5;
                    }else if (dart_goal_set.launcherable_num == 3) {
                        robot_ctrl.yaw += 0.5;
                    }
#else //AI
                    if (dart_goal_set.launcherable_num == 0) {
                        robot_ctrl.yaw += 0.18;
                    }else if(dart_goal_set.launcherable_num == 1) {
                        robot_ctrl.yaw += 0.19;
                    }else if (dart_goal_set.launcherable_num == 2) {
                        robot_ctrl.yaw += 0.41;
                    }else if (dart_goal_set.launcherable_num == 3) {
                        robot_ctrl.yaw += 0.25;
                    }
#endif
                }
            }
            //yaw1=robot_ctrl.yaw;
            // 离线检查部分没有
            break;
        }
        default:{
            break;
        }
    }
    index += decode_receive_header.data_length + 2;
    return index;
}