#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"
#include "../Referee_system/Referee.h"

/*
 * 上位机/视觉 USB 协议定义
 * ------------------------
 * 所有线上结构都位于 pack(1) 区域，字段顺序和宽度属于协议 ABI，修改时必须同步上位机。
 * 解析与序列化分别位于 component/comunication/decode.c 和 packet.c。
 */

/* 固定帧标志及单帧最大长度。 */
#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

/* 帧内 16 位命令 ID。当前 Dart 固件只消费 CHASSIS_CTRL_CMD_ID。 */
typedef enum
{
    CHASSIS_ODOM_CMD_ID = 0x0101,
    CHASSIS_CTRL_CMD_ID = 0x0102,
    RGB_ID = 0x0103,
    RC_ID=0x0104,
    VISION_ID=0x0105
} data_cmd_id;


typedef enum
{
    /* 增量解析器按以下顺序逐字节推进，可跨 USB 数据包保存状态。 */
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;
/* USB 增量解包器的全部持久状态，不包含动态内存。 */
typedef struct
{
    /* p_header 为历史保留字段，当前解析器直接复制 protocol_packet。 */
    frame_header_struct_t *p_header;
    uint16_t       data_len;
    uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e  unpack_step;
    uint16_t       index;
} unpack_data_t;

typedef struct
{
    /* 额外的 CR/LF 风格帧结束标志，不参与 CRC16。 */
    uint8_t end1;
    uint8_t end2;
} msg_end_info ;

typedef struct
{
    /* 历史底盘里程信息；当前 Dart 默认构建不发送。 */
    float vx;
    float vy;
    float vw;
}  chassis_odom_info_t;
/* 发往视觉端的姿态与发射速度信息，保留作后续接口。 */
typedef struct
{
    uint16_t id;
    uint16_t mode; /* 自动模式历史值为 0x21。 */
    fp32 pitch;
    fp32 yaw;
    fp32 roll;
    fp32 quaternion[4];
    fp32 shoot_speed;
} vision_t;
/* 视觉端发回的控制信息；Dart 当前使用 yaw、target_lock。 */
typedef struct
{
    /* 遥控器透传结构，当前默认链路未使用。 */
    fp32 vx;
    fp32 vy;
    fp32 vw;
    fp32 yaw;
    fp32 pitch;
    int8_t target_lock;
    int8_t fire_command;
}  robot_ctrl_info_t;

typedef struct
{
    /* RGB 调试/指示信息，保留协议 ID 兼容性。 */
    int16_t ch[5];
    char s[2];
} rc_info_t;
typedef struct
{
    uint16_t R;
    uint16_t G;
    uint16_t B;
} RBG_info_t;
#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
