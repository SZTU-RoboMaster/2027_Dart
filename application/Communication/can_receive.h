#ifndef DART_CAN_RECEIVE_H
#define DART_CAN_RECEIVE_H

#include "../A_Dart/struct_typedef.h"
#include "PID.h"

/*
 * 飞镖系统 CAN 电机接口
 * ---------------------
 * 本文件只保留当前飞镖机构实际使用的报文标识、反馈结构和发送接口。业务状态机不应
 * 直接访问这些对象；它通过板级适配层读取统一的轴反馈并写入统一的执行器命令。
 */

/* 当前板卡使用的标准帧标识。发送标识与反馈标识可以位于不同 CAN 总线。 */
typedef enum {
    CAN_MOTOR_0x200_ID = 0x200, /* 大疆电机一至四号电流控制帧。 */
    CAN_MOTOR_0x1FF_ID = 0x1FF, /* 大疆电机五至八号电流控制帧。 */
    CAN_MOTOR_0x2FF_ID = 0x2FF, /* 预留的大疆电机控制帧。 */

    DM4310_TURN_MOTOR_ID = 0x01, /* 水平轴达妙电机控制标识。 */
    DM6006_TURN_MOTOR_ID = 0x09, /* 可选换弹转盘达妙电机控制标识。 */

    CAN_6020_YAW = 0x205,         /* 水平轴大疆电机反馈。 */
    CAN_3508_TRIGGER = 0x207,     /* 扳机轴反馈。 */
    CAN_3508_DRIVE_RIGHT = 0x206, /* 右推板轴反馈。 */
    CAN_3508_DRIVE_LEFT = 0x208,  /* 左推板轴反馈。 */
    CAN_3508_TURN_LEFT = 0x202,   /* 可选换弹机构左升降轴反馈。 */
    CAN_3508_TURN_RIGHT = 0x201,  /* 可选换弹机构右升降轴反馈。 */

    CAN_DM4310_TURN = 0x12, /* 水平轴达妙电机反馈标识。 */
    CAN_DM6006_TURN = 0x19  /* 可选换弹转盘达妙电机反馈标识。 */
} can_msg_id_e;

/* 发送接口使用的物理总线选择。 */
typedef enum {
    CAN_1,
    CAN_2,
} CAN_TYPE;

/*
 * 大疆电机反馈及多圈编码器累计值。
 * 中断回调写入这些字段，DartTask 在下一控制周期读取，因此字段类型保持为单次可访问的
 * 标量，不在结构中保存动态对象。
 */
typedef struct {
    uint16_t ecd;           /* 当前单圈编码器值，范围为零至八千一百九十一。 */
    int16_t speed_rpm;      /* 电机反馈转速。 */
    int16_t given_current;  /* 电机反馈电流。 */
    uint8_t temperate;      /* 电机反馈温度。 */
    int16_t last_ecd;       /* 上一帧单圈编码器值，用于判断跨圈。 */
    int32_t round_cnt;      /* 相对校准点累计的整圈数。 */
    int32_t total_ecd;      /* 扣除零点偏移后的连续累计编码器值。 */
    int32_t offset_ecd;     /* 机械回零时记录的累计编码器零点。 */
    int32_t total_dis;      /* 历史兼容距离字段，当前控制不直接使用。 */
    float torque_round_cnt; /* 电机输出轴换算前的累计圈数。 */
    float real_round_cnt;   /* 减速机构换算后的累计圈数。 */
    float real_angle_deg;   /* 单圈机械角度。 */
} motor_measure_t;

/* 三个 3508 轴共用的串级位置、速度控制上下文。 */
typedef struct {
    motor_measure_t *motor_measure; /* 指向 CAN 中断持续更新的反馈对象。 */
    fp32 speed;                     /* 当前速度反馈。 */
    fp32 rpm_set;                   /* 速度环目标。 */
    pid_t angle_p;                  /* 外层位置控制器。 */
    pid_t speed_p;                  /* 内层速度控制器。 */
    int16_t give_current;           /* 限幅后的最终电流命令。 */
} motor_3508_t;

/* 水平轴 6020 电机的串级控制上下文。 */
typedef struct {
    motor_measure_t *motor_measure; /* 指向水平轴反馈对象。 */
    pid_t angle_p;                  /* 常规位置控制器。 */
    pid_t speed_p;                  /* 常规速度控制器。 */
    pid_t angle_p_auto;             /* 视觉瞄准预留位置控制器。 */
    pid_t speed_p_auto;             /* 视觉瞄准预留速度控制器。 */
    fp32 max_relative_angle;        /* 软件允许的最大相对角度。 */
    fp32 min_relative_angle;        /* 软件允许的最小相对角度。 */
    fp32 relative_angle_get;        /* 当前相对角度。 */
    fp32 relative_angle_set;        /* 相对角度目标。 */
    fp32 absolute_angle_get;        /* 当前绝对角度。 */
    fp32 absolute_angle_set;        /* 绝对角度目标。 */
    fp32 gyro_set;                  /* 内层速度目标。 */
    int16_t give_current;           /* 限幅后的最终电流命令。 */
} motor_6020_t;

/* 当前飞镖机构实际使用的五个 3508 反馈和一个 6020 反馈。 */
extern motor_measure_t motor_3508[5];
extern motor_measure_t motor_6020[2];

/* 每个反馈对象最后一次收到合法 CAN 帧的系统时刻，用于二百毫秒离线判断。 */
extern volatile uint32_t motor_3508_last_update[5];
extern volatile uint32_t motor_6020_last_update[2];
extern volatile uint32_t dm6006_last_update;

/**
 * @brief 按大疆四电机控制帧格式发送四路有符号电流。
 *
 * @param can_type 目标物理总线。
 * @param command_id 控制帧标准标识。
 * @param motor1 第一电机电流。
 * @param motor2 第二电机电流。
 * @param motor3 第三电机电流。
 * @param motor4 第四电机电流。
 */
void CAN_cmd_motor(CAN_TYPE can_type,
                   can_msg_id_e command_id,
                   int16_t motor1,
                   int16_t motor2,
                   int16_t motor3,
                   int16_t motor4);

/**
 * @brief 通过指定控制器发送一帧标准 CAN 数据。
 * @param hcan 目标控制器句柄。
 * @param id 标准帧标识。
 * @param data 数据首地址。
 * @param length 有效字节数，标准数据帧不得超过八字节。
 * @return 零表示底层接受发送请求，非零表示当前发送失败。
 */
uint8_t CANx_SendStdData(CAN_HandleTypeDef *hcan,
                         uint16_t id,
                         uint8_t *data,
                         uint16_t length);

#endif /* DART_CAN_RECEIVE_H */
