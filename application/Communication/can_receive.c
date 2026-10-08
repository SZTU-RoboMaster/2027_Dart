#include "can_receive.h"

#include "../A_Dart/dart.h"
#include "dart_build_config.h"
#include "DM_MOTOR.h"
#include "main.h"

/*
 * 当前板卡的 CAN 收发实现
 * -----------------------
 * CAN1 接收达妙水平轴以及可选的换弹转盘，CAN2 接收扳机、左右推板、左右升降和水平轴
 * 的大疆电机反馈。中断只完成定长报文解码和更新时间记录，不运行 PID、不推进状态机。
 * 这样可以控制中断执行时间，并让所有业务判断统一发生在 DartTask 的反馈快照中。
 */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*
 * 解码大疆电机固定八字节反馈。
 * 使用单语句宏是为了避免函数调用开销；保护性循环保证它可以安全出现在条件分支中。
 */
#define GET_MOTOR_MEASURE(measure, bytes)                                      \
    do {                                                                        \
        (measure)->last_ecd = (measure)->ecd;                                   \
        (measure)->ecd = (uint16_t)((bytes)[0] << 8 | (bytes)[1]);              \
        (measure)->speed_rpm = (int16_t)((bytes)[2] << 8 | (bytes)[3]);         \
        (measure)->given_current = (int16_t)((bytes)[4] << 8 | (bytes)[5]);     \
        (measure)->temperate = (bytes)[6];                                      \
    } while (0)

/*
 * 把零至八千一百九十一的单圈值展开为连续多圈值。
 * 相邻反馈跨过半圈时才视为跨越编码器零点，避免普通高速变化被误判为整圈跳变。
 */
#define UPDATE_MOTOR_ROUND_COUNT(measure)                                      \
    do {                                                                        \
        if ((measure).ecd - (measure).last_ecd > 4192) {                        \
            (measure).round_cnt--;                                              \
        } else if ((measure).ecd - (measure).last_ecd < -4192) {               \
            (measure).round_cnt++;                                              \
        }                                                                       \
        (measure).total_ecd =                                                   \
            (measure).round_cnt * 8192 + (measure).ecd - (measure).offset_ecd;  \
    } while (0)

motor_measure_t motor_3508[5];
motor_measure_t motor_6020[2];
volatile uint32_t motor_3508_last_update[5];
volatile uint32_t motor_6020_last_update[2];
volatile uint32_t dm6006_last_update;

/* 发送缓冲区仅由 DartTask 使用；集中保存可避免每周期在任务栈上分配八字节数组。 */
static CAN_TxHeaderTypeDef motor_tx_header;
static uint8_t motor_tx_data[8];

/**
 * @brief 按大疆电机协议发送一组四路电流命令。
 *
 * 四个有符号电流按高字节在前打包到同一标准 CAN 帧，并根据总线编号选择 CAN1 或 CAN2。
 * 本函数只提交一次发送，不等待邮箱完成；当前调用路径由 DartTask 单独拥有发送缓冲区。
 *
 * @param can_type 目标 CAN 控制器编号。
 * @param command_id 电机组命令标准标识符。
 * @param motor1 第一通道电流命令。
 * @param motor2 第二通道电流命令。
 * @param motor3 第三通道电流命令。
 * @param motor4 第四通道电流命令。
 */
void CAN_cmd_motor(CAN_TYPE can_type,
                   can_msg_id_e command_id,
                   int16_t motor1,
                   int16_t motor2,
                   int16_t motor3,
                   int16_t motor4)
{
    uint32_t mailbox;
    motor_tx_header.StdId = command_id;
    motor_tx_header.IDE = CAN_ID_STD;
    motor_tx_header.RTR = CAN_RTR_DATA;
    motor_tx_header.DLC = 8U;

    /* 协议规定高字节在前，显式拆分可避免处理器字节序影响总线数据。 */
    motor_tx_data[0] = (uint8_t)(motor1 >> 8);
    motor_tx_data[1] = (uint8_t)motor1;
    motor_tx_data[2] = (uint8_t)(motor2 >> 8);
    motor_tx_data[3] = (uint8_t)motor2;
    motor_tx_data[4] = (uint8_t)(motor3 >> 8);
    motor_tx_data[5] = (uint8_t)motor3;
    motor_tx_data[6] = (uint8_t)(motor4 >> 8);
    motor_tx_data[7] = (uint8_t)motor4;

    CAN_HandleTypeDef *bus = can_type == CAN_1 ? &hcan1 : &hcan2;
    (void)HAL_CAN_AddTxMessage(bus, &motor_tx_header, motor_tx_data, &mailbox);
}

/**
 * @brief 解码 CAN 接收 FIFO0 中的一帧电机反馈。
 *
 * 回调先读取一帧数据，再按总线和标准标识符分派到对应电机对象。3508/6020 反馈会更新
 * 多圈编码累计值和最后在线时刻；原换弹转盘反馈只在机构编译开关启用时处理。未知标识符
 * 被安全忽略。该函数运行在 HAL 回调上下文，不执行阻塞操作。
 *
 * @param hcan 产生接收中断的 CAN 控制器句柄。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
        return;
    }

    if (hcan == &hcan1) {
        switch (header.StdId) {
            case CAN_DM4310_TURN:
                /* 达妙水平轴驱动自行解析位置、速度和力矩。 */
                DM_Motor_Decode(&YAW_Motor, CAN_1, header.StdId, data);
                break;

#if DART_ENABLE_CAROUSEL_LOADER
            case CAN_DM6006_TURN:
                /* 只有明确安装原换弹机构时才解码转盘反馈并刷新其在线时刻。 */
                DM_Motor_Decode(&turndish_dart.carousel_motor, CAN_1, header.StdId, data);
                dm6006_last_update = HAL_GetTick();
                break;
#endif

            default:
                /* 当前固件不拥有的标识直接忽略。 */
                break;
        }
        return;
    }

    if (hcan != &hcan2) {
        return;
    }

    /*
     * 数组下标属于板级资源映射：零和一是右、左推板，二是扳机，三和四是可选左右
     * 升降轴；6020 数组零号为水平轴。每帧解码后立即记录更新时间。
     */
    switch (header.StdId) {
        case CAN_3508_DRIVE_RIGHT:
            GET_MOTOR_MEASURE(&motor_3508[0], data);
            UPDATE_MOTOR_ROUND_COUNT(motor_3508[0]);
            motor_3508_last_update[0] = HAL_GetTick();
            break;
        case CAN_3508_DRIVE_LEFT:
            GET_MOTOR_MEASURE(&motor_3508[1], data);
            UPDATE_MOTOR_ROUND_COUNT(motor_3508[1]);
            motor_3508_last_update[1] = HAL_GetTick();
            break;
        case CAN_3508_TRIGGER:
            GET_MOTOR_MEASURE(&motor_3508[2], data);
            UPDATE_MOTOR_ROUND_COUNT(motor_3508[2]);
            motor_3508_last_update[2] = HAL_GetTick();
            break;
        case CAN_6020_YAW:
            GET_MOTOR_MEASURE(&motor_6020[0], data);
            UPDATE_MOTOR_ROUND_COUNT(motor_6020[0]);
            motor_6020_last_update[0] = HAL_GetTick();
            break;
        case CAN_3508_TURN_LEFT:
            GET_MOTOR_MEASURE(&motor_3508[3], data);
            UPDATE_MOTOR_ROUND_COUNT(motor_3508[3]);
            motor_3508_last_update[3] = HAL_GetTick();
            break;
        case CAN_3508_TURN_RIGHT:
            GET_MOTOR_MEASURE(&motor_3508[4], data);
            UPDATE_MOTOR_ROUND_COUNT(motor_3508[4]);
            motor_3508_last_update[4] = HAL_GetTick();
            break;
        default:
            break;
    }
}

/**
 * @brief 提交一帧通用标准标识符 CAN 数据。
 *
 * 本函数构造标准数据帧并交给 HAL 选择空发送邮箱，不等待物理发送完成。
 *
 * @param hcan 目标 CAN 控制器句柄。
 * @param id 十一位标准标识符。
 * @param data 待发送数据地址。
 * @param length 有效数据字节数，必须符合经典 CAN 单帧长度限制。
 *
 * @return
 * - 0：HAL 已接受发送请求；
 * - 1：参数、邮箱或底层 CAN 状态导致提交失败。
 */
uint8_t CANx_SendStdData(CAN_HandleTypeDef *hcan,
                         uint16_t id,
                         uint8_t *data,
                         uint16_t length)
{
    CAN_TxHeaderTypeDef header = {
        .StdId = id,
        .ExtId = 0U,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = length,
    };
    uint32_t mailbox;

    /* HAL 自动选择空邮箱；没有空邮箱或参数无效时把错误交给调用方处理。 */
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK ? 0U : 1U;
}
