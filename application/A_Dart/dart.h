#ifndef DART_H
#define DART_H

/*
 * F427 平台适配层使用的历史电机对象
 * --------------------------------
 * 新业务代码必须通过 dart_sm 和 dart_platform 工作，禁止继续向本文件增加业务状态或
 * 处理流程。保留这些精简结构，仅用于兼容 CAN 解码和少量仍读取旧全局对象的通信代码。
 */

#include <stdint.h>

#include "DM_MOTOR.h"
#include "../Communication/can_receive.h"

#define DART_TASK_INIT_TIME 201U

/* 旧 USB 与裁判代码临时读取的状态枚举，只作为观察值。 */
typedef enum {
    DART_RELAX = 0,
    DART_BACK,
    DART_CONTROL,
    DART_GOAL_SET,
    DART_READY,
    DART_TRIGGER,
    DART_LAUNCH,
    DART_SCAN
} dart_legacy_mode_t;

typedef struct {
    motor_3508_t push_motor_right;
    motor_3508_t push_motor_left;
    motor_3508_t trigger_motor;
} dart_launcher_hardware_t;

typedef struct {
    dart_legacy_mode_t mode;
    motor_6020_t yaw_motor;
} dart_gimbal_hardware_t;

typedef struct {
    motor_3508_t lift_motor_left;
    motor_3508_t lift_motor_right;
    DM_Motor_t carousel_motor;
    float carousel_target;
} dart_carousel_hardware_t;

typedef struct {
    uint8_t dart_goal;
    uint8_t launcherable_num;
} dart_legacy_status_t;

extern dart_launcher_hardware_t launcher_dart;
extern dart_gimbal_hardware_t gimbal_dart;
extern dart_carousel_hardware_t turndish_dart;
extern dart_legacy_status_t dart_goal_set;

/**
 * @brief 进入飞镖主任务运行时。
 *
 * 此函数仅保留静态任务表需要的 CMSIS-RTOS 风格签名，并把参数原样转交给运行时；
 * 实际初始化和周期控制均在 dart_runtime_task 中完成，函数不会返回。
 *
 * @param argument 任务参数，当前配置为 NULL。
 */
void dart_task(void const *argument);

#endif /* DART_H */
