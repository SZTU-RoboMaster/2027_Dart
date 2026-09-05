#ifndef DEMO1_DART_H
#define DEMO1_DART_H
#include "../Communication/can_receive.h"
#include "main.h"
#include "PID.h"
#include "../Operate/remote.h"
#include "AHRS.h"
#include "DM_MOTOR.h"
#include "stdbool.h"

#define DART_TASK_INIT_TIME 201

#define DART_LENGTH          770
#define SLIDE_LENGTH         105

//YAW轴角度环PID
#define YAW_ANGLE_PID_KP     900.0f//140
#define YAW_ANGLE_PID_KI     0.0f//0.008
#define YAW_ANGLE_PID_KD     0.0f//1800
#define YAW_ANGLE_MAX_OUT    10000.0f
#define YAW_ANGLE_MAX_IOUT   3000.0f
//YAW轴速度环PID
#define YAW_SPEED_PID_KP     75.0f//3.0
#define YAW_SPEED_PID_KI     0.0f//1.3
#define YAW_SPEED_PID_KD     0.0f//0
#define YAW_SPEED_MAX_OUT    25000.0f
#define YAW_SPEED_MAX_IOUT   25000.0f

//推动电机角度PID
#define DRIVE_ANGLE_left_PID_KP       100.0f//150
#define DRIVE_ANGLE_left_PID_KI       0.001f//0.01
#define DRIVE_ANGLE_left_PID_KD       1.0f
#define DRIVE_ANGLE_right_PID_KP      100.0f//150
#define DRIVE_ANGLE_right_PID_KI      0.001//0.01
#define DRIVE_ANGLE_right_PID_KD      1.0f
#define DRIVE_ANGLE_MAX_OUT           10000.0f
#define DRIVE_ANGLE_MAX_IOUT          3000.0f
//推动电机速度PID
#define DRIVE_SPEED_left_PID_KP       35.0f//12
#define DRIVE_SPEED_left_PID_KI       0.0f
#define DRIVE_SPEED_left_PID_KD       0.0f
#define DRIVE_SPEED_right_PID_KP      35.0f//12
#define DRIVE_SPEED_right_PID_KI      0.0f
#define DRIVE_SPEED_right_PID_KD      0.0f
#define DRIVE_SPEED_MAX_OUT           16000.0f
#define DRIVE_SPEED_MAX_IOUT          16000.0f

//扳机移动电机角度PID
#define TRIGGER_MOVE_ANGLE_PID_KP     9900.0f//10000
#define TRIGGER_MOVE_ANGLE_PID_KI     0.0f
#define TRIGGER_MOVE_ANGLE_PID_KD     0.0f
#define TRIGGER_MOVE_ANGLE_MAX_OUT    10000.0f
#define TRIGGER_MOVE_ANGLE_MAX_IOUT   8000.0f
//扳机移动电机速度PID
#define TRIGGER_MOVE_SPEED_PID_KP     20.0f    //20
#define TRIGGER_MOVE_SPEED_PID_KI     0.0f     //1.5
#define TRIGGER_MOVE_SPEED_PID_KD     0.0f
#define TRIGGER_MOVE_SPEED_MAX_OUT    16000.0f
#define TRIGGER_MOVE_SPEED_MAX_IOUT   8000.0f

//转盘角度电机角度PID
#define TURN_DISH_ANGLE_PID_KP     3000.0f//80
#define TURN_DISH_ANGLE_PID_KI     0.0f
#define TURN_DISH_ANGLE_PID_KD     0.0f
#define TURN_DISH_ANGLE_MAX_OUT    10000.0f
#define TURN_DISH_ANGLE_MAX_IOUT   3000.0f
//转盘角度电机速度PID
#define TURN_DISH_SPEED_PID_KP     30.0f//35
#define TURN_DISH_SPEED_PID_KI     0.0f
#define TURN_DISH_SPEED_PID_KD     0.0f//25
#define TURN_DISH_SPEED_MAX_OUT    16000.0f
#define TURN_DISH_SPEED_MAX_IOUT   16000.0f

#define RESET_YAW_POS                125.0f
#define MIN_YAW_POS                  15.0f
#define MAX_YAW_POS                  142.0f

#define RIGHT_YAW_POS                140.0f
#define MID_YAW_POS                  73.944f

#define RESET_PUSH_POS               5.0f

#define RESET_TRIGGER_POS            (-55.0f)

#define MAX_TRIGGER_POS              (-15.0f)
#define MIN_TRIGGER_POS              (-95.0f)

#define MAX_PUSH_POS                 675.0f
#define MIN_PUSH_POS                 5.0f

#define MIN_TURN_POS                 2.0f
#define MAX_TURN_POS                 200.0f
#define RESET_TURN_POS               3.0f

#define LIFTING_LOAD_POS             45.7069f

#define READ_YAW_PIN()   (HAL_GPIO_ReadPin(YAW_Init_GPIO_Port, YAW_Init_Pin)?1:0)

#define READ_TRIGGER_PIN()  (HAL_GPIO_ReadPin(TRIGGER_INIT_GPIO_Port,TRIGGER_INIT_Pin)?1:0)

#define READ_PUSH_L_PIN()   (HAL_GPIO_ReadPin(PUSH_INIT_L_GPIO_Port, PUSH_INIT_L_Pin)?1:0)

#define READ_PUSH_R_PIN()   (HAL_GPIO_ReadPin(PUSH_INIT_R_GPIO_Port, PUSH_INIT_R_Pin)?1:0)

#define READ_L_TURN_PIN()  (HAL_GPIO_ReadPin(TURN_L_DISH_GPIO_Port,TURN_L_DISH_Pin)?1:0)

#define READ_R_TURN_PIN()  (HAL_GPIO_ReadPin(TURN_R_DISH_GPIO_Port,TURN_R_DISH_Pin)?1:0)


#define MOTOR_PUSH_L_CHECK_POS() \
    (fabs(launcher_dart.push_motor_l.angle_p.set-launcher_dart.push_motor_l.angle_p.get)<7.0f)
#define MOTOR_PUSH_R_CHECK_POS()\
    (fabs(launcher_dart.push_motor_r.angle_p.set-launcher_dart.push_motor_r.angle_p.get)<7.0f)

#define MOTOR_YAW_CHECK_POS()  \
    (fabs(gimbal_dart.motor_yaw.angle_p.set - gimbal_dart.motor_yaw.angle_p.get)<1.0f)

#define MOTOR_TRIGGER_CHECK_POS() \
    (fabs(launcher_dart.trigger_motor.angle_p.set-launcher_dart.trigger_motor.angle_p.get)<0.5f)

#define MOTOR_L_TURN_CHECK_POS() \
(fabs(turndish_dart.turn_l_motor.angle_p.set-turndish_dart.turn_l_motor.angle_p.get)<0.3f)

#define MOTOR_R_TURN_CHECK_POS() \
(fabs(turndish_dart.turn_r_motor.angle_p.set-turndish_dart.turn_r_motor.angle_p.get)<0.3f)

#define MOTOR_DM_TURN_CHECK_POS() \
(fabs(turndish_dart.turn_angle_set - turndish_dart.dm_turn_motor.position)<0.1f)

#define LAUNCHER_MOTOR_RESET() \
do { \
    launcher_dart.trigger_motor.motor_measure->offset_ecd = \
    launcher_dart.trigger_motor.motor_measure->total_ecd; \
} while (0)


#define PUSH_L_MOTOR_RESET()\
do{\
    launcher_dart.push_motor_l.motor_measure->offset_ecd = \
    launcher_dart.push_motor_l.motor_measure->total_ecd; \
} while (0)

#define PUSH_R_MOTOR_RESET()\
do{\
    launcher_dart.push_motor_r.motor_measure->offset_ecd = \
    launcher_dart.push_motor_r.motor_measure->total_ecd; \
}while(0)

#define TRIGGER_MOTOR_RESET()\
do{\
    launcher_dart.trigger_motor.motor_measure->offset_ecd = \
    launcher_dart.trigger_motor.motor_measure->total_ecd;\
} while (0)

#define GIMBAL_MOTOR_RESET() \
do { \
    gimbal_dart.motor_yaw.motor_measure->offset_ecd = \
    gimbal_dart.motor_yaw.motor_measure->total_ecd; \
} while (0)

#define TURN_L_MOTOR_RESET()\
do{\
turndish_dart.turn_l_motor.motor_measure->offset_ecd = \
turndish_dart.turn_l_motor.motor_measure->total_ecd; \
} while (0)

#define TURN_R_MOTOR_RESET()\
do{\
turndish_dart.turn_r_motor.motor_measure->offset_ecd = \
turndish_dart.turn_r_motor.motor_measure->total_ecd; \
} while (0)

#define CLAMP_VALUE(var, max, min) \
do { \
    typeof(var) _v = (var); \
    typeof(max) _max = (max); \
    typeof(min) _min = (min); \
    (var) = ((_v > _max) ? _max : ((_v < _min) ? _min : _v)); \
} while(0)

#define GAME_MODE     1

/******************** extern *******************/

enum Dart_Mode{
    DART_RELAX=0,
    DART_BACK,
    DART_CONTROL,
    DART_GOAL_SET,
    DART_READY,
    DART_TRIGGER,//5
    DART_LAUNCH,//6
    DART_SCAN
};

enum Dart_goal{
    GOAL_NOT_SELECT=0,
    GOAL_FRONT_STATION,//前哨站
    GOAL_BASE_STATION//基地
};

enum Yaw_goal {
    YAW_MAX = 0,
    YAW_MID
};

typedef struct
{
    uint32_t id;
    fp32 pos_r;
    fp32 angular_vel;
    fp32 torque;
}Dm4310;

typedef struct{
    bool reset_push_l_flag;
    bool reset_push_r_flag;
    bool reset_trigger_flag;

    float reset_push_pos;            //只用一个就行,另一个取负值
    float max_push_pos;
    float min_push_pos;
    float reset_trigger_pos;

    float min_trigger_pos;
    float max_trigger_pos;

    motor_3508_t push_motor_r;
    motor_3508_t push_motor_l;
    motor_3508_t trigger_motor;

    void (*trigger_motor_reset)(void);
    void (*push_l_motor_reset)(void);
    void (*push_r_motor_reset)(void);

    void (*push_pid_calc)();
    void (*trigger_pid_calc)();

}Launch_t;


typedef struct{
    bool reset_yaw_flag;

    float reset_yaw_pos;
    float min_yaw_pos;
    float max_yaw_pos;
    uint8_t set_yaw_goal;

    enum Dart_Mode mode;
    enum Dart_Mode last_mode;
    motor_6020_t motor_yaw;//yaw轴电机

    void(*gimbal_motor_reset)(void);
    void (*pid_calc)();
}Gimbal_t;

typedef struct{
    motor_2006_t thrust_angle_motor;//推弹角度电机
    motor_2006_t thrust_move_motor;//推弹移动电机
    motor_2006_t trigger_motor;//扳机移动
}Thrust_t;

typedef struct {
    bool reset_l_turn_flag;
    bool reset_r_turn_flag;

    motor_3508_t turn_l_motor;
    motor_3508_t turn_r_motor;

    DM_Motor_t dm_turn_motor;

    float turn_max_pos;
    float turn_min_pos;
    float reset_turn_pos;
    uint8_t turn_angle_step;
    float turn_angle_set;
    uint8_t turn_servo_index;

    float turn_angle_ver[7];

    void (*turn_l_motor_reset)(void);
    void (*turn_r_motor_reset)(void);

    void (*turn_pid_calc)();
}Turndish_t;

typedef struct
{
    int16_t reset_speed_trigger;
    int16_t reset_speed_drive;
    int16_t reset_speed_yaw;
    int16_t init_speed_turn;
}Dart_Init_Speed_t;

typedef struct
{
    bool dart_back_flag;
}Dart_Back_t;

typedef struct
{
    bool finish_flag;
    uint8_t dart_goal;
    uint8_t launcherable_num;
    float yaw_angle_set[3];
    float trigger_distance_set[3][4];
}Dart_Goal_Set_t;

typedef struct
{
    bool launch_start_flag;
    bool launch_finish_flag;
    uint32_t launch_continue_time;
}Dart_Launch_Manage_t;

typedef struct
{
    uint8_t finish_flag;
    uint32_t first_flag;
    uint32_t first_time;
}Dart_Scan_t;

typedef struct
{
    int8_t direction;
    bool trigger_move_down_l; //确定左拨杆放下面的标志位
    bool trigger_move_mid_l;  //确定左拨杆放中间的标志位
}Dart_Trigger_t;

typedef struct
{
    uint8_t ready_process_step;
    bool ready_set_flag;
    bool ready_drive_start_flag;
    bool ready_finish_flag;
    bool ready_drive_slow_ok;
    bool ready_load_ok;
    bool ready_turn_set_ok;
    uint32_t ready_drive_down_time;
    uint32_t ready_load_time;
    uint32_t ready_turn_set_time;
    uint32_t ready_drive_up_time;
    float ready_set_drive_right_distance;
    float ready_set_drive_left_distance;
    float ready_start_drive_left_distance;
    float ready_start_drive_right_distance;
    float ready_set_lifting_distance;
    void (*set_drive_distance)(void);
    void (*set_turn_distance)(void);
    void (*set_lifting_load_distance)(void);
    void (*set_lifting_reset_distance)(void);
}Dart_Ready_Manage_t;

extern void dart_task(void const*pvParameters);

extern  Launch_t launcher_dart;
extern  Gimbal_t gimbal_dart;
extern  Thrust_t thrust_motor;
extern Dart_Goal_Set_t dart_goal_set;
extern Turndish_t turndish_dart;

#endif //DEMO1_DART_H
