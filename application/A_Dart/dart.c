#include <math.h>
#include "dart.h"
#include "cmsis_os.h"
#include "../Communication/can_receive.h"
#include "../Communication/Atti.h"
#include "protocol_shaob.h"
#include "stdlib.h"
#include "user_lib.h"
#include "tim.h"
#include "DM_MOTOR.h"
#include "bsp_servo_pwm.h"
#include "event_groups.h"
#include "Send_to_Screen.h"
#include "usart.h"
#include "zp10s_servo.h"
#include "../Referee_system/Referee.h"

extern RC_ctrl_t rc_ctrl;
extern robot_ctrl_info_t robot_ctrl;

static ZP10S_HandleTypeDef dart_bus_servo;
static bool dart_bus_servo_inited = false;

#define DART_BUS_SERVO_OPEN_PULSE_US 2000U
#define DART_BUS_SERVO_CLOSE_PULSE_US 1500U
#define DART_BUS_SERVO_MOVE_TIME_MS 500U

/*    函数及声明    */
static void dart_init();
static void dart_mode_set();
static void dart_relax_handle();
static void dart_back_handle();
static void dart_control_handle();
static void dart_goal_set_handle();
static void dart_ready_handle();
static void dart_launch_handle();
static void dart_trigger_handle();
static void dart_scan_handle();
static void dart_data_update();
static void yaw_control();
static void turn_control();
static void drive_control();
static void trigger_control();
static void dart_reset();
static fp32 trigger_distance_conversion(int32_t ecd);
static fp32 drive_distance_conversion(int32_t ecd);
static fp32 turn_distance_conversion(int32_t ecd);
static void steer_motor_control();
static void trigger_open();
static void trigger_off();
static void turn_motor_init();

static void dart_motor_loop_cal();

void motor_yaw_pid_calc();
void motor_push_pid_calc();
void motor_trigger_pid_calc();
void motor_turn_pid_calc();

static void Turn_l_motor_reset(void);
static void Turn_r_motor_reset(void);
static void Push_l_motor_reset(void);
static void Push_r_motor_reset(void);
static void Gimbal_motor_reset(void);
static void Trigger_motor_reset(void);
static void dart_pos_reset();
static void set_drive_distance();
static void set_turn_distance();
static void set_lifting_load_distance();
static void set_lifting_reset_distance();
static void dart_bus_servo_init(void);
static void dart_bus_servo_open_load(void);
static void dart_bus_servo_close_load(void);

static void dm6006_turn_ctrl_handle(void);
static float yaw_slew_step(float current, float target, float max_step);

Dart_Init_Speed_t dart_init_speed=
{
    .reset_speed_drive=-500,
    .reset_speed_yaw=-1000,
    .reset_speed_trigger = 4000,
    .init_speed_turn = -4000,
};

Gimbal_t gimbal_dart=
{
    .motor_yaw.motor_measure = &motor_6020[0],
    .pid_calc = motor_yaw_pid_calc,
    .gimbal_motor_reset = Gimbal_motor_reset,
    .reset_yaw_flag = false,
    .reset_yaw_pos = MID_YAW_POS,
    .min_yaw_pos = MIN_YAW_POS,
    .max_yaw_pos = MAX_YAW_POS,
    .set_yaw_goal = YAW_MAX,
};

Launch_t launcher_dart =
{
    .push_motor_r.motor_measure=&motor_3508[0],
    .push_motor_l.motor_measure=&motor_3508[1],
    .trigger_motor.motor_measure=&motor_3508[2],
    .push_l_motor_reset = Push_l_motor_reset,
    .push_r_motor_reset = Push_r_motor_reset,
    .trigger_motor_reset = Trigger_motor_reset,
    .push_pid_calc = motor_push_pid_calc,
    .trigger_pid_calc = motor_trigger_pid_calc,
    .reset_push_l_flag = false,
    .reset_push_r_flag = false,
    .reset_trigger_flag = false,
    .reset_push_pos = RESET_PUSH_POS,
    .reset_trigger_pos = RESET_TRIGGER_POS,
    .max_push_pos = MAX_PUSH_POS,
    .min_push_pos = MIN_PUSH_POS,
    .min_trigger_pos = MIN_TRIGGER_POS,
    .max_trigger_pos = MAX_TRIGGER_POS,
};

//电流为负值往上走
Turndish_t turndish_dart =
{
    .turn_l_motor = &motor_3508[3],
    .turn_r_motor = &motor_3508[4],

    .turn_l_motor_reset = Turn_l_motor_reset,
    .turn_r_motor_reset = Turn_r_motor_reset,
    .turn_pid_calc = motor_turn_pid_calc,

    .turn_max_pos = MAX_TURN_POS,
    .turn_min_pos = MIN_TURN_POS,
    .reset_l_turn_flag = false,
    .reset_r_turn_flag = false,
    .reset_turn_pos = RESET_TURN_POS,
    .turn_angle_set = 0,
    .turn_servo_index = 0,
};

Dart_Back_t dart_back=
{
    .dart_back_flag = false,
};

//记得改
Dart_Goal_Set_t dart_goal_set=
{
    .dart_goal = GOAL_BASE_STATION,
    .launcherable_num = 0,
    .yaw_angle_set = { MID_YAW_POS, MID_YAW_POS, MID_YAW_POS },
};

Dart_Launch_Manage_t dart_launch_manage=
{
    .launch_continue_time = 0,
    .launch_start_flag = false,
    .launch_finish_flag = false
};

Dart_Scan_t dart_scan=
{
    .finish_flag = false
};

Dart_Trigger_t dart_trigger=
{
    .direction = 0
};


Dart_Ready_Manage_t dart_ready_manage=
{
    .ready_load_ok = false ,
    .ready_set_flag = false ,
    .ready_drive_slow_ok = false,
    .ready_finish_flag = false,
    .ready_drive_start_flag = false,
    .ready_start_drive_left_distance = RESET_PUSH_POS,//上弹位置
    .ready_start_drive_right_distance = -RESET_PUSH_POS,
    .set_drive_distance = set_drive_distance,
    .set_turn_distance = set_turn_distance,
    .set_lifting_load_distance = set_lifting_load_distance,
    .set_lifting_reset_distance = set_lifting_reset_distance,
    .ready_turn_set_ok = false,
    .ready_turn_set_time = 0,
    .ready_drive_up_time = 0
};

/*      滤波      */
first_order_filter_type_t filter_yaw_in;
void dart_task(void const*pvParameters)
{
    vTaskDelay(DART_TASK_INIT_TIME);
    dart_init();//记得改
    turn_motor_init();
    //dart_reset();
    while(1)
    {
        dart_data_update();
        dart_mode_set();//记得改
        switch(gimbal_dart.mode)
        {
            case DART_RELAX:
            {
                dart_relax_handle();
                break;
            }

            case DART_BACK:
            {
                dart_back_handle();
                break;
            }

            case DART_CONTROL:
            {
                dart_control_handle();
                break;
            }

            case DART_GOAL_SET:
            {
                dart_goal_set_handle();
                break;
            }

            case DART_READY:
            {
                dart_ready_handle();
                break;
            }

            case DART_LAUNCH:
            {
                //记得改
                dart_launch_handle();
                break;
            }

            case DART_TRIGGER:
            {
                dart_trigger_handle();
                break;
            }
            case DART_SCAN:
            {
                //记得改
                dart_scan_handle();
                break;
            }
            default:{
                break;
            }
        }

        if (gimbal_dart.mode !=DART_RELAX)
        {
            dart_motor_loop_cal();
            dm6006_turn_ctrl_handle();
        }

        CAN_cmd_motor(CAN_2,
                        CAN_MOTOR_0x200_ID,
                        turndish_dart.turn_r_motor.give_current,
                        turndish_dart.turn_l_motor.give_current,
                        0,
                        0);
        CAN_cmd_motor(CAN_2,
                              CAN_MOTOR_0x1FF_ID,
                       gimbal_dart.motor_yaw.give_current,
                       launcher_dart.push_motor_r.give_current,
                       launcher_dart.trigger_motor.give_current,
                       launcher_dart.push_motor_l.give_current);

        vTaskDelay(1);
    }
}

static void turn_motor_init()
{
    //DM_Send_CMD(CAN_1, DM6006_TURN_MOTOR_ID, DM_Clear_Error_CMD);
    DM_Send_CMD(CAN_1, DM6006_TURN_MOTOR_ID, DM_Enable_CMD);
    //越大越往右（从后面看）
#if 0// AI系列
    turndish_dart.turn_angle_ver[0] = 0.95;
    turndish_dart.turn_angle_ver[1] = 2.038;
    turndish_dart.turn_angle_ver[2] = 3.05;
    turndish_dart.turn_angle_ver[3] = 4.112;
    turndish_dart.turn_angle_ver[4] = 5.15;
    turndish_dart.turn_angle_ver[5] = 6.22;
    turndish_dart.turn_angle_ver[6] = 5.15;

#else
    turndish_dart.turn_angle_ver[0] = 0.95;
    turndish_dart.turn_angle_ver[1] = 2.038;
    turndish_dart.turn_angle_ver[2] = 3.05;
    turndish_dart.turn_angle_ver[3] = 4.128;
    turndish_dart.turn_angle_ver[4] = 5.15;
    turndish_dart.turn_angle_ver[5] = 6.228;
    turndish_dart.turn_angle_ver[6] = 5.15;
#endif

}

static void dart_reset()
{
     while(1)
    {
         dart_data_update();
         turndish_dart.turn_angle_set = turndish_dart.turn_angle_ver[0];
         //判断YAW轴复位是否成功
         if (!READ_YAW_PIN()&&!gimbal_dart.reset_yaw_flag)
         {
             gimbal_dart.motor_yaw.angle_p.set=0xff;
             gimbal_dart.motor_yaw.speed_p.set = dart_init_speed.reset_speed_yaw;
         }
         if (READ_YAW_PIN()&&!gimbal_dart.reset_yaw_flag)
         {
             gimbal_dart.reset_yaw_flag = true;
             gimbal_dart.gimbal_motor_reset();
             gimbal_dart.motor_yaw.angle_p.set=gimbal_dart.reset_yaw_pos;
         }
         //判断左右两个推板是否复位成功
         if (!READ_PUSH_L_PIN()&&!launcher_dart.reset_push_l_flag)
         {
             launcher_dart.push_motor_l.angle_p.set=0xff;
             launcher_dart.push_motor_l.speed_p.set = dart_init_speed.reset_speed_drive;
         }
         if (READ_PUSH_L_PIN()&&!launcher_dart.reset_push_l_flag)
         {
             launcher_dart.reset_push_l_flag = true;
             launcher_dart.push_l_motor_reset();
             launcher_dart.push_motor_l.angle_p.set=launcher_dart.reset_push_pos;
         }
         if (!READ_PUSH_R_PIN()&&!launcher_dart.reset_push_r_flag)
         {
             launcher_dart.push_motor_r.angle_p.set=0xff;
             launcher_dart.push_motor_r.speed_p.set = -dart_init_speed.reset_speed_drive;
         }
         if (READ_PUSH_R_PIN()&&!launcher_dart.reset_push_r_flag)
         {
             launcher_dart.reset_push_r_flag = true;
             launcher_dart.push_r_motor_reset();
             launcher_dart.push_motor_r.angle_p.set=-launcher_dart.reset_push_pos;
         }
        if (!READ_L_TURN_PIN()&&!turndish_dart.reset_l_turn_flag)
        {
            turndish_dart.turn_l_motor.angle_p.set=0xff;
            turndish_dart.turn_l_motor.speed_p.set = dart_init_speed.init_speed_turn;
        }
         if (READ_L_TURN_PIN()&&!turndish_dart.reset_l_turn_flag)
         {
             turndish_dart.reset_l_turn_flag = true;
             turndish_dart.turn_l_motor_reset();
             turndish_dart.turn_l_motor.angle_p.set = turndish_dart.reset_turn_pos;

         }

         if (!READ_R_TURN_PIN()&&!turndish_dart.reset_r_turn_flag)
         {
             turndish_dart.turn_r_motor.angle_p.set=0xff;
             turndish_dart.turn_r_motor.speed_p.set = dart_init_speed.init_speed_turn;
         }
         if (READ_R_TURN_PIN()&&!turndish_dart.reset_r_turn_flag)
         {
             turndish_dart.reset_r_turn_flag = true;
             turndish_dart.turn_r_motor_reset();
             turndish_dart.turn_r_motor.angle_p.set = turndish_dart.reset_turn_pos;
         }
         //判断扳机是否复位成功
         if (!READ_TRIGGER_PIN()&&!launcher_dart.reset_trigger_flag)
         {
             launcher_dart.trigger_motor.angle_p.set = 0xff;
             launcher_dart.trigger_motor.speed_p.set = dart_init_speed.reset_speed_trigger;
         }
         if (READ_TRIGGER_PIN()&&!launcher_dart.reset_trigger_flag)
         {
             launcher_dart.reset_trigger_flag = true;
             launcher_dart.trigger_motor_reset();
             launcher_dart.trigger_motor.angle_p.set = launcher_dart.reset_trigger_pos;
         }

         if (gimbal_dart.reset_yaw_flag && launcher_dart.reset_push_l_flag &&
             launcher_dart.reset_trigger_flag && launcher_dart.reset_push_r_flag &&
             turndish_dart.reset_l_turn_flag && turndish_dart.reset_r_turn_flag &&
             MOTOR_L_TURN_CHECK_POS() && MOTOR_R_TURN_CHECK_POS() &&
             MOTOR_PUSH_L_CHECK_POS() &&MOTOR_YAW_CHECK_POS() &&
             MOTOR_TRIGGER_CHECK_POS() && MOTOR_PUSH_R_CHECK_POS())
         {
             break;
         }

         dart_motor_loop_cal();
         dm6006_turn_ctrl_handle();

         CAN_cmd_motor(CAN_2,
                              CAN_MOTOR_0x1FF_ID,
                       gimbal_dart.motor_yaw.give_current,
                       launcher_dart.push_motor_r.give_current,
                       launcher_dart.trigger_motor.give_current,
                       launcher_dart.push_motor_l.give_current);
          CAN_cmd_motor(CAN_2,
                         CAN_MOTOR_0x200_ID,
                         turndish_dart.turn_r_motor.give_current,
                         turndish_dart.turn_l_motor.give_current,
                         0,
                         0);
         vTaskDelay(1);
    }
}

static void dart_scan_handle()
{
    if (dart_progress_mod == 0xff) {
        return;
    }
#if 0
    dart_scan.finish_flag = true;
#else
    //制导用
    // if (dart_goal_set.launcherable_num <= 1 ) {
    //     gimbal_dart.motor_yaw.angle_p.set = 103.5f;
    //     if (MOTOR_YAW_CHECK_POS()) {
    //         dart_scan.finish_flag = true;
    //         return;
    //     }
    // }
    if (dart_goal_set.dart_goal != GOAL_NOT_SELECT)
    {
        if (!robot_ctrl.target_lock && !dart_scan.finish_flag) {
            switch (gimbal_dart.set_yaw_goal) {
                case YAW_MAX:
                    gimbal_dart.motor_yaw.angle_p.set = RIGHT_YAW_POS;
                    if (MOTOR_YAW_CHECK_POS()) {
                        gimbal_dart.set_yaw_goal = YAW_MID;
                    }
                    break;
                case YAW_MID:
                    gimbal_dart.motor_yaw.angle_p.set = MID_YAW_POS;
                    if (MOTOR_YAW_CHECK_POS()) {
                        gimbal_dart.set_yaw_goal = YAW_MAX;
                    }
                    break;
            }
        }
        if (robot_ctrl.target_lock && !dart_scan.finish_flag)
        {

            gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get + filter_yaw_in.out;
            if(fabs(filter_yaw_in.out)<=0.05)
            {
                gimbal_dart.motor_yaw.angle_p.set=gimbal_dart.motor_yaw.angle_p.get;
                if (MOTOR_YAW_CHECK_POS() && !dart_scan.first_flag) {
                    dart_scan.first_flag = true;
                    dart_scan.first_time = HAL_GetTick();
                }
                if (dart_scan.first_flag && HAL_GetTick() - dart_scan.first_time >= 500) {
                    dart_scan.finish_flag = true;
                }
            }
        }
        if (dart_scan.finish_flag && fabs(filter_yaw_in.out)<=0.05) {
            gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get;
            if (MOTOR_YAW_CHECK_POS() && !dart_scan.first_flag) {
                dart_scan.first_flag = true;
                dart_scan.first_time = HAL_GetTick();
            }
            if (dart_scan.first_flag && HAL_GetTick() - dart_scan.first_time >= 500) {
                dart_scan.finish_flag = true;
            }
        }
        if (dart_scan.finish_flag && fabs(filter_yaw_in.out)>0.1) {
            dart_scan.finish_flag = false;
            dart_scan.first_flag = false;
            dart_scan.first_time = HAL_GetTick();
        }
    }
#endif
}

static void dart_trigger_handle()
{
    if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))
    {
        dart_trigger.direction=1;
    }
    if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        dart_trigger.direction=-1;
    }
    if(dart_trigger.trigger_move_mid_l==1 && dart_trigger.trigger_move_down_l==1)
    {
        if(dart_trigger.direction==1)
        {
            launcher_dart.trigger_motor.angle_p.set=launcher_dart.trigger_motor.angle_p.get+1.0f;
            for (int i = 0; i < 4; i++) {
                dart_goal_set.trigger_distance_set[dart_goal_set.dart_goal][i]+=1;
            }
        }
        if(dart_trigger.direction==-1)
        {
            launcher_dart.trigger_motor.angle_p.set=launcher_dart.trigger_motor.angle_p.get-1.0f;
            for (int i = 0; i < 4; i++) {
                dart_goal_set.trigger_distance_set[dart_goal_set.dart_goal][i]+=1;
            }
        }
        dart_trigger.trigger_move_down_l=0;
        dart_trigger.trigger_move_mid_l=0;
    }
    if(switch_is_mid(rc_ctrl.rc.s[RC_s_L]))
    {
        dart_trigger.trigger_move_mid_l=1;
    }
    if(switch_is_down(rc_ctrl.rc.s[RC_s_L]))
    {
        dart_trigger.trigger_move_down_l=1;
    }
}

static void dart_launch_handle()
{
    trigger_open();

    if (dart_launch_manage.launch_start_flag == false)
    {
        dart_goal_set.launcherable_num++;
        dart_launch_manage.launch_continue_time = HAL_GetTick();
        dart_launch_manage.launch_start_flag = true;
    }

    if (HAL_GetTick() - dart_launch_manage.launch_continue_time>=3000)
    {
        dart_launch_manage.launch_start_flag = false;

        if (dart_goal_set.launcherable_num == 3 || dart_goal_set.launcherable_num == 1)
        {
            gimbal_dart.mode = DART_READY;
        }
        else if (dart_goal_set.launcherable_num == 2)
        {
            gimbal_dart.mode = DART_BACK;
            dart_goal_set.dart_goal = GOAL_BASE_STATION;
            //dart_goal_set.dart_goal = GOAL_NOT_SELECT;
        }
        else if (dart_goal_set.launcherable_num == 4)
        {
            dart_back.dart_back_flag = false;
            gimbal_dart.mode = DART_BACK;
            dart_goal_set.launcherable_num = 0;
            dart_goal_set.dart_goal = GOAL_NOT_SELECT;
            dart_ready_manage.ready_process_step = 0;
            turndish_dart.turn_angle_step = 0;
        }
    }
}

static void set_drive_distance()
{
    dart_ready_manage.ready_set_drive_left_distance = (DART_LENGTH-SLIDE_LENGTH-10.5);
    dart_ready_manage.ready_set_drive_right_distance = -(DART_LENGTH-SLIDE_LENGTH-10.5);
}

static void set_turn_distance() {
    dart_ready_manage.ready_set_drive_left_distance = (DART_LENGTH-SLIDE_LENGTH - 10 - 108.95);//43
    dart_ready_manage.ready_set_drive_right_distance = -(DART_LENGTH-SLIDE_LENGTH-10 - 108.95);
}

static void set_lifting_load_distance() {
    dart_ready_manage.ready_set_lifting_distance = LIFTING_LOAD_POS;
}

static void dart_bus_servo_init(void)
{
    if (!dart_bus_servo_inited)
    {
        ZP10S_Init(&dart_bus_servo, &huart8);
        ZP10S_SetTimeouts(&dart_bus_servo, 20U, 30U);
        dart_bus_servo_inited = true;
    }
}

static void dart_bus_servo_open_load(void)
{
    dart_bus_servo_init();
    (void)ZP10S_Move(&dart_bus_servo, turndish_dart.turn_servo_index, DART_BUS_SERVO_OPEN_PULSE_US, DART_BUS_SERVO_MOVE_TIME_MS);
}

static void dart_bus_servo_close_load(void)
{
    dart_bus_servo_init();
    (void)ZP10S_Move(&dart_bus_servo, 1U, DART_BUS_SERVO_CLOSE_PULSE_US, DART_BUS_SERVO_MOVE_TIME_MS);
    (void)ZP10S_Move(&dart_bus_servo, 2U, DART_BUS_SERVO_CLOSE_PULSE_US, DART_BUS_SERVO_MOVE_TIME_MS);
    (void)ZP10S_Move(&dart_bus_servo, 3U, DART_BUS_SERVO_CLOSE_PULSE_US, DART_BUS_SERVO_MOVE_TIME_MS);
}
static void set_lifting_reset_distance() {
    dart_ready_manage.ready_set_lifting_distance = RESET_TURN_POS;
}

static void trigger_open()
{
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);
}

static void trigger_off()
{
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,1200);
}

static void dart_ready()
{
    switch (dart_ready_manage.ready_process_step)
    {
        case 0:
            {
                dart_ready_manage.set_drive_distance();
                dart_ready_manage.ready_process_step ++;
            }
        break;

        case 1:
            {
                launcher_dart.push_motor_l.angle_p.set = dart_ready_manage.ready_start_drive_left_distance;
                launcher_dart.push_motor_r.angle_p.set = dart_ready_manage.ready_start_drive_right_distance;
                if (MOTOR_PUSH_L_CHECK_POS()&&MOTOR_PUSH_R_CHECK_POS())
                {
                    dart_ready_manage.ready_process_step ++;
                }
            }
        break;

        case 2:
            {
                if (dart_ready_manage.ready_drive_slow_ok == false)
                {
                    dart_ready_manage.ready_drive_down_time = HAL_GetTick();
                    dart_ready_manage.ready_drive_slow_ok =true;
                } //记得补一下卡弹检测
                if (HAL_GetTick() - dart_ready_manage.ready_drive_down_time <1000)
                {
                    launcher_dart.push_motor_l.angle_p.set = 0xff;
                    launcher_dart.push_motor_l.speed_p.set = 2000;
                    launcher_dart.push_motor_r.angle_p.set = 0xff;
                    launcher_dart.push_motor_r.speed_p.set = -2000;
                }
                else
                {
                    launcher_dart.push_motor_l.angle_p.set = dart_ready_manage.ready_set_drive_left_distance;
                    launcher_dart.push_motor_r.angle_p.set = dart_ready_manage.ready_set_drive_right_distance;
                    if (MOTOR_PUSH_L_CHECK_POS()&&MOTOR_PUSH_R_CHECK_POS())
                    {
                        trigger_off();
                        if (!dart_ready_manage.ready_load_ok)
                        {
                            dart_ready_manage.ready_load_time = HAL_GetTick();
                            dart_ready_manage.ready_load_ok = true;
                        }
                    }
                    if ((HAL_GetTick() - dart_ready_manage.ready_load_time > 1000) && dart_ready_manage.ready_load_ok)
                    {
                        dart_ready_manage.ready_process_step++;
                    }
                }
            }
        break;

        case 3:
            {
                launcher_dart.push_motor_l.angle_p.set = launcher_dart.reset_push_pos;
                launcher_dart.push_motor_r.angle_p.set = -launcher_dart.reset_push_pos;
                launcher_dart.trigger_motor.angle_p.set = dart_goal_set.trigger_distance_set[dart_goal_set.dart_goal][dart_goal_set.launcherable_num];
                if (MOTOR_PUSH_L_CHECK_POS() && MOTOR_PUSH_R_CHECK_POS() && MOTOR_TRIGGER_CHECK_POS())
                {
                    if (dart_goal_set.launcherable_num == 0)
                    {
                        dart_ready_manage.ready_finish_flag = true;
                    }else
                    {
                        //dart_ready_manage.ready_process_step++;
                        dart_ready_manage.ready_finish_flag = true;
                    }
                }
                break;
            }

        case 4:
            {
                // dart_goal_set.launcherable_num = 2;
                // //机械臂上弹部分
                // switch (dart_goal_set.launcherable_num)
                // {
                // case 1:
                //     xEventGroupSetBits(g_arm_cmd,ARM_CMD_START_0);
                //     break;
                //
                // case 2:
                //     xEventGroupSetBits(g_arm_cmd,ARM_CMD_START_1);
                //     break;
                //
                // case 3:
                //     xEventGroupSetBits(g_arm_cmd,ARM_CMD_START_2);
                //     break;
                // }
                // EventBits_t ack = xEventGroupGetBits(g_arm_ack);
                // if (ack & ARM_ACK_DONE)
                // {
                //     dart_ready_manage.ready_finish_flag = true;
                //     xEventGroupClearBits(g_arm_cmd, ARM_CMD_START_0 | ARM_CMD_START_1 | ARM_CMD_START_2);
                //     xEventGroupClearBits(g_arm_ack, ARM_ACK_DONE | ARM_ACK_FAIL);
                // }
                // break;
            }
        break;
        // case 0:
        //     {
        //         gimbal_dart.motor_yaw.angle_p.set = dart_goal_set.yaw_angle_set[dart_goal_set.dart_goal];
        //         launcher_dart.trigger_motor.angle_p.set = dart_goal_set.trigger_distance_set[dart_goal_set.dart_goal][dart_goal_set.launcherable_num];
        //         if (MOTOR_YAW_CHECK_POS()&&MOTOR_TRIGGER_CHECK_POS())
        //         {
        //             trigger_open();
        //             dart_ready_manage.set_drive_distance();
        //             dart_ready_manage.ready_process_step ++;
        //         }
        //     }
        // break;
        //
        // case 1:
        //     {
        //         launcher_dart.push_motor_l.angle_p.set = dart_ready_manage.ready_start_drive_left_distance;
        //         launcher_dart.push_motor_r.angle_p.set = dart_ready_manage.ready_start_drive_right_distance;
        //         if (MOTOR_PUSH_L_CHECK_POS()&&MOTOR_PUSH_R_CHECK_POS())
        //         {
        //             dart_ready_manage.ready_process_step ++;
        //         }
        //     }
        // break;
        //
        // case 2:
        //     {
        //         if (dart_ready_manage.ready_drive_slow_ok == false)
        //         {
        //             dart_ready_manage.ready_drive_down_time = HAL_GetTick();
        //             dart_ready_manage.ready_drive_slow_ok =true;
        //         } //记得补一下卡弹检测
        //         if (HAL_GetTick() - dart_ready_manage.ready_drive_down_tim33e <1000)
        //         {
        //             launcher_dart.push_motor_l.angle_p.set = 0xff;
        //             launcher_dart.push_motor_l.speed_p.set = 2000;
        //             launcher_dart.push_motor_r.angle_p.set = 0xff;
        //             launcher_dart.push_motor_r.speed_p.set = -2000;
        //         }
        //         else
        //         {
        //             launcher_dart.push_motor_l.angle_p.set = dart_ready_manage.ready_set_drive_left_distance;
        //             launcher_dart.push_motor_r.angle_p.set = dart_ready_manage.ready_set_drive_right_distance;
        //             if (MOTOR_PUSH_L_CHECK_POS()&&MOTOR_PUSH_R_CHECK_POS())
        //             {
        //                 trigger_off();
        //                 if (!dart_ready_manage.ready_load_ok)
        //                 {
        //                     dart_ready_manage.ready_load_time = HAL_GetTick();
        //                     dart_ready_manage.ready_load_ok = true;
        //                 }
        //             }
        //             if ((HAL_GetTick() - dart_ready_manage.ready_load_time > 1000) && dart_ready_manage.ready_load_ok)
        //             {
        //                 if (dart_goal_set.launcherable_num == 0)
        //                 {
        //                     dart_ready_manage.ready_process_step = 4;
        //                 }else
        //                 {
        //                     dart_ready_manage.ready_process_step =4;
        //                 }
        //             }
        //         }
        //     }
        // break;
        //
        // case 3:
        //     {
        //         //机械臂上弹部分
        //         switch (dart_goal_set.launcherable_num)
        //         {
        //         case 1:
        //             xEventGroupSetBits(g_arm_cmd,ARM_CMD_START_0);
        //             break;
        //
        //         case 2:
        //             xEventGroupSetBits(g_arm_cmd,ARM_CMD_START_1);
        //             break;
        //
        //         case 3:
        //             xEventGroupSetBits(g_arm_cmd,ARM_CMD_START_2);
        //             break;
        //         }
        //         EventBits_t ack = xEventGroupGetBits(g_arm_ack);
        //         if (ack & ARM_ACK_DONE)
        //         {
        //             dart_ready_manage.ready_process_step++;
        //             xEventGroupClearBits(g_arm_cmd, ARM_CMD_START_0 | ARM_CMD_START_1 | ARM_CMD_START_2);
        //             xEventGroupClearBits(g_arm_ack, ARM_ACK_DONE | ARM_ACK_FAIL);
        //         }
        //         break;
        //     }
        //
        // case 4:
        //     {
        //         launcher_dart.push_motor_l.angle_p.set = launcher_dart.reset_push_pos;
        //         launcher_dart.push_motor_r.angle_p.set = -launcher_dart.reset_push_pos;
        //         if (MOTOR_PUSH_L_CHECK_POS() && MOTOR_PUSH_R_CHECK_POS())
        //         {
        //             dart_ready_manage.ready_finish_flag = true;
        //         }
        //     }
        // break;
    }
}

static void dart_ready_turn()
{
    //gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get;
    switch (dart_ready_manage.ready_process_step) {
        case 0:
        {
            dart_ready_manage.set_turn_distance();
            dart_ready_manage.ready_process_step ++;
        }
            break;
        case 1:
        {
            launcher_dart.push_motor_l.angle_p.set = dart_ready_manage.ready_start_drive_left_distance;
            launcher_dart.push_motor_r.angle_p.set = dart_ready_manage.ready_start_drive_right_distance;
            launcher_dart.trigger_motor.angle_p.set = dart_goal_set.trigger_distance_set[dart_goal_set.dart_goal][dart_goal_set.launcherable_num];
            if (MOTOR_PUSH_L_CHECK_POS()&&MOTOR_PUSH_R_CHECK_POS()&&MOTOR_TRIGGER_CHECK_POS())
            {
                if (dart_goal_set.launcherable_num!=0)
                    dart_ready_manage.ready_process_step ++;
                else {
                    dart_ready_manage.ready_process_step = 7;
                    dart_ready_manage.set_drive_distance();
                }

            }
        }
            break;
        //下到上弹位置
        case 2:
        {
            if (dart_ready_manage.ready_drive_slow_ok == false)
            {
                dart_ready_manage.ready_drive_down_time = HAL_GetTick();
                dart_ready_manage.ready_drive_slow_ok =true;
            } //记得补一下卡弹检测
            if (HAL_GetTick() - dart_ready_manage.ready_drive_down_time <1000)
            {
                launcher_dart.push_motor_l.angle_p.set = 0xff;
                launcher_dart.push_motor_l.speed_p.set = 2000;
                launcher_dart.push_motor_r.angle_p.set = 0xff;
                launcher_dart.push_motor_r.speed_p.set = -2000;
            }
            else
            {
                launcher_dart.push_motor_l.angle_p.set = dart_ready_manage.ready_set_drive_left_distance;
                launcher_dart.push_motor_r.angle_p.set = dart_ready_manage.ready_set_drive_right_distance;
                if (MOTOR_PUSH_L_CHECK_POS()&&MOTOR_PUSH_R_CHECK_POS()&&!dart_ready_manage.ready_turn_set_ok)
                {
                    turndish_dart.turn_angle_set = turndish_dart.turn_angle_ver[++turndish_dart.turn_angle_step];
                    dart_ready_manage.ready_turn_set_ok = true;
                    dart_ready_manage.ready_turn_set_time = HAL_GetTick();
                }
                if ( MOTOR_DM_TURN_CHECK_POS() && dart_ready_manage.ready_turn_set_ok&& (HAL_GetTick() - dart_ready_manage.ready_turn_set_time > 100))
                {
                    dart_ready_manage.ready_process_step++;
                    dart_ready_manage.ready_turn_set_ok = false;
                }
            }
        }
            break;

        //上弹
        case 3:
        {
            dart_ready_manage.set_lifting_load_distance();
            turndish_dart.turn_l_motor.angle_p.set = dart_ready_manage.ready_set_lifting_distance;
            turndish_dart.turn_r_motor.angle_p.set = dart_ready_manage.ready_set_lifting_distance;
            if (MOTOR_L_TURN_CHECK_POS() && MOTOR_R_TURN_CHECK_POS()) {
                //开舵机；
                if (!dart_ready_manage.ready_load_ok)
                {
                    turndish_dart.turn_servo_index ++;
                    dart_bus_servo_open_load();
                    dart_ready_manage.ready_load_time = HAL_GetTick();
                    dart_ready_manage.ready_load_ok = true;
                }
                if (dart_ready_manage.ready_load_ok && (HAL_GetTick() - dart_ready_manage.ready_load_time > 1500)) {
                    dart_ready_manage.ready_process_step++;
                    dart_ready_manage.ready_load_ok = false;
                }
            }
        }
            break;
        case 4:
        {
            dart_ready_manage.set_drive_distance();
            launcher_dart.push_motor_l.angle_p.set = dart_ready_manage.ready_set_drive_left_distance;
            launcher_dart.push_motor_r.angle_p.set = dart_ready_manage.ready_set_drive_right_distance;

            if (MOTOR_PUSH_L_CHECK_POS() && MOTOR_PUSH_R_CHECK_POS()) {
                trigger_off();
                if (!dart_ready_manage.ready_load_ok) {
                    dart_ready_manage.ready_load_time = HAL_GetTick();
                    dart_ready_manage.ready_load_ok = true;
                }
            }

            if ((HAL_GetTick() - dart_ready_manage.ready_load_time > 2000) &&
                dart_ready_manage.ready_load_ok &&
                MOTOR_DM_TURN_CHECK_POS() )
            {
                dart_ready_manage.ready_process_step++;
                dart_ready_manage.ready_turn_set_ok = false;
                dart_ready_manage.ready_load_ok = false;
            }
        }
            break;

        case 5:
        {
            if (!dart_ready_manage.ready_turn_set_ok) {
                turndish_dart.turn_angle_set = turndish_dart.turn_angle_ver[++turndish_dart.turn_angle_step];
                dart_ready_manage.ready_turn_set_ok = true;
            }

            dart_ready_manage.set_lifting_reset_distance();
            turndish_dart.turn_l_motor.angle_p.set = dart_ready_manage.ready_set_lifting_distance;
            turndish_dart.turn_r_motor.angle_p.set = dart_ready_manage.ready_set_lifting_distance;

            if (MOTOR_L_TURN_CHECK_POS() && MOTOR_R_TURN_CHECK_POS()) {
                dart_ready_manage.ready_process_step++;
                dart_ready_manage.ready_turn_set_ok = false;
            }
        }
            break;

        case 6:
        {
            launcher_dart.push_motor_l.angle_p.set = launcher_dart.reset_push_pos;
            launcher_dart.push_motor_r.angle_p.set = -launcher_dart.reset_push_pos;
            launcher_dart.trigger_motor.angle_p.set = dart_goal_set.trigger_distance_set[dart_goal_set.dart_goal][dart_goal_set.launcherable_num];
            if (MOTOR_PUSH_L_CHECK_POS() && MOTOR_PUSH_R_CHECK_POS() && MOTOR_TRIGGER_CHECK_POS()) {
                if (dart_goal_set.launcherable_num == 0)
                {
                    dart_ready_manage.ready_finish_flag = true;
                }else
                {
                    //dart_ready_manage.ready_process_step++;
                    dart_ready_manage.ready_finish_flag = true;
                }
            }
            (void)ZP10S_Move(&dart_bus_servo, 1U, DART_BUS_SERVO_CLOSE_PULSE_US, DART_BUS_SERVO_MOVE_TIME_MS);
            (void)ZP10S_Move(&dart_bus_servo, 2U, DART_BUS_SERVO_CLOSE_PULSE_US, DART_BUS_SERVO_MOVE_TIME_MS);
            (void)ZP10S_Move(&dart_bus_servo, 3U, DART_BUS_SERVO_CLOSE_PULSE_US, DART_BUS_SERVO_MOVE_TIME_MS);
        }
            break;
        case 7:
        {
            if (dart_ready_manage.ready_drive_slow_ok == false)
            {
                dart_ready_manage.ready_drive_down_time = HAL_GetTick();
                dart_ready_manage.ready_drive_slow_ok =true;
            } //记得补一下卡弹检测
            if (HAL_GetTick() - dart_ready_manage.ready_drive_down_time <1000)
            {
                launcher_dart.push_motor_l.angle_p.set = 0xff;
                launcher_dart.push_motor_l.speed_p.set = 2000;
                launcher_dart.push_motor_r.angle_p.set = 0xff;
                launcher_dart.push_motor_r.speed_p.set = -2000;
            }
            else
            {
                launcher_dart.push_motor_l.angle_p.set = dart_ready_manage.ready_set_drive_left_distance;
                launcher_dart.push_motor_r.angle_p.set = dart_ready_manage.ready_set_drive_right_distance;
                if (MOTOR_PUSH_L_CHECK_POS()&&MOTOR_PUSH_R_CHECK_POS())
                {
                    trigger_off();
                    if (!dart_ready_manage.ready_load_ok)
                    {
                        dart_ready_manage.ready_load_time = HAL_GetTick();
                        dart_ready_manage.ready_load_ok = true;
                    }
                }
                if ((HAL_GetTick() - dart_ready_manage.ready_load_time > 1500) && dart_ready_manage.ready_load_ok)
                {
                    dart_ready_manage.ready_process_step = 6;
                    dart_ready_manage.ready_load_ok = false;
                }
            }
        }
            break;

    }
}

static void dart_ready_handle()
{
    //dart_ready();
    dart_ready_turn();
}

static void dart_goal_set_handle()
{
    if (dart_goal_set.finish_flag == false)
    {
        gimbal_dart.motor_yaw.angle_p.set = dart_goal_set.yaw_angle_set[dart_goal_set.dart_goal];
        launcher_dart.trigger_motor.angle_p.set=dart_goal_set.trigger_distance_set[dart_goal_set.dart_goal][dart_goal_set.launcherable_num];
    }

    if (MOTOR_YAW_CHECK_POS()&&MOTOR_TRIGGER_CHECK_POS())
    {
        dart_goal_set.finish_flag = true;
    }
}

static fp32 yaw_distance_conversion(int32_t ecd)
{
    return (fp32)ecd/8189*5;
}

//已完成
static void dart_control_handle()
{
    if(rc_ctrl.rc.ch[4]<-500)
    {
        trigger_off();
    }
    if(rc_ctrl.rc.ch[4]>500)
    {
        trigger_open();
    }
    yaw_control();
    turn_control();
    drive_control();
    trigger_control();
    //turndish_dart.turn_angle_set = turndish_dart.turn_angle_ver[0];
}


static fp32 trigger_distance_conversion(int32_t ecd)
{
    return (fp32)ecd/8192/19*5;
}

static fp32 drive_distance_conversion(int32_t ecd)
{
    //return (fp32)ecd/8192/3591*187*27*PI;
    return ((fp32)ecd/8192/27)*19.32*2*PI;
}

static fp32 turn_distance_conversion(int32_t ecd)
{
    //return (fp32)ecd/8192/3591*187*27*PI;
    return (fp32)ecd/8192/19*5;
}

static void trigger_control()
{
    if(rc_ctrl.rc.ch[1]<=20&&rc_ctrl.rc.ch[1]>=-20)
    {
        rc_ctrl.rc.ch[1]=0;
    }
    launcher_dart.trigger_motor.angle_p.set-=rc_ctrl.rc.ch[1]*0.0001;
}

static void drive_control()
{

    if(rc_ctrl.rc.ch[3]<=40&&rc_ctrl.rc.ch[3]>=-40)
    {
        rc_ctrl.rc.ch[3]=0;
    }

    launcher_dart.push_motor_l.angle_p.set += rc_ctrl.rc.ch[3] * 0.0005;
    launcher_dart.push_motor_r.angle_p.set-=rc_ctrl.rc.ch[3]*0.0005;

}

static void turn_control()
{

    if(rc_ctrl.rc.ch[0]<=40&&rc_ctrl.rc.ch[0]>=-40)
    {
        rc_ctrl.rc.ch[0]=0;
    }

    turndish_dart.turn_l_motor.angle_p.set += rc_ctrl.rc.ch[0] * 0.0005;
    turndish_dart.turn_r_motor.angle_p.set +=rc_ctrl.rc.ch[0]*0.0005;
}

static void yaw_control()
{
#if 0
    if (!robot_ctrl.target_lock) {
        switch (gimbal_dart.set_yaw_goal) {
            case YAW_MAX:
                gimbal_dart.motor_yaw.angle_p.set = MAX_YAW_POS;
                if (MOTOR_YAW_CHECK_POS()) {
                    gimbal_dart.set_yaw_goal = YAW_MID;
                }
                break;
            case YAW_MID:
                gimbal_dart.motor_yaw.angle_p.set = MID_YAW_POS;
                if (MOTOR_YAW_CHECK_POS()) {
                    gimbal_dart.set_yaw_goal = YAW_MAX;
                }
                break;
        }
    }
    if (robot_ctrl.target_lock && fabs(filter_yaw_in.out)>=0.05)
    {
        gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get + filter_yaw_in.out;
    }
    if (robot_ctrl.target_lock && fabs(filter_yaw_in.out)<=0.05) {
        gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get;
    }
#else
    if(rc_ctrl.rc.ch[2]<=40&&rc_ctrl.rc.ch[2]>=-40)
    {
        rc_ctrl.rc.ch[2]=0;
    }
    gimbal_dart.motor_yaw.angle_p.set += rc_ctrl.rc.ch[2] * 0.01*0.005;
#endif
}

static void dart_back_handle()
{
    DM_Send_CMD(CAN_1, DM6006_TURN_MOTOR_ID, DM_Enable_CMD);

    turndish_dart.turn_angle_set = turndish_dart.turn_angle_ver[turndish_dart.turn_angle_step];

    if (!dart_back.dart_back_flag)
    {
        dart_pos_reset();
        trigger_open();
    }
    if (MOTOR_PUSH_L_CHECK_POS() && MOTOR_YAW_CHECK_POS() &&
        MOTOR_PUSH_R_CHECK_POS() && MOTOR_TRIGGER_CHECK_POS() &&
        MOTOR_DM_TURN_CHECK_POS()&&
        !dart_back.dart_back_flag)
    {
        dart_back.dart_back_flag = true;
    }
}
static void dart_relax_handle()
{
    launcher_dart.push_motor_r.give_current=0;
    launcher_dart.push_motor_l.give_current=0;
    gimbal_dart.motor_yaw.give_current=0;
    launcher_dart.trigger_motor.give_current=0;
    turndish_dart.turn_l_motor.give_current=0;
    turndish_dart.turn_r_motor.give_current=0;
    DM_Send_CMD(CAN_1, DM6006_TURN_MOTOR_ID, DM_Disable_CMD);
    trigger_open();
    if (dart_goal_set.launcherable_num>=4) {
        dart_goal_set.launcherable_num = 0;
    }
    dart_ready_manage.ready_process_step = 0;

}

static void dart_init()
{
    dart_bus_servo_close_load();
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
    trigger_open();
    //模式初始化
    gimbal_dart.mode=DART_RELAX;
    //pid初始化
    pid_init(&launcher_dart.push_motor_r.angle_p,
             DRIVE_ANGLE_MAX_OUT,
             DRIVE_ANGLE_MAX_IOUT,
             DRIVE_ANGLE_right_PID_KP,
             DRIVE_ANGLE_right_PID_KI,
             DRIVE_ANGLE_right_PID_KD);

    pid_init(&launcher_dart.push_motor_r.speed_p,
             DRIVE_SPEED_MAX_OUT,
             DRIVE_SPEED_MAX_IOUT,
             DRIVE_SPEED_right_PID_KP,
             DRIVE_SPEED_right_PID_KI,
             DRIVE_SPEED_right_PID_KD);

    pid_init(&launcher_dart.push_motor_l.angle_p,
             DRIVE_ANGLE_MAX_OUT,
             DRIVE_ANGLE_MAX_IOUT,
             DRIVE_ANGLE_left_PID_KP,
             DRIVE_ANGLE_left_PID_KI,
             DRIVE_ANGLE_left_PID_KD);

    pid_init(&launcher_dart.push_motor_l.speed_p,
             DRIVE_SPEED_MAX_OUT,
             DRIVE_SPEED_MAX_IOUT,
             DRIVE_SPEED_left_PID_KP,
             DRIVE_SPEED_left_PID_KI,
             DRIVE_SPEED_left_PID_KD);

    pid_init(&gimbal_dart.motor_yaw.angle_p,
             YAW_ANGLE_MAX_OUT,
             YAW_ANGLE_MAX_IOUT,
             YAW_ANGLE_PID_KP,
             YAW_ANGLE_PID_KI,
             YAW_ANGLE_PID_KD);

    pid_init(&gimbal_dart.motor_yaw.speed_p,
             YAW_SPEED_MAX_OUT,
             YAW_SPEED_MAX_IOUT,
             YAW_SPEED_PID_KP,
             YAW_SPEED_PID_KI,
             YAW_SPEED_PID_KD);

    pid_init(&launcher_dart.trigger_motor.angle_p,
             TRIGGER_MOVE_ANGLE_MAX_OUT,
             TRIGGER_MOVE_ANGLE_MAX_IOUT,
             TRIGGER_MOVE_ANGLE_PID_KP,
             TRIGGER_MOVE_ANGLE_PID_KI,
             TRIGGER_MOVE_ANGLE_PID_KD);

    pid_init(&launcher_dart.trigger_motor.speed_p,
             TRIGGER_MOVE_SPEED_MAX_OUT,
             TRIGGER_MOVE_SPEED_MAX_IOUT,
             TRIGGER_MOVE_SPEED_PID_KP,
             TRIGGER_MOVE_SPEED_PID_KI,
             TRIGGER_MOVE_SPEED_PID_KD);

    pid_init(&turndish_dart.turn_l_motor.angle_p,
             TURN_DISH_ANGLE_MAX_OUT,
             TURN_DISH_ANGLE_MAX_IOUT,
             TURN_DISH_ANGLE_PID_KP,
             TURN_DISH_ANGLE_PID_KI,
             TURN_DISH_ANGLE_PID_KD);

    pid_init(&turndish_dart.turn_l_motor.speed_p,
             TURN_DISH_SPEED_MAX_OUT,
             TURN_DISH_SPEED_MAX_IOUT,
             TURN_DISH_SPEED_PID_KP,
             TURN_DISH_SPEED_PID_KI,
             TURN_DISH_SPEED_PID_KD);

    pid_init(&turndish_dart.turn_r_motor.angle_p,
             TURN_DISH_ANGLE_MAX_OUT,
             TURN_DISH_ANGLE_MAX_IOUT,
             TURN_DISH_ANGLE_PID_KP,
             TURN_DISH_ANGLE_PID_KI,
             TURN_DISH_ANGLE_PID_KD);

    pid_init(&turndish_dart.turn_r_motor.speed_p,
             TURN_DISH_SPEED_MAX_OUT,
             TURN_DISH_SPEED_MAX_IOUT,
             TURN_DISH_SPEED_PID_KP,
             TURN_DISH_SPEED_PID_KI,
             TURN_DISH_SPEED_PID_KD);

    first_order_filter_init(&filter_yaw_in,5,30);

#if 0   //AI系列
    dart_goal_set.trigger_distance_set[GOAL_NOT_SELECT][0] = -68.6753;
    dart_goal_set.trigger_distance_set[GOAL_NOT_SELECT][1] = -63.0224792;
    dart_goal_set.trigger_distance_set[GOAL_NOT_SELECT][2] = -67.0299621;
    dart_goal_set.trigger_distance_set[GOAL_NOT_SELECT][3] = -64.0862838;

    dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][0] = -59.6753;
    dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][1] = -60.2224792;
    dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][2] = -60.9299621;
    dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][3] = -61.8862838;

    dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][0] = -82.00;
    dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][1] = -80.70;
    dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][2] = -81.00;
    dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][3] = -79.40;
#else//Q系列
    dart_goal_set.trigger_distance_set[GOAL_NOT_SELECT][0] = -68.2753;
    dart_goal_set.trigger_distance_set[GOAL_NOT_SELECT][1] = -62.7224792;
    dart_goal_set.trigger_distance_set[GOAL_NOT_SELECT][2] = -67.0299621;
    dart_goal_set.trigger_distance_set[GOAL_NOT_SELECT][3] = -64.0862838;

    dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][0] = -57.6753;
    dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][1] = -60.2224792;
    dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][2] = -60.9299621;
    dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][3] = -61.8862838;

    dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][0] = -77.30;
    dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][1] = -78.00;
    dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][2] = -78.50;
    dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][3] = -80.00;
#endif
    //Send_to_Screen_Data(dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION],dart_goal_set.trigger_distance_set[GOAL_BASE_STATION]);

}
//已完成
static void dart_mode_set()
{
#ifdef GAME_MODE
        if(switch_is_down(rc_ctrl.rc.s[RC_s_L]) && switch_is_down(rc_ctrl.rc.s[RC_s_R]))
        {
            gimbal_dart.mode=DART_RELAX;
        }
        if(gimbal_dart.mode==DART_RELAX || gimbal_dart.mode==DART_CONTROL) {
            if (switch_is_mid(rc_ctrl.rc.s[RC_s_L]) && switch_is_mid(rc_ctrl.rc.s[RC_s_R])) {
                gimbal_dart.mode = DART_BACK;
            }
        }
#if 0
    if (switch_is_up(rc_ctrl.rc.s[RC_s_L]) && switch_is_up(rc_ctrl.rc.s[RC_s_R]))
        {
            gimbal_dart.mode = DART_CONTROL;
        }
#else

#if 1 //通过裁判系统自动发射
    if(gimbal_dart.mode==DART_BACK && dart_back.dart_back_flag && dart_goal_set.dart_goal != GOAL_NOT_SELECT && ((dart_launch_mode == 0x01 && dart_goal_set.launcherable_num <= 1) || (dart_launch_mode == 0x02 && dart_goal_set.launcherable_num > 1)) && gimbal_dart.mode!= DART_CONTROL)
    {
        dart_back.dart_back_flag = false;
        gimbal_dart.mode=DART_READY;
        dart_launch_mode = 0xff;
    }
#else//日常测试使用，不经过裁判系统发射
    if(gimbal_dart.mode==DART_BACK && dart_back.dart_back_flag && dart_goal_set.dart_goal != GOAL_NOT_SELECT )
    {
        dart_back.dart_back_flag = false;
        gimbal_dart.mode=DART_READY;
        dart_launch_mode = 0xff;
    }
#endif

#endif
        if(gimbal_dart.mode==DART_READY&&dart_ready_manage.ready_finish_flag)
        {
            gimbal_dart.mode=DART_SCAN;
            dart_ready_manage.ready_finish_flag = false;
            dart_ready_manage.ready_drive_slow_ok = false;
            dart_ready_manage.ready_drive_start_flag = false;
            dart_ready_manage.ready_load_ok = false;
            dart_ready_manage.ready_set_flag = false;
            dart_ready_manage.ready_process_step = 0;
        }
#if 1
        if(gimbal_dart.mode!=DART_RELAX && gimbal_dart.mode!=DART_BACK)
        {
            if(rc_ctrl.rc.ch[2]>400&&rc_ctrl.rc.ch[3]>400&&rc_ctrl.rc.ch[0]<-400&&rc_ctrl.rc.ch[1]>400)
            {
                gimbal_dart.mode=DART_CONTROL;
                dart_ready_manage.ready_process_step = 0;
            }
        }
#endif
        if(gimbal_dart.mode==DART_SCAN && dart_scan.finish_flag && gimbal_dart.mode!= DART_CONTROL)//&& dart_scan.finish_flag
        {
            if(rc_ctrl.rc.ch[4]<-500 || launch_grant)// || dart_ready_manage.ready_door_opened == true
            {
                gimbal_dart.mode=DART_LAUNCH;
                dart_scan.first_flag = false;
                dart_scan.finish_flag = false;
            }
        }
#elif
        if(gimbal_dart.mode==DART_RELAX || gimbal_dart.mode==DART_CONTROL)
        {
            if(switch_is_mid(rc_ctrl.rc.s[RC_s_L]) && switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
            {
                gimbal_dart.mode=DART_BACK;
                dart_back.dart_back_flag = false;
            }
        }
        if(switch_is_up(rc_ctrl.rc.s[RC_s_L]) && switch_is_up(rc_ctrl.rc.s[RC_s_R]) &&
            (gimbal_dart.mode==DART_BACK && dart_back.dart_back_flag) &&
            rc_ctrl.rc.ch[4]==0)
        {
            gimbal_dart.mode=DART_CONTROL;
        }
        if(gimbal_dart.mode==DART_BACK && rc_ctrl.rc.ch[4]<-500)
        {
            if(switch_is_up(rc_ctrl.rc.s[RC_s_L]))
            {
                launcherable_num=2;
            }
            if(switch_is_down(rc_ctrl.rc.s[RC_s_L]))
            {
                launcherable_num=1;
            }
            if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))
            {
                dart_goal=GOAL_FRONT_STATION;
            }
            if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
            {
                dart_goal=GOAL_BASE_STATION;
            }
            if(launcherable_num>0 && dart_goal>0)
            {
                gimbal_dart.mode=DART_GOAL_SET;
            }
        }
        if(gimbal_dart.mode==DART_GOAL_SET)
        {
            if(rc_ctrl.rc.ch[2]>400&&rc_ctrl.rc.ch[3]>400&&rc_ctrl.rc.ch[0]<-400&&rc_ctrl.rc.ch[1]>400)
            {
                gimbal_dart.mode=DART_READY;
            }
        }
        if(gimbal_dart.mode==DART_READY)
        {
            if(switch_is_up(rc_ctrl.rc.s[RC_s_L])&&dart_ready_manage.ready_finish_flag == true)
            {
                gimbal_dart.mode=DART_TRIGGER;
                dart_ready_manage.ready_finish_flag = false;
            }
        }
        if (gimbal_dart.mode==DART_TRIGGER)
        {
            if(rc_ctrl.rc.ch[4]<-500)
            {
                gimbal_dart.mode=DART_LAUNCH;
            }
        }
#endif
}

static void dart_data_update()
{
#if 1
    if(switch_is_up(rc_ctrl.rc.s[RC_s_R])&&(gimbal_dart.mode!=DART_TRIGGER&&gimbal_dart.mode!=DART_READY&&gimbal_dart.mode!=DART_RELAX)&&!dart_goal_set.dart_goal)
    {
        dart_goal_set.dart_goal = GOAL_FRONT_STATION;
        gimbal_dart.motor_yaw.angle_p.set=dart_goal_set.yaw_angle_set[dart_goal_set.dart_goal];
    }
    if(switch_is_up(rc_ctrl.rc.s[RC_s_L])&&(gimbal_dart.mode!=DART_TRIGGER&&gimbal_dart.mode!=DART_READY&&gimbal_dart.mode!=DART_RELAX)&&!dart_goal_set.dart_goal)
    {
        dart_goal_set.dart_goal = GOAL_BASE_STATION;
        gimbal_dart.motor_yaw.angle_p.set=dart_goal_set.yaw_angle_set[dart_goal_set.dart_goal];
    }
#endif
    gimbal_dart.motor_yaw.angle_p.get=yaw_distance_conversion(gimbal_dart.motor_yaw.motor_measure->total_ecd);
    first_order_filter_cali(&filter_yaw_in,robot_ctrl.yaw);
    launcher_dart.push_motor_r.angle_p.get=drive_distance_conversion(launcher_dart.push_motor_r.motor_measure->total_ecd);
    launcher_dart.push_motor_l.angle_p.get=drive_distance_conversion(launcher_dart.push_motor_l.motor_measure->total_ecd);
    launcher_dart.trigger_motor.angle_p.get = trigger_distance_conversion(launcher_dart.trigger_motor.motor_measure->total_ecd);
    turndish_dart.turn_l_motor.angle_p.get = turn_distance_conversion(turndish_dart.turn_l_motor.motor_measure->total_ecd);
    turndish_dart.turn_r_motor.angle_p.get = turn_distance_conversion(turndish_dart.turn_r_motor.motor_measure->total_ecd);
}

static void Push_l_motor_reset(void)
{
    PUSH_L_MOTOR_RESET();
}

static void Turn_l_motor_reset(void)
{
    TURN_L_MOTOR_RESET();
}

static void Turn_r_motor_reset(void)
{
    TURN_R_MOTOR_RESET();
}

static void Push_r_motor_reset(void)
{
    PUSH_R_MOTOR_RESET();
}

static void Gimbal_motor_reset(void)
{
    GIMBAL_MOTOR_RESET();
}

static void Trigger_motor_reset(void)
{
    TRIGGER_MOTOR_RESET();
}

void motor_yaw_pid_calc()
{
    if (gimbal_dart.motor_yaw.angle_p.set==0xff)
    {
        gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                  gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                  gimbal_dart.motor_yaw.speed_p.set);
    }else
    {
        CLAMP_VALUE(gimbal_dart.motor_yaw.angle_p.set, gimbal_dart.max_yaw_pos, gimbal_dart.min_yaw_pos);

        gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                     gimbal_dart.motor_yaw.angle_p.get,
                                                     gimbal_dart.motor_yaw.angle_p.set);
        gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                      gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                      gimbal_dart.motor_yaw.speed_p.set);
    }
}

void motor_push_pid_calc( )
{
    if (launcher_dart.push_motor_r.angle_p.set==0xff)
    {
        launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                            launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                            launcher_dart.push_motor_r.speed_p.set);
    }else
    {
        if (gimbal_dart.mode == DART_READY && dart_ready_manage.ready_process_step == 4) {
            CLAMP_VALUE(launcher_dart.push_motor_r.speed_p.set,  0, -4000);
        }else
        {
            CLAMP_VALUE(launcher_dart.push_motor_r.angle_p.set,-launcher_dart.min_push_pos,-launcher_dart.max_push_pos);
        }
        launcher_dart.push_motor_r.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                                launcher_dart.push_motor_r.angle_p.get,
                                                                launcher_dart.push_motor_r.angle_p.set);

        launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                            launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                            launcher_dart.push_motor_r.speed_p.set);
    }
    if (launcher_dart.push_motor_l.angle_p.set == 0xff)
    {
        launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                            launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                            launcher_dart.push_motor_l.speed_p.set);
    }else
    {
        if (gimbal_dart.mode == DART_READY && dart_ready_manage.ready_process_step == 4) {
            CLAMP_VALUE(launcher_dart.push_motor_l.speed_p.set,  4000, 0);
        }else
        {
            CLAMP_VALUE(launcher_dart.push_motor_l.angle_p.set,launcher_dart.max_push_pos,launcher_dart.min_push_pos);
        }
        launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                                launcher_dart.push_motor_l.angle_p.get,
                                                                launcher_dart.push_motor_l.angle_p.set);

        launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                            launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                            launcher_dart.push_motor_l.speed_p.set);
    }
}

void motor_trigger_pid_calc( )
{
    if ( launcher_dart.trigger_motor.angle_p.set == 0xff)
    {
        launcher_dart.trigger_motor.give_current = pid_calc(&launcher_dart.trigger_motor.speed_p,
                                                            launcher_dart.trigger_motor.motor_measure->speed_rpm,
                                                            launcher_dart.trigger_motor.speed_p.set);
    }else
    {
        CLAMP_VALUE(launcher_dart.trigger_motor.angle_p.set, launcher_dart.max_trigger_pos, launcher_dart.min_trigger_pos);
        launcher_dart.trigger_motor.speed_p.set = pid_calc(&launcher_dart.trigger_motor.angle_p,
                                                            launcher_dart.trigger_motor.angle_p.get,
                                                            launcher_dart.trigger_motor.angle_p.set);
        launcher_dart.trigger_motor.give_current = pid_calc(&launcher_dart.trigger_motor.speed_p,
                                                            launcher_dart.trigger_motor.motor_measure->speed_rpm,
                                                            launcher_dart.trigger_motor.speed_p.set);
    }
}

void motor_turn_pid_calc()
{
    if (turndish_dart.turn_l_motor.angle_p.set==0xff)
    {
        turndish_dart.turn_l_motor.give_current = pid_calc(&turndish_dart.turn_l_motor.speed_p,
                                                            turndish_dart.turn_l_motor.motor_measure->speed_rpm,
                                                            turndish_dart.turn_l_motor.speed_p.set);
    }else
    {
        CLAMP_VALUE(turndish_dart.turn_l_motor.angle_p.set,turndish_dart.turn_max_pos,turndish_dart.turn_min_pos);
        turndish_dart.turn_l_motor.speed_p.set = pid_calc(&turndish_dart.turn_l_motor.angle_p,
                                                            turndish_dart.turn_l_motor.angle_p.get,
                                                            turndish_dart.turn_l_motor.angle_p.set);

        turndish_dart.turn_l_motor.give_current = pid_calc(&turndish_dart.turn_l_motor.speed_p,
                                                            turndish_dart.turn_l_motor.motor_measure->speed_rpm,
                                                            turndish_dart.turn_l_motor.speed_p.set);
    }
    if (turndish_dart.turn_r_motor.angle_p.set == 0xff)
    {
        turndish_dart.turn_r_motor.give_current = pid_calc(&turndish_dart.turn_r_motor.speed_p,
                                                            turndish_dart.turn_r_motor.motor_measure->speed_rpm,
                                                            turndish_dart.turn_r_motor.speed_p.set);
    }else
    {
        CLAMP_VALUE(turndish_dart.turn_r_motor.angle_p.set,turndish_dart.turn_max_pos,turndish_dart.turn_min_pos);
        turndish_dart.turn_r_motor.speed_p.set = pid_calc(&turndish_dart.turn_r_motor.angle_p,
                                                            turndish_dart.turn_r_motor.angle_p.get,
                                                            turndish_dart.turn_r_motor.angle_p.set);

        turndish_dart.turn_r_motor.give_current = pid_calc(&turndish_dart.turn_r_motor.speed_p,
                                                            turndish_dart.turn_r_motor.motor_measure->speed_rpm,
                                                            turndish_dart.turn_r_motor.speed_p.set);
    }
}

static void dart_pos_reset()
{
    gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.reset_yaw_pos;
    launcher_dart.push_motor_l.angle_p.set = launcher_dart.reset_push_pos;
    launcher_dart.push_motor_r.angle_p.set = -launcher_dart.reset_push_pos;
    launcher_dart.trigger_motor.angle_p.set = launcher_dart.reset_trigger_pos;
}

static void dart_motor_loop_cal()
{
    launcher_dart.trigger_pid_calc();
    gimbal_dart.pid_calc();
    launcher_dart.push_pid_calc();
    turndish_dart.turn_pid_calc();
}

float DMkp = 0.45;
float DMkd = 0.05;


static void dm6006_turn_ctrl_handle(void)
{
    float pos_cmd = turndish_dart.turn_angle_set;

    if (pos_cmd > 6.28f) pos_cmd = 6.28f;
    if (pos_cmd < 0.0f) pos_cmd = 0.0f;

    DM_MIT_Ctrl_Motor(CAN_1, DM6006_TURN_MOTOR_ID, pos_cmd, 0, DMkp, DMkd, 0.0);
}

static float yaw_slew_step(float current, float target, float max_step)
{
    /* d 表示当前周期还差多少才能追上目标值。 */
    float d = target - current;
    /* 如果正向变化太大，就最多只走 max_step。 */
    if (d > max_step) d = max_step;
    /* 如果反向变化太大，就最多只走 -max_step。 */
    if (d < -max_step) d = -max_step;
    /* 返回本周期更新后的平滑命令值。 */
    return current + d;
}