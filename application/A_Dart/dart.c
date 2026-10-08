/*
 * STM32F427 飞镖硬件适配器
 * =======================
 * 本文件不包含任何发射或换弹业务状态机，只负责以下硬件转换工作：
 *   1. 把原始编码器反馈换算为物理位置；
 *   2. 把抽象轴命令转换为串级位置环和速度环；
 *   3. 向现有 CAN、PWM 驱动输出最终电流或脉宽；
 *   4. 状态机确认限位后更新编码器零点；
 *   5. 为尚未迁移的通信代码提供少量旧全局观察值。
 *
 * 已拆除的 DM6006、双升降轴和舵机门均由 DART_ENABLE_CAROUSEL_LOADER 保护。默认
 * 构建强制升降电流为零，不使能 DM 电机，并且不会编译舵机串口命令。
 */

#include "dart.h"

#include <string.h>

#include "dart_build_config.h"
#include "dart_hw_legacy.h"
#include "dart_runtime.h"
#include "main.h"
#include "tim.h"

#if DART_ENABLE_CAROUSEL_LOADER
#include "usart.h"
#include "zp10s_servo.h"
#endif

/* 当前机械结构沿用的编码器和传动换算常量。 */
#define ENCODER_COUNTS               8192.0f
#define REDUCTION_RATIO                19.0f
#define YAW_TRAVEL_PER_REVOLUTION_MM    5.0f
#define TRIGGER_TRAVEL_PER_REV_MM       5.0f
#define PUSH_REDUCTION_RATIO            27.0f
#define PUSH_PULLEY_RADIUS_MM           19.32f
#define DART_PI                          3.14159265358979323846f

/* 核心轴软件位置限幅，防止异常目标越过机械行程。 */
#define YAW_POSITION_MIN               15.0f
#define YAW_POSITION_MAX              142.0f
#define PUSH_LEFT_POSITION_MIN          5.0f
#define PUSH_LEFT_POSITION_MAX        675.0f
#define TRIGGER_POSITION_MIN          -95.0f
#define TRIGGER_POSITION_MAX          -15.0f

/* 当前 F427 机构的串级 PID 标定值。 */
#define YAW_ANGLE_KP                  900.0f
#define YAW_ANGLE_MAX               10000.0f
#define YAW_ANGLE_IMAX               3000.0f
#define YAW_SPEED_KP                   75.0f
#define YAW_SPEED_MAX               25000.0f

#define PUSH_ANGLE_KP                 100.0f
#define PUSH_ANGLE_KI                   0.001f
#define PUSH_ANGLE_KD                   1.0f
#define PUSH_ANGLE_MAX               10000.0f
#define PUSH_ANGLE_IMAX               3000.0f
#define PUSH_SPEED_KP                   35.0f
#define PUSH_SPEED_MAX               16000.0f

#define TRIGGER_ANGLE_KP             9900.0f
#define TRIGGER_ANGLE_MAX           10000.0f
#define TRIGGER_ANGLE_IMAX           8000.0f
#define TRIGGER_SPEED_KP               20.0f
#define TRIGGER_SPEED_MAX           16000.0f
#define TRIGGER_SPEED_IMAX           8000.0f

#if DART_ENABLE_CAROUSEL_LOADER
#define LIFT_POSITION_MIN               2.0f
#define LIFT_POSITION_MAX             200.0f
#define LIFT_ANGLE_KP                3000.0f
#define LIFT_ANGLE_MAX              10000.0f
#define LIFT_ANGLE_IMAX              3000.0f
#define LIFT_SPEED_KP                  30.0f
#define LIFT_SPEED_MAX              16000.0f
#define GATE_OPEN_PULSE_US           2000U
#define GATE_CLOSE_PULSE_US          1500U
#define GATE_MOVE_TIME_MS             500U
#endif

#define LAUNCHER_SAFE_COMPARE        1200U
#define LAUNCHER_OPEN_COMPARE         500U

dart_launcher_hardware_t launcher_dart = {
    .push_motor_right.motor_measure = &motor_3508[0],
    .push_motor_left.motor_measure = &motor_3508[1],
    .trigger_motor.motor_measure = &motor_3508[2],
};

dart_gimbal_hardware_t gimbal_dart = {
    .mode = DART_RELAX,
    .yaw_motor.motor_measure = &motor_6020[0],
};

dart_carousel_hardware_t turndish_dart = {
    .lift_motor_left.motor_measure = &motor_3508[3],
    .lift_motor_right.motor_measure = &motor_3508[4],
};

dart_legacy_status_t dart_goal_set;

static dart_axis_mode_t active_mode[DART_AXIS_COUNT];

#if DART_ENABLE_CAROUSEL_LOADER
static bool carousel_enabled;
static ZP10S_HandleTypeDef gate_bus;
static bool gate_bus_initialized;
#endif

/* 把目标限制在机械允许区间内，所有位置闭环在计算电流前都经过该保护。 */
/**
 * @brief 把浮点目标限制在闭区间内。
 * @return 小于下限时返回下限，大于上限时返回上限，否则返回原值。
 */
static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/** @brief 把水平轴连续编码器计数换算为业务位置。 */
static float yaw_position_from_encoder(int32_t encoder)
{
    /* 水平轴丝杆每转移动五毫米，当前编码器标定沿用历史每圈八千一百八十九计数。 */
    return ((float)encoder / 8189.0f) * YAW_TRAVEL_PER_REVOLUTION_MM;
}

/** @brief 按编码器每圈计数、减速比和丝杆行程换算扳机位置。 */
static float trigger_position_from_encoder(int32_t encoder)
{
    /* 扳机位置等于电机累计圈数除以减速比后，再乘丝杆单圈行程。 */
    return ((float)encoder / ENCODER_COUNTS / REDUCTION_RATIO) *
           TRIGGER_TRAVEL_PER_REV_MM;
}

/** @brief 按减速比和带轮周长换算推板直线位置。 */
static float push_position_from_encoder(int32_t encoder)
{
    /* 推板由同步带轮驱动，输出圈数乘带轮周长得到直线位移。 */
    return ((float)encoder / ENCODER_COUNTS / PUSH_REDUCTION_RATIO) *
           PUSH_PULLEY_RADIUS_MM * 2.0f * DART_PI;
}

/** @brief 把原换弹机构升降电机累计计数换算为升降高度。 */
static float lift_position_from_encoder(int32_t encoder)
{
    /* 当前可选升降机构使用十九比一减速和五毫米单圈行程。 */
    return ((float)encoder / ENCODER_COUNTS / REDUCTION_RATIO) * 5.0f;
}

/**
 * @brief 使用统一参数顺序初始化一个历史 PID 对象。
 *
 * 该薄封装把本文件的浮点标定值传给旧 PID 模块，避免各轴初始化代码重复底层调用约定。
 *
 * @param pid 待初始化的 PID 对象。
 * @param max_out 输出绝对值上限。
 * @param max_iout 积分项绝对值上限。
 * @param kp 比例增益。
 * @param ki 积分增益。
 * @param kd 微分增益。
 */
static void initialize_pid(pid_t *pid, float max_out, float max_iout,
                           float kp, float ki, float kd)
{
    pid_init(pid, max_out, max_iout, kp, ki, kd);
}

/**
 * @brief 使用当前机构标定值初始化全部串级控制器。
 *
 * 左右成对轴使用相同增益但独立保存误差和积分历史；拆机配置不初始化升降控制器。
 */
static void initialize_control_loops(void)
{
    /*
     * 每个物理轴使用外层位置环和内层速度环。左右推板及左右升降使用相同参数，但仍保留
     * 独立控制器，避免一侧误差或积分历史串入另一侧。
     */
    initialize_pid(&launcher_dart.push_motor_right.angle_p,
                   PUSH_ANGLE_MAX, PUSH_ANGLE_IMAX,
                   PUSH_ANGLE_KP, PUSH_ANGLE_KI, PUSH_ANGLE_KD);
    initialize_pid(&launcher_dart.push_motor_right.speed_p,
                   PUSH_SPEED_MAX, PUSH_SPEED_MAX, PUSH_SPEED_KP, 0.0f, 0.0f);
    initialize_pid(&launcher_dart.push_motor_left.angle_p,
                   PUSH_ANGLE_MAX, PUSH_ANGLE_IMAX,
                   PUSH_ANGLE_KP, PUSH_ANGLE_KI, PUSH_ANGLE_KD);
    initialize_pid(&launcher_dart.push_motor_left.speed_p,
                   PUSH_SPEED_MAX, PUSH_SPEED_MAX, PUSH_SPEED_KP, 0.0f, 0.0f);
    initialize_pid(&launcher_dart.trigger_motor.angle_p,
                   TRIGGER_ANGLE_MAX, TRIGGER_ANGLE_IMAX,
                   TRIGGER_ANGLE_KP, 0.0f, 0.0f);
    initialize_pid(&launcher_dart.trigger_motor.speed_p,
                   TRIGGER_SPEED_MAX, TRIGGER_SPEED_IMAX,
                   TRIGGER_SPEED_KP, 0.0f, 0.0f);
    initialize_pid(&gimbal_dart.yaw_motor.angle_p,
                   YAW_ANGLE_MAX, YAW_ANGLE_IMAX, YAW_ANGLE_KP, 0.0f, 0.0f);
    initialize_pid(&gimbal_dart.yaw_motor.speed_p,
                   YAW_SPEED_MAX, YAW_SPEED_MAX, YAW_SPEED_KP, 0.0f, 0.0f);

#if DART_ENABLE_CAROUSEL_LOADER
    initialize_pid(&turndish_dart.lift_motor_left.angle_p,
                   LIFT_ANGLE_MAX, LIFT_ANGLE_IMAX, LIFT_ANGLE_KP, 0.0f, 0.0f);
    initialize_pid(&turndish_dart.lift_motor_left.speed_p,
                   LIFT_SPEED_MAX, LIFT_SPEED_MAX, LIFT_SPEED_KP, 0.0f, 0.0f);
    initialize_pid(&turndish_dart.lift_motor_right.angle_p,
                   LIFT_ANGLE_MAX, LIFT_ANGLE_IMAX, LIFT_ANGLE_KP, 0.0f, 0.0f);
    initialize_pid(&turndish_dart.lift_motor_right.speed_p,
                   LIFT_SPEED_MAX, LIFT_SPEED_MAX, LIFT_SPEED_KP, 0.0f, 0.0f);
#endif
}

/**
 * @brief 把平台无关轴命令转换为旧 PID 对象的设定值。
 *
 * 位置模式写入外环目标；速度模式使用旧控制器约定的特殊标志跳过位置环，并直接写入速度
 * 目标。禁用模式不改写设定值，实际零输出由串级计算函数保证。
 *
 * @param mode 当前轴控制模式。
 * @param target 当前模式对应的位置或速度目标。
 * @param angle_pid 该轴位置外环对象。
 * @param speed_pid 该轴速度内环对象。
 */
static void apply_axis_setpoint(dart_axis_mode_t mode, float target,
                                pid_t *angle_pid, pid_t *speed_pid)
{
    if (mode == DART_AXIS_MODE_POSITION) {
        angle_pid->set = target;
    } else if (mode == DART_AXIS_MODE_SPEED) {
        /* 0xff 只在适配层内部作为跳过位置环、直接进入速度环的历史标志。 */
        angle_pid->set = 0xff;
        speed_pid->set = target;
    }
}

/**
 * @brief 计算一个 3508 轴的位置外环和速度内环。
 * @return 限幅后的有符号电流；禁用模式固定返回零。
 */
static int16_t run_cascade(motor_3508_t *motor, dart_axis_mode_t mode,
                           float position, float minimum, float maximum)
{
    /* 禁用时直接返回零电流；位置模式先算速度目标，速度模式则跳过外环。 */
    if (mode == DART_AXIS_MODE_DISABLED) return 0;
    if (mode == DART_AXIS_MODE_POSITION) {
        motor->angle_p.set = clamp_float(motor->angle_p.set, minimum, maximum);
        motor->speed_p.set = pid_calc(&motor->angle_p, position, motor->angle_p.set);
    }
    return (int16_t)pid_calc(&motor->speed_p,
                             motor->motor_measure->speed_rpm,
                             motor->speed_p.set);
}

/**
 * @brief 计算水平轴的位置外环和速度内环。
 * @return 限幅后的水平轴有符号电流；禁用模式固定返回零。
 */
static int16_t run_yaw_cascade(dart_axis_mode_t mode, float position)
{
    /* 水平轴控制流程与 3508 相同，但使用独立机械限幅和 6020 反馈对象。 */
    motor_6020_t *motor = &gimbal_dart.yaw_motor;
    if (mode == DART_AXIS_MODE_DISABLED) return 0;
    if (mode == DART_AXIS_MODE_POSITION) {
        motor->angle_p.set = clamp_float(motor->angle_p.set,
                                         YAW_POSITION_MIN, YAW_POSITION_MAX);
        motor->speed_p.set = pid_calc(&motor->angle_p, position, motor->angle_p.set);
    }
    return (int16_t)pid_calc(&motor->speed_p,
                             motor->motor_measure->speed_rpm,
                             motor->speed_p.set);
}

/**
 * @brief 把发射机构开关意图转换为定时器比较值。
 * @param open true 表示进入发射位置，false 表示回到安全位置。
 */
static void launcher_set_open(bool open)
{
    /* 定时器比较值对应发射机构安全位置与动作位置，状态机只传递布尔意图。 */
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,
                         open ? LAUNCHER_OPEN_COMPARE : LAUNCHER_SAFE_COMPARE);
}

#if DART_ENABLE_CAROUSEL_LOADER
/**
 * @brief 按需初始化原换弹机构的舵机总线。
 *
 * 函数保持幂等，首次调用时绑定串口并设置通信超时，后续调用直接返回。该代码仅在明确
 * 启用原换弹机构时参与编译。
 */
static void gate_bus_init(void)
{
    if (gate_bus_initialized) return;
    ZP10S_Init(&gate_bus, &huart8);
    ZP10S_SetTimeouts(&gate_bus, 20U, 30U);
    gate_bus_initialized = true;
}

/**
 * @brief 向 DM6006 转盘发送一周期位置控制命令。
 *
 * 目标角度会先限制在机构允许范围内，再按达妙电机协议发送。该函数只服务原换弹机构，
 * 拆机配置不会编译也不会访问对应 CAN 标识符。
 */
static void carousel_control(void)
{
    const float target = clamp_float(turndish_dart.carousel_target, 0.0f, 6.28f);
    DM_MIT_Ctrl_Motor(CAN_1, DM6006_TURN_MOTOR_ID,
                      target, 0.0f, 0.45f, 0.05f, 0.0f);
}
#endif

/**
 * @brief 提供任务注册表使用的 DartTask 入口。
 *
 * 该函数不重复实现任务循环，只把控制权交给 `dart_runtime_task()`，确保旧工程入口名称
 * 与新运行时框架兼容。
 *
 * @param argument FreeRTOS 任务参数，原样传递给运行时。
 */
void dart_task(void const *argument)
{
    /* 保留旧任务符号作为静态任务表入口，具体任务生命周期由运行时实现。 */
    dart_runtime_task(argument);
}

/**
 * @brief 初始化控制器、发射脉宽和旧兼容状态。
 * @return true 表示初始化完成；当前实现没有异步失败分支。
 */
bool dart_hw_legacy_init(void)
{
    /* 上电默认关闭全部轴模式和发射机构，然后初始化控制器历史。 */
    memset(active_mode, 0, sizeof(active_mode));
    initialize_control_loops();
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    launcher_set_open(false);
    gimbal_dart.mode = DART_BACK;
#if DART_ENABLE_CAROUSEL_LOADER
    carousel_enabled = false;
#endif
    return true;
}

/**
 * @brief 返回允许自然回绕的单调毫秒时钟。
 *
 * @return 当前 HAL 系统节拍，状态机使用无符号时间差处理回绕。
 */
uint32_t dart_hw_legacy_now_ms(void)
{
    return HAL_GetTick();
}

/**
 * @brief 生成一份位置、速度、限位和在线状态一致的反馈快照。
 * @param feedback 输出快照；空指针会被安全忽略。
 */
void dart_hw_legacy_sample_feedback(dart_feedback_t *feedback)
{
    /*
     * 先把同一时刻的原始累计编码器转换为业务单位，再统一采集限位和在线状态。调用者
     * 只使用这份快照，不在一个状态机周期内重复读取可能被中断更新的全局反馈。
     */
    if (feedback == NULL) return;
    const uint32_t now_ms = HAL_GetTick();

    feedback->position[DART_AXIS_TRIGGER] =
        trigger_position_from_encoder(launcher_dart.trigger_motor.motor_measure->total_ecd);
    feedback->position[DART_AXIS_PUSH_LEFT] =
        push_position_from_encoder(launcher_dart.push_motor_left.motor_measure->total_ecd);
    feedback->position[DART_AXIS_PUSH_RIGHT] =
        push_position_from_encoder(launcher_dart.push_motor_right.motor_measure->total_ecd);
    feedback->position[DART_AXIS_LIFT_LEFT] =
        lift_position_from_encoder(turndish_dart.lift_motor_left.motor_measure->total_ecd);
    feedback->position[DART_AXIS_LIFT_RIGHT] =
        lift_position_from_encoder(turndish_dart.lift_motor_right.motor_measure->total_ecd);
    feedback->position[DART_AXIS_YAW] =
        yaw_position_from_encoder(gimbal_dart.yaw_motor.motor_measure->total_ecd);

    feedback->speed[DART_AXIS_TRIGGER] = launcher_dart.trigger_motor.motor_measure->speed_rpm;
    feedback->speed[DART_AXIS_PUSH_LEFT] = launcher_dart.push_motor_left.motor_measure->speed_rpm;
    feedback->speed[DART_AXIS_PUSH_RIGHT] = launcher_dart.push_motor_right.motor_measure->speed_rpm;
    feedback->speed[DART_AXIS_LIFT_LEFT] = turndish_dart.lift_motor_left.motor_measure->speed_rpm;
    feedback->speed[DART_AXIS_LIFT_RIGHT] = turndish_dart.lift_motor_right.motor_measure->speed_rpm;
    feedback->speed[DART_AXIS_YAW] = gimbal_dart.yaw_motor.motor_measure->speed_rpm;

    /* 在这里统一 GPIO 有效电平，业务层只接收已经归一化的布尔限位值。 */
    feedback->limit[DART_AXIS_TRIGGER] =
        HAL_GPIO_ReadPin(TRIGGER_INIT_GPIO_Port, TRIGGER_INIT_Pin) == GPIO_PIN_SET;
    feedback->limit[DART_AXIS_PUSH_LEFT] =
        HAL_GPIO_ReadPin(PUSH_INIT_L_GPIO_Port, PUSH_INIT_L_Pin) == GPIO_PIN_SET;
    feedback->limit[DART_AXIS_PUSH_RIGHT] =
        HAL_GPIO_ReadPin(PUSH_INIT_R_GPIO_Port, PUSH_INIT_R_Pin) == GPIO_PIN_SET;
    feedback->limit[DART_AXIS_LIFT_LEFT] =
        HAL_GPIO_ReadPin(TURN_L_DISH_GPIO_Port, TURN_L_DISH_Pin) == GPIO_PIN_SET;
    feedback->limit[DART_AXIS_LIFT_RIGHT] =
        HAL_GPIO_ReadPin(TURN_R_DISH_GPIO_Port, TURN_R_DISH_Pin) == GPIO_PIN_SET;
    feedback->limit[DART_AXIS_YAW] =
        HAL_GPIO_ReadPin(YAW_Init_GPIO_Port, YAW_Init_Pin) == GPIO_PIN_SET;

    feedback->online[DART_AXIS_TRIGGER] = (uint32_t)(now_ms - motor_3508_last_update[2]) <= 200U;
    feedback->online[DART_AXIS_PUSH_LEFT] = (uint32_t)(now_ms - motor_3508_last_update[1]) <= 200U;
    feedback->online[DART_AXIS_PUSH_RIGHT] = (uint32_t)(now_ms - motor_3508_last_update[0]) <= 200U;
    feedback->online[DART_AXIS_LIFT_LEFT] = (uint32_t)(now_ms - motor_3508_last_update[3]) <= 200U;
    feedback->online[DART_AXIS_LIFT_RIGHT] = (uint32_t)(now_ms - motor_3508_last_update[4]) <= 200U;
    feedback->online[DART_AXIS_YAW] = (uint32_t)(now_ms - motor_6020_last_update[0]) <= 200U;
    feedback->loader_turn_position = turndish_dart.carousel_motor.position;
    feedback->loader_turn_online = (uint32_t)(now_ms - dm6006_last_update) <= 200U;
    feedback->timestamp_ms = now_ms;
}

/**
 * @brief 把抽象执行器命令锁存到旧电机控制对象。
 *
 * 本函数不发送电流。拆机配置会无条件禁用升降轴，并忽略转盘和舵机门相关目标。
 *
 * @param command 本周期执行器命令；空指针会被安全忽略。
 */
void dart_hw_legacy_apply(const dart_actuator_command_t *command)
{
    /*
     * 此函数只锁存状态机本周期目标，不计算或发送电流。这样 service 可以基于全部轴均已
     * 更新的一组目标，一次性产生一致的总线输出。
     */
    if (command == NULL) return;
    memcpy(active_mode, command->mode, sizeof(active_mode));

    apply_axis_setpoint(command->mode[DART_AXIS_TRIGGER], command->target[DART_AXIS_TRIGGER],
                        &launcher_dart.trigger_motor.angle_p,
                        &launcher_dart.trigger_motor.speed_p);
    apply_axis_setpoint(command->mode[DART_AXIS_PUSH_LEFT], command->target[DART_AXIS_PUSH_LEFT],
                        &launcher_dart.push_motor_left.angle_p,
                        &launcher_dart.push_motor_left.speed_p);
    apply_axis_setpoint(command->mode[DART_AXIS_PUSH_RIGHT], command->target[DART_AXIS_PUSH_RIGHT],
                        &launcher_dart.push_motor_right.angle_p,
                        &launcher_dart.push_motor_right.speed_p);
    apply_axis_setpoint(command->mode[DART_AXIS_YAW], command->target[DART_AXIS_YAW],
                        &gimbal_dart.yaw_motor.angle_p, &gimbal_dart.yaw_motor.speed_p);

#if DART_ENABLE_CAROUSEL_LOADER
    apply_axis_setpoint(command->mode[DART_AXIS_LIFT_LEFT], command->target[DART_AXIS_LIFT_LEFT],
                        &turndish_dart.lift_motor_left.angle_p,
                        &turndish_dart.lift_motor_left.speed_p);
    apply_axis_setpoint(command->mode[DART_AXIS_LIFT_RIGHT], command->target[DART_AXIS_LIFT_RIGHT],
                        &turndish_dart.lift_motor_right.angle_p,
                        &turndish_dart.lift_motor_right.speed_p);
    if (command->loader_turn_enabled != carousel_enabled) {
        DM_Send_CMD(CAN_1, DM6006_TURN_MOTOR_ID,
                    command->loader_turn_enabled ? DM_Enable_CMD : DM_Disable_CMD);
    }
    carousel_enabled = command->loader_turn_enabled;
    turndish_dart.carousel_target = command->loader_turn_target;
#else
    /* 拆机版本忽略全部可选机构目标，并强制关闭两个升降轴的软件模式。 */
    active_mode[DART_AXIS_LIFT_LEFT] = DART_AXIS_MODE_DISABLED;
    active_mode[DART_AXIS_LIFT_RIGHT] = DART_AXIS_MODE_DISABLED;
#endif
    launcher_set_open(command->launcher_open);
}

/**
 * @brief 基于已锁存目标计算全部控制器并发送本周期硬件输出。
 *
 * 两组大疆电机控制帧在所有轴计算结束后发送，保证同一帧中的电流来自同一个控制周期。
 */
void dart_hw_legacy_service(void)
{
    /* 基于同一周期的反馈和目标完成计算，再发送一组一致的 CAN 电流帧。 */
    launcher_dart.trigger_motor.give_current =
        run_cascade(&launcher_dart.trigger_motor, active_mode[DART_AXIS_TRIGGER],
                    trigger_position_from_encoder(launcher_dart.trigger_motor.motor_measure->total_ecd),
                    TRIGGER_POSITION_MIN, TRIGGER_POSITION_MAX);
    launcher_dart.push_motor_left.give_current =
        run_cascade(&launcher_dart.push_motor_left, active_mode[DART_AXIS_PUSH_LEFT],
                    push_position_from_encoder(launcher_dart.push_motor_left.motor_measure->total_ecd),
                    PUSH_LEFT_POSITION_MIN, PUSH_LEFT_POSITION_MAX);
    launcher_dart.push_motor_right.give_current =
        run_cascade(&launcher_dart.push_motor_right, active_mode[DART_AXIS_PUSH_RIGHT],
                    push_position_from_encoder(launcher_dart.push_motor_right.motor_measure->total_ecd),
                    -PUSH_LEFT_POSITION_MAX, -PUSH_LEFT_POSITION_MIN);
    gimbal_dart.yaw_motor.give_current =
        run_yaw_cascade(active_mode[DART_AXIS_YAW],
                        yaw_position_from_encoder(gimbal_dart.yaw_motor.motor_measure->total_ecd));

#if DART_ENABLE_CAROUSEL_LOADER
    turndish_dart.lift_motor_left.give_current =
        run_cascade(&turndish_dart.lift_motor_left, active_mode[DART_AXIS_LIFT_LEFT],
                    lift_position_from_encoder(turndish_dart.lift_motor_left.motor_measure->total_ecd),
                    LIFT_POSITION_MIN, LIFT_POSITION_MAX);
    turndish_dart.lift_motor_right.give_current =
        run_cascade(&turndish_dart.lift_motor_right, active_mode[DART_AXIS_LIFT_RIGHT],
                    lift_position_from_encoder(turndish_dart.lift_motor_right.motor_measure->total_ecd),
                    LIFT_POSITION_MIN, LIFT_POSITION_MAX);
    if (carousel_enabled) carousel_control();
#else
    turndish_dart.lift_motor_left.give_current = 0;
    turndish_dart.lift_motor_right.give_current = 0;
#endif

    CAN_cmd_motor(CAN_2, CAN_MOTOR_0x200_ID,
                  turndish_dart.lift_motor_right.give_current,
                  turndish_dart.lift_motor_left.give_current, 0, 0);
    CAN_cmd_motor(CAN_2, CAN_MOTOR_0x1FF_ID,
                  gimbal_dart.yaw_motor.give_current,
                  launcher_dart.push_motor_right.give_current,
                  launcher_dart.trigger_motor.give_current,
                  launcher_dart.push_motor_left.give_current);
}

/**
 * @brief 清除单个 PID 对象的动态运行历史。
 *
 * 比例、积分、微分增益以及输出限幅保持不变，只清除误差、积分累计和各项输出，避免复位
 * 后沿用故障或上一发次产生的控制历史。
 *
 * @param pid 待清理的 PID 对象。
 */
static void clear_pid_runtime(pid_t *pid)
{
    /* 保留增益和限幅等标定值，只清除误差、积分和输出历史。 */
    pid->err[LAST] = 0.0f;
    pid->err[NOW] = 0.0f;
    pid->sum_err = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
}

/**
 * @brief 把电机当前原始累计编码位置登记为新的机械零点。
 *
 * 偏移值按圈数和当前单圈编码值重新构造，不能直接使用已经扣除旧偏移的 `total_ecd`。
 * 这样连续执行第二轮及后续回零时仍能得到正确零点。
 *
 * @param measure 待更新的电机测量对象。
 */
static void zero_motor_measure(motor_measure_t *measure)
{
    /*
     * total_ecd 已经减去了上一次 offset，计算式为：
     *   total_ecd = round_cnt × 8192 + ecd - offset_ecd
     * 如果再次把 total_ecd 写回 offset，只有第一次回零正确。这里保存原始累计编码值，
     * 确保第二轮及以后每次复位都能真正从零开始。
     */
    measure->offset_ecd = measure->round_cnt * (int32_t)ENCODER_COUNTS +
                          (int32_t)measure->ecd;
    measure->total_ecd = 0;
}

/**
 * @brief 清除全部已启用控制器的动态历史。
 *
 * 仅清除误差、积分和输出，不修改比例积分微分增益、限幅、编码器零点或持久化参数。
 */
void dart_hw_legacy_reset_control_state(void)
{
    /* 业务恢复完成后清空所有控制器动态历史，参数增益和机械零点保持不变。 */
    pid_t *const core_loops[] = {
        &launcher_dart.trigger_motor.angle_p, &launcher_dart.trigger_motor.speed_p,
        &launcher_dart.push_motor_left.angle_p, &launcher_dart.push_motor_left.speed_p,
        &launcher_dart.push_motor_right.angle_p, &launcher_dart.push_motor_right.speed_p,
        &gimbal_dart.yaw_motor.angle_p, &gimbal_dart.yaw_motor.speed_p,
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(core_loops) / sizeof(core_loops[0])); ++i) {
        clear_pid_runtime(core_loops[i]);
    }
#if DART_ENABLE_CAROUSEL_LOADER
    clear_pid_runtime(&turndish_dart.lift_motor_left.angle_p);
    clear_pid_runtime(&turndish_dart.lift_motor_left.speed_p);
    clear_pid_runtime(&turndish_dart.lift_motor_right.angle_p);
    clear_pid_runtime(&turndish_dart.lift_motor_right.speed_p);
#endif
}

/**
 * @brief 将指定轴当前原始累计编码器位置登记为机械零点。
 * @param axis 已确认命中机械限位的轴。
 */
void dart_hw_legacy_zero_axis(dart_axis_t axis)
{
    /* 只有负责该轴的状态机确认物理限位后，才允许调用本函数。 */
    switch (axis) {
        case DART_AXIS_TRIGGER:
            zero_motor_measure(launcher_dart.trigger_motor.motor_measure);
            break;
        case DART_AXIS_PUSH_LEFT:
            zero_motor_measure(launcher_dart.push_motor_left.motor_measure);
            break;
        case DART_AXIS_PUSH_RIGHT:
            zero_motor_measure(launcher_dart.push_motor_right.motor_measure);
            break;
        case DART_AXIS_YAW:
            zero_motor_measure(gimbal_dart.yaw_motor.motor_measure);
            break;
#if DART_ENABLE_CAROUSEL_LOADER
        case DART_AXIS_LIFT_LEFT:
            zero_motor_measure(turndish_dart.lift_motor_left.motor_measure);
            break;
        case DART_AXIS_LIFT_RIGHT:
            zero_motor_measure(turndish_dart.lift_motor_right.motor_measure);
            break;
#else
        case DART_AXIS_LIFT_LEFT:
        case DART_AXIS_LIFT_RIGHT:
            break;
#endif
        default:
            break;
    }
}

/**
 * @brief 设置原换弹机构指定舵机门的目标状态。
 *
 * 安装版会按需初始化舵机总线并发送目标脉宽；拆机版在预处理阶段保留为空操作，不访问
 * 已拆除硬件。零号门被视为无效编号并安全忽略。
 *
 * @param index 舵机门编号。
 * @param open true 表示打开，false 表示关闭。
 */
void dart_hw_legacy_gate_set(uint8_t index, bool open)
{
    /* 拆机固件编译为空操作；安装版才初始化总线并驱动指定舵机门。 */
#if DART_ENABLE_CAROUSEL_LOADER
    if (index == 0U) return;
    gate_bus_init();
    (void)ZP10S_Move(&gate_bus, index,
                     open ? GATE_OPEN_PULSE_US : GATE_CLOSE_PULSE_US,
                     GATE_MOVE_TIME_MS);
#else
    (void)index;
    (void)open;
#endif
}

/**
 * @brief 请求关闭原换弹机构的全部舵机门。
 *
 * 安装版依次发送四扇门的关闭位置；拆机版为空操作。该函数不等待舵机实际运动完成，等待
 * 时间由换弹策略对应步骤管理。
 */
void dart_hw_legacy_gate_close_all(void)
{
    /* 完成换弹或恢复时依次关闭四扇门，防止某扇门保留上一轮状态。 */
#if DART_ENABLE_CAROUSEL_LOADER
    gate_bus_init();
    for (uint8_t index = 1U; index <= 4U; ++index) {
        (void)ZP10S_Move(&gate_bus, index, GATE_CLOSE_PULSE_US, GATE_MOVE_TIME_MS);
    }
#endif
}
