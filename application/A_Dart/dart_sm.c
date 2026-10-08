#include "dart_sm.h"

#include <math.h>
#include <string.h>

/*
 * Dart 发射业务状态机
 * ==================
 *
 * 本模块负责四发循环、瞄准稳定判定、裁判发射许可、核心轴回零、故障锁定和受控恢复。
 * 它刻意不知道 CAN ID、DM6006、升降机构尺寸和舵机门协议；这些细节全部由当前选择的
 * dart_reload_strategy_t 管理，换弹机构拆除时也可以使用无硬件策略。
 *
 * 所有函数都非阻塞。DartTask 每 1 ms 调用一次 dart_sm_step()，每个子步骤只做四件事：
 *   1. 写入本周期执行器目标；
 *   2. 检查真实位置、限位和在线反馈；
 *   3. 满足完成条件时推进一个状态/子状态；
 *   4. 超时或反馈异常时停止相关输出并进入恢复或锁定故障。
 *
 * 重要约束：shot_index 为零基索引，只能在 FIRE 保持时间完整结束后递增。第四发完成
 * 必须经过 CYCLE_COMPLETE -> RECOVERING，绝不能直接把计数清零回到 STANDBY。
 */
#define POSITION_PUSH_EPSILON     7.0f
#define POSITION_TRIGGER_EPSILON  0.5f
#define POSITION_YAW_EPSILON      1.0f
#define VISION_STALE_MS           200U
#define REFEREE_STALE_MS          200U
#define INPUT_STARTUP_GRACE_MS    1000U
#define AIM_STABLE_MS             500U
#define AIM_ERROR_EPSILON         0.05f
#define PAIR_LIMIT_SKEW_MS        250U
#define PUSH_HOME_SPEED          -500.0f
#define TRIGGER_HOME_SPEED       4000.0f
#define TRIGGER_RESET_POSITION    -55.0f
#define YAW_HOME_SPEED          -1000.0f
#define YAW_RESET_POSITION         73.944f

enum {
    /* 上电回零顺序：扳机 -> 推板对 -> 可选换弹机构 -> Yaw。 */
    HOME_TRIGGER_SEEK = 0,
    HOME_TRIGGER_SETTLE,
    HOME_PUSH_SEEK,
    HOME_PUSH_SETTLE,
    HOME_OPTIONAL_LOADER,
    HOME_YAW_SEEK,
    HOME_YAW_SETTLE,
    HOME_FINISHED
};

enum {
    /*
     * 业务恢复顺序：先让换弹策略完成安全退让，再回零扳机、推板对、可选机构和 Yaw。
     * 对拆机策略而言两个可选机构步骤立即完成且不输出任何动作。
     */
    RECOVER_RELOAD_SAFE = 0,
    RECOVER_TRIGGER_SEEK,
    RECOVER_TRIGGER_SETTLE,
    RECOVER_PUSH_SEEK,
    RECOVER_PUSH_SETTLE,
    RECOVER_OPTIONAL_LOADER_HOME,
    RECOVER_YAW_SEEK,
    RECOVER_YAW_SETTLE,
    RECOVER_FINISHED
};

/**
 * @brief 判断一段允许系统时钟回绕的毫秒时长是否已经结束。
 * @param now_ms 当前时刻。
 * @param start_ms 起始时刻。
 * @param duration_ms 需要等待的持续时间。
 * @return true 表示持续时间已经达到或超过目标值。
 */
static bool elapsed(uint32_t now_ms, uint32_t start_ms, uint32_t duration_ms)
{
    /* 无符号减法天然兼容 HAL_GetTick() 约 49 天一次的回绕。 */
    return (uint32_t)(now_ms - start_ms) >= duration_ms;
}

/**
 * @brief 判断实际位置是否进入目标位置的绝对误差范围。
 *
 * 该辅助函数只判断位置误差，不判断电机是否在线、是否超时或是否触发限位。调用者必须
 * 根据所在步骤补充这些安全条件。
 *
 * @param actual 反馈得到的实际位置。
 * @param target 当前步骤要求的目标位置。
 * @param tolerance 允许的最大绝对位置误差，必须为非负值。
 *
 * @return
 * - true：实际位置已经进入目标容差范围；
 * - false：实际位置仍在目标容差范围之外。
 */
static bool near(float actual, float target, float tolerance)
{
    return fabsf(actual - target) <= tolerance;
}

/**
 * @brief 统一进入一个新的顶层业务状态。
 *
 * 函数会同时清零子状态、子步骤计时和一次性动作标志，避免旧状态历史泄漏到新状态。
 *
 * @param sm 状态机对象。
 * @param state 目标顶层状态。
 * @param now_ms 新状态进入时刻。
 */
static void transition(dart_sm_t *sm, dart_state_t state, uint32_t now_ms)
{
    /* 所有状态进入动作统一收口，防止上一状态的计时/单次动作标志泄漏。 */
    sm->status.state = state;
    sm->status.substate = 0U;
    sm->status.state_entered_ms = now_ms;
    sm->substate_started_ms = now_ms;
    sm->action_started = false;
    sm->aim_stable = false;
    sm->status.launch_permitted = false;
    sm->pair_skew_active = false;
    sm->pair_left_homed = false;
    sm->pair_right_homed = false;
}

/**
 * @brief 在当前顶层状态内推进到下一个物理子步骤。
 * @param sm 状态机对象。
 * @param now_ms 新子步骤的独立超时起点。
 */
static void next_substate(dart_sm_t *sm, uint32_t now_ms)
{
    /* 每个物理子步骤拥有独立超时起点和“仅执行一次”标志。 */
    sm->status.substate++;
    sm->substate_started_ms = now_ms;
    sm->action_started = false;
    sm->pair_skew_active = false;
    sm->pair_left_homed = false;
    sm->pair_right_homed = false;
}

/**
 * @brief 禁用指定轴并清除该轴遗留目标。
 *
 * 除切换为禁用模式外，本函数还把目标值清零，防止后续重新启用时意外沿用上一业务步骤
 * 的位置或速度目标。本函数只修改状态机输出，不直接发送 CAN 命令。
 *
 * @param sm 状态机对象，内部输出数组必须已经初始化。
 * @param axis 待禁用的轴编号，必须小于 `DART_AXIS_COUNT`。
 */
static void disable_axis(dart_sm_t *sm, dart_axis_t axis)
{
    sm->output.mode[axis] = DART_AXIS_MODE_DISABLED;
    sm->output.target[axis] = 0.0f;
}

/**
 * @brief 为指定轴写入本周期的位置闭环目标。
 *
 * 本函数只更新统一执行器命令，实际 PID 计算和总线发送由平台层在本周期末完成。
 *
 * @param sm 状态机对象。
 * @param axis 需要控制的轴编号。
 * @param position 以该轴业务单位表示的绝对位置目标。
 */
static void command_position(dart_sm_t *sm, dart_axis_t axis, float position)
{
    sm->output.mode[axis] = DART_AXIS_MODE_POSITION;
    sm->output.target[axis] = position;
}

/**
 * @brief 为指定轴写入本周期的速度闭环目标。
 *
 * 速度模式主要用于寻找机械限位。调用者必须同时负责限位判断和超时处理，避免执行器持续
 * 顶住限位或在反馈失效时无限运动。
 *
 * @param sm 状态机对象。
 * @param axis 需要控制的轴编号。
 * @param speed 带方向符号的速度目标。
 */
static void command_speed(dart_sm_t *sm, dart_axis_t axis, float speed)
{
    sm->output.mode[axis] = DART_AXIS_MODE_SPEED;
    sm->output.target[axis] = speed;
}

/**
 * @brief 在同一周期为左右推板写入互为相反数的位置目标。
 * @param sm 状态机对象。
 * @param left_position 左推板目标；右推板自动使用其相反数。
 */
static void command_push_pair(dart_sm_t *sm, float left_position)
{
    /* 两侧安装方向相反，同一物理方向对应左右互为相反数的目标位置。 */
    command_position(sm, DART_AXIS_PUSH_LEFT, left_position);
    command_position(sm, DART_AXIS_PUSH_RIGHT, -left_position);
}

/**
 * @brief 关闭发射机构并让状态机拥有的核心轴保持安全位置。
 *
 * 本函数不接触升降、转盘和舵机门，因为这些可选硬件可能没有安装。
 *
 * @param sm 状态机对象；必须已绑定当前有效参数。
 */
static void hold_safe_positions(dart_sm_t *sm)
{
    /*
     * 安全保持只覆盖发射核心：关闭发射机构，扳机/推板/Yaw 回安全位置。
     * 此处绝不命令升降、转盘或舵机门，因为这些硬件可能根本没有安装。
     */
    sm->output.launcher_open = false;
    command_position(sm, DART_AXIS_TRIGGER, TRIGGER_RESET_POSITION);
    command_push_pair(sm, sm->parameters->push_back_position);
    command_position(sm, DART_AXIS_YAW, YAW_RESET_POSITION);
    disable_axis(sm, DART_AXIS_LIFT_LEFT);
    disable_axis(sm, DART_AXIS_LIFT_RIGHT);
    sm->output.loader_turn_enabled = false;
}

/**
 * @brief 锁存故障并选择进入受控恢复或立即安全锁定。
 *
 * @param sm 状态机对象。
 * @param fault 需要保留并上报的故障码。
 * @param can_recover true 表示反馈仍足以安全执行恢复；false 表示必须立即停止。
 * @param now_ms 故障发生时刻。
 */
static void report_fault(dart_sm_t *sm, dart_fault_code_t fault, bool can_recover, uint32_t now_ms)
{
    /*
     * 可恢复故障：先中止当前换弹动作，再自动进入严格恢复序列，完成后仍保持故障锁定，
     * 等待人工 CONFIRM_FAULT。不可安全运动的故障：立即禁用所有轴并直接锁定。
     */
    sm->status.fault = fault;
    sm->recovery_from_fault = true;
    sm->status.recovery_required_confirmation = true;
    sm->output.launcher_open = false;
    if (can_recover) {
        sm->reload_ops->abort(sm->reload_context);
        transition(sm, DART_STATE_RECOVERING, now_ms);
    } else {
        for (uint8_t axis = 0U; axis < DART_AXIS_COUNT; ++axis) {
            disable_axis(sm, (dart_axis_t)axis);
        }
        sm->output.loader_turn_enabled = false;
        transition(sm, DART_STATE_FAULT_LATCHED, now_ms);
    }
}

/**
 * @brief 判断当前子步骤是否超过普通动作或回零动作时限。
 *
 * 起始时刻来自最近一次状态或子步骤切换。使用无符号时间差，因此系统毫秒计数器回绕时
 * 仍能得到正确结果。
 *
 * @param sm 状态机对象。
 * @param now_ms 当前单调毫秒时刻。
 * @param homing true 使用回零超时；false 使用普通动作超时。
 *
 * @return
 * - true：当前步骤已经达到或超过对应时限；
 * - false：当前步骤仍处于允许执行时间内。
 */
static bool action_timeout(const dart_sm_t *sm, uint32_t now_ms, bool homing)
{
    uint32_t limit = homing ? sm->parameters->homing_timeout_ms : sm->parameters->action_timeout_ms;
    return elapsed(now_ms, sm->substate_started_ms, limit);
}

/**
 * @brief 检查扳机、推板对和水平轴四个核心反馈是否全部在线。
 *
 * 可拆卸换弹机构的升降轴和转盘不在本函数检查范围内，它们由所选换弹策略自行校验。
 * 这样拆除换弹机构后，核心发射机构仍可完成上电回零和安全待机。
 *
 * @param feedback 当前控制周期的完整反馈快照。
 *
 * @return
 * - true：主状态机直接拥有的四个核心轴反馈全部在线；
 * - false：至少一个核心轴反馈缺失，禁止继续运动或发射。
 */
static bool required_feedback_online(const dart_feedback_t *feedback)
{
    /*
     * 主状态机只要求自己拥有的四个核心轴在线。安装版换弹策略在动作期间自行检查升降轴
     * 和 DM 反馈，因此拆掉可选机构不会阻止 BOOT -> HOMING。
     */
    return feedback->online[DART_AXIS_TRIGGER] &&
           feedback->online[DART_AXIS_PUSH_LEFT] &&
           feedback->online[DART_AXIS_PUSH_RIGHT] &&
           feedback->online[DART_AXIS_YAW];
}

/**
 * @brief 判断左右推板是否分别到达互为镜像的目标位置。
 *
 * 左右推板机械安装方向相反，因此右侧目标为左侧目标的相反数。任意一侧未到位都会保持
 * 当前步骤，保证成对轴不会因单侧先到而提前推进流程。
 *
 * @param feedback 当前控制周期的推板位置反馈。
 * @param target 左推板目标位置，右推板目标由函数自动取相反数。
 *
 * @return
 * - true：左右两侧均进入位置容差；
 * - false：至少一侧尚未到位。
 */
static bool push_pair_reached(const dart_feedback_t *feedback, float target)
{
    return near(feedback->position[DART_AXIS_PUSH_LEFT], target, POSITION_PUSH_EPSILON) &&
           near(feedback->position[DART_AXIS_PUSH_RIGHT], -target, POSITION_PUSH_EPSILON);
}

/**
 * @brief 检查左右推板镜像位置误差是否处于参数允许范围。
 *
 * 理想镜像运动满足“左位置 + 右位置 = 0”。本函数把该绝对和与参数中的同步容差比较，
 * 不单独判断目标是否到达。
 *
 * @param sm 状态机对象，用于读取当前同步误差阈值。
 * @param feedback 当前控制周期的推板位置反馈。
 *
 * @return
 * - true：左右推板同步误差在允许范围内；
 * - false：同步误差超限，调用者必须同时停止两侧并锁存故障。
 */
static bool push_pair_synchronized(const dart_sm_t *sm, const dart_feedback_t *feedback)
{
    /* 理想镜像位置满足 left + right = 0，绝对和即两侧同步误差。 */
    return fabsf(feedback->position[DART_AXIS_PUSH_LEFT] +
                 feedback->position[DART_AXIS_PUSH_RIGHT]) <= sm->parameters->push_sync_tolerance;
}

/**
 * @brief 推进扳机轴寻找机械限位并登记零点的步骤。
 *
 * 限位未触发时持续给出固定方向速度；触发后立即禁用扳机输出、登记编码器零点并进入
 * 下一子步骤。寻找过程超时会进入不可继续运动的回零故障。
 *
 * @param sm 状态机对象。
 * @param feedback 当前控制周期反馈。
 * @param now_ms 当前单调毫秒时刻。
 */
static void trigger_home_step(dart_sm_t *sm, const dart_feedback_t *feedback, uint32_t now_ms)
{
    /* 未触发限位时恒速寻找；触发后先停电流，再把该机械点写为编码器零点。 */
    if (!feedback->limit[DART_AXIS_TRIGGER]) {
        command_speed(sm, DART_AXIS_TRIGGER, TRIGGER_HOME_SPEED);
        if (action_timeout(sm, now_ms, true)) {
            report_fault(sm, DART_FAULT_HOME_TRIGGER, false, now_ms);
        }
        return;
    }
    disable_axis(sm, DART_AXIS_TRIGGER);
    sm->platform->zero_axis(DART_AXIS_TRIGGER);
    next_substate(sm, now_ms);
}

/**
 * @brief 推进扳机轴离开限位并回到安全待机位置的步骤。
 *
 * 只有真实位置进入容差后才推进，避免编码器刚清零但机构仍压住限位时启动后续动作。
 *
 * @param sm 状态机对象。
 * @param feedback 当前控制周期反馈。
 * @param now_ms 当前单调毫秒时刻。
 */
static void trigger_settle_step(dart_sm_t *sm, const dart_feedback_t *feedback, uint32_t now_ms)
{
    /* 离开限位并运动到扳机安全待机位置，真实位置到达后才允许下一步。 */
    command_position(sm, DART_AXIS_TRIGGER, TRIGGER_RESET_POSITION);
    if (near(feedback->position[DART_AXIS_TRIGGER], TRIGGER_RESET_POSITION, POSITION_TRIGGER_EPSILON)) {
        next_substate(sm, now_ms);
    } else if (action_timeout(sm, now_ms, true)) {
        report_fault(sm, DART_FAULT_HOME_TRIGGER, false, now_ms);
    }
}

/**
 * @brief 同步推进左右推板寻找各自限位并分别登记零点。
 *
 * 两侧在同一周期启动。某一侧先触发限位时只停止该侧并登记零点，另一侧允许在限定时间
 * 内继续寻找；两侧触发时间差过大、整体超时或任何必要条件失败都会同时终止流程。必须
 * 两侧都完成后才进入下一步。
 *
 * @param sm 状态机对象，内部保存两侧完成标志和时间差计时。
 * @param feedback 当前控制周期反馈。
 * @param now_ms 当前单调毫秒时刻。
 */
static void push_home_step(dart_sm_t *sm, const dart_feedback_t *feedback, uint32_t now_ms)
{
    /*
     * 左右推板同时开始寻找限位。某一侧先到时立即单独停轴并记录其 offset，另一侧最多
     * 继续 PAIR_LIMIT_SKEW_MS。必须两侧都成功才完成，单侧开关损坏不会被掩盖。
     */
    if (!sm->pair_left_homed) command_speed(sm, DART_AXIS_PUSH_LEFT, PUSH_HOME_SPEED);
    if (!sm->pair_right_homed) command_speed(sm, DART_AXIS_PUSH_RIGHT, -PUSH_HOME_SPEED);
    if (feedback->limit[DART_AXIS_PUSH_LEFT] && !sm->pair_left_homed) {
        sm->platform->zero_axis(DART_AXIS_PUSH_LEFT);
        sm->pair_left_homed = true;
    }
    if (feedback->limit[DART_AXIS_PUSH_RIGHT] && !sm->pair_right_homed) {
        sm->platform->zero_axis(DART_AXIS_PUSH_RIGHT);
        sm->pair_right_homed = true;
    }
    if (sm->pair_left_homed) disable_axis(sm, DART_AXIS_PUSH_LEFT);
    if (sm->pair_right_homed) disable_axis(sm, DART_AXIS_PUSH_RIGHT);

    if (sm->pair_left_homed != sm->pair_right_homed && !sm->pair_skew_active) {
        sm->pair_skew_active = true;
        sm->pair_skew_started_ms = now_ms;
    }
    if (sm->pair_skew_active && sm->pair_left_homed != sm->pair_right_homed &&
        elapsed(now_ms, sm->pair_skew_started_ms, PAIR_LIMIT_SKEW_MS)) {
        report_fault(sm, DART_FAULT_PUSH_SYNC, false, now_ms);
        return;
    }

    if (sm->pair_left_homed && sm->pair_right_homed) {
        next_substate(sm, now_ms);
    } else if (action_timeout(sm, now_ms, true)) {
        report_fault(sm, DART_FAULT_HOME_PUSH, false, now_ms);
    }
}

/**
 * @brief 让已校零的左右推板同步离开限位并返回后位。
 *
 * 本步骤持续检查镜像同步误差，并且只有两侧都到达参数指定的后位才结束。同步误差超限
 * 或回位超时均视为不可安全继续的成对轴故障。
 *
 * @param sm 状态机对象。
 * @param feedback 当前控制周期反馈。
 * @param now_ms 当前单调毫秒时刻。
 */
static void push_settle_step(dart_sm_t *sm, const dart_feedback_t *feedback, uint32_t now_ms)
{
    /* 两侧同步离开限位并返回 BACK；位置差超限时同时停轴并锁定故障。 */
    command_push_pair(sm, sm->parameters->push_back_position);
    if (!push_pair_synchronized(sm, feedback)) {
        report_fault(sm, DART_FAULT_PUSH_SYNC, false, now_ms);
    } else if (push_pair_reached(feedback, sm->parameters->push_back_position)) {
        next_substate(sm, now_ms);
    } else if (action_timeout(sm, now_ms, true)) {
        report_fault(sm, DART_FAULT_HOME_PUSH, false, now_ms);
    }
}

/**
 * @brief 通过当前换弹策略推进可选换弹机构回零。
 *
 * 第一次进入时只启动一次回零事务，之后每个控制周期调用策略的推进函数并读取状态。
 * 安装版策略负责真实机构回零；拆机策略立即完成且不访问硬件。下层故障会原样转换为
 * 主状态机锁定故障。
 *
 * @param sm 状态机对象，必须绑定完整换弹策略操作表。
 * @param now_ms 当前单调毫秒时刻。
 */
static void loader_home_step(dart_sm_t *sm, uint32_t now_ms)
{
    /*
     * 所选策略拥有完整换弹机构回零（包括离开限位后的稳定位置）。拆机策略会立即返回
     * DONE 且不产生输出；主状态机无需知道当前机构类型。
     */
    if (!sm->action_started) {
        if (sm->reload_ops->start_home == NULL ||
            !sm->reload_ops->start_home(sm->reload_context)) {
            report_fault(sm, DART_FAULT_LOADER, false, now_ms);
            return;
        }
        sm->action_started = true;
    }
    sm->reload_ops->step(sm->reload_context, now_ms);
    dart_reload_status_t status = sm->reload_ops->get_status(sm->reload_context);
    if (status.result == DART_RELOAD_DONE) {
        next_substate(sm, now_ms);
    } else if (status.result == DART_RELOAD_FAULT) {
        report_fault(sm, status.fault, false, now_ms);
    }
}

/**
 * @brief 推进 Yaw 轴寻找限位并登记机械零点。
 *
 * 限位未触发时保持固定方向速度；命中限位后立即停止、更新编码器零点并进入离限位步骤。
 * 若在回零时限内没有命中限位，则锁存 Yaw 回零故障。
 *
 * @param sm 状态机对象。
 * @param feedback 当前控制周期反馈。
 * @param now_ms 当前单调毫秒时刻。
 */
static void yaw_home_step(dart_sm_t *sm, const dart_feedback_t *feedback, uint32_t now_ms)
{
    /* Yaw 单轴寻找限位，命中后停止并更新编码器绝对累计 offset。 */
    if (!feedback->limit[DART_AXIS_YAW]) {
        command_speed(sm, DART_AXIS_YAW, YAW_HOME_SPEED);
        if (action_timeout(sm, now_ms, true)) {
            report_fault(sm, DART_FAULT_HOME_YAW, false, now_ms);
        }
        return;
    }
    disable_axis(sm, DART_AXIS_YAW);
    sm->platform->zero_axis(DART_AXIS_YAW);
    next_substate(sm, now_ms);
}

/**
 * @brief 让已校零的 Yaw 轴离开限位并回到安全角度。
 *
 * 到达安全角度前不会推进后续步骤，以免机构长期顶住限位。运动超时将保留当前子步骤并
 * 进入故障锁定，便于定位卡滞位置。
 *
 * @param sm 状态机对象。
 * @param feedback 当前控制周期反馈。
 * @param now_ms 当前单调毫秒时刻。
 */
static void yaw_settle_step(dart_sm_t *sm, const dart_feedback_t *feedback, uint32_t now_ms)
{
    /* 从限位点回到中间安全角度，避免长期顶住机械限位。 */
    command_position(sm, DART_AXIS_YAW, YAW_RESET_POSITION);
    if (near(feedback->position[DART_AXIS_YAW], YAW_RESET_POSITION, POSITION_YAW_EPSILON)) {
        next_substate(sm, now_ms);
    } else if (action_timeout(sm, now_ms, true)) {
        report_fault(sm, DART_FAULT_HOME_YAW, false, now_ms);
    }
}

/**
 * @brief 按固定顺序推进上电自动回零。
 *
 * 顺序为扳机找限位并离开、推板对找限位并返回后位、可选换弹机构回零、水平轴找限位并
 * 回安全位置。任一步未完成时不会跳到后续步骤。
 *
 * @param sm 状态机对象。
 * @param feedback 当前控制周期反馈。
 * @param now_ms 当前单调毫秒时刻。
 */
static void homing_step(dart_sm_t *sm, const dart_feedback_t *feedback, uint32_t now_ms)
{
    /* 每个 case 只允许在自己的真实完成条件满足后 next_substate。 */
    switch (sm->status.substate) {
        case HOME_TRIGGER_SEEK: trigger_home_step(sm, feedback, now_ms); break;
        case HOME_TRIGGER_SETTLE: trigger_settle_step(sm, feedback, now_ms); break;
        case HOME_PUSH_SEEK: push_home_step(sm, feedback, now_ms); break;
        case HOME_PUSH_SETTLE: push_settle_step(sm, feedback, now_ms); break;
        case HOME_OPTIONAL_LOADER: loader_home_step(sm, now_ms); break;
        case HOME_YAW_SEEK: yaw_home_step(sm, feedback, now_ms); break;
        case HOME_YAW_SETTLE: yaw_settle_step(sm, feedback, now_ms); break;
        case HOME_FINISHED:
            sm->status.homed = true;
            sm->status.fault = DART_FAULT_NONE;
            hold_safe_positions(sm);
            transition(sm, DART_STATE_STANDBY, now_ms);
            break;
        default:
            report_fault(sm, DART_FAULT_ACTION_TIMEOUT, false, now_ms);
            break;
    }
}

/**
 * @brief 中止当前动作并从第一步启动完整受控恢复。
 * @param sm 状态机对象。
 * @param from_fault true 表示恢复完成后仍需人工确认故障。
 * @param now_ms 恢复事务开始时刻。
 */
static void start_recovery(dart_sm_t *sm, bool from_fault, uint32_t now_ms)
{
    /* 中止当前机构动作并清除临时发射请求；标定参数不会在业务复位中清除。 */
    sm->reload_ops->abort(sm->reload_context);
    sm->recovery_from_fault = from_fault;
    sm->status.recovery_required_confirmation = from_fault;
    sm->status.homed = false;
    sm->recovery_complete = false;
    sm->fire_requested = false;
    sm->output.launcher_open = false;
    transition(sm, DART_STATE_RECOVERING, now_ms);
}

/**
 * @brief 严格按步骤推进工作中复位或四发结束后的机械恢复。
 *
 * 换弹装置先安全退让，随后依次回零扳机、推板对、换弹机构和水平轴。任何步骤失败都
 * 保留当前步骤号并进入故障锁定，不自动重试、不跳步。
 *
 * @param sm 状态机对象。
 * @param feedback 当前控制周期反馈。
 * @param now_ms 当前单调毫秒时刻。
 */
static void recovery_step(dart_sm_t *sm, const dart_feedback_t *feedback, uint32_t now_ms)
{
    /*
     * 恢复过程不自动重试、不跳步。任一步失败立即锁定并保留 status.substate 供诊断；
     * 只有到达 RECOVER_FINISHED 才统一清除发数、目标和许可等业务上下文。
     */
    switch (sm->status.substate) {
        case RECOVER_RELOAD_SAFE: {
            if (!sm->action_started) {
                if (!sm->reload_ops->start_recover(sm->reload_context)) {
                    report_fault(sm, DART_FAULT_LOADER, false, now_ms);
                    return;
                }
                sm->action_started = true;
            }
            sm->reload_ops->step(sm->reload_context, now_ms);
            dart_reload_status_t status = sm->reload_ops->get_status(sm->reload_context);
            if (status.result == DART_RELOAD_DONE) {
                next_substate(sm, now_ms);
            } else if (status.result == DART_RELOAD_FAULT) {
                report_fault(sm, status.fault, false, now_ms);
            }
            break;
        }
        case RECOVER_TRIGGER_SEEK: trigger_home_step(sm, feedback, now_ms); break;
        case RECOVER_TRIGGER_SETTLE: trigger_settle_step(sm, feedback, now_ms); break;
        case RECOVER_PUSH_SEEK: push_home_step(sm, feedback, now_ms); break;
        case RECOVER_PUSH_SETTLE: push_settle_step(sm, feedback, now_ms); break;
        case RECOVER_OPTIONAL_LOADER_HOME: loader_home_step(sm, now_ms); break;
        case RECOVER_YAW_SEEK: yaw_home_step(sm, feedback, now_ms); break;
        case RECOVER_YAW_SETTLE: yaw_settle_step(sm, feedback, now_ms); break;
        case RECOVER_FINISHED:
            sm->status.shot_index = 0U;
            sm->status.goal = DART_GOAL_NONE;
            sm->status.homed = true;
            sm->fire_requested = false;
            sm->recovery_complete = true;
            hold_safe_positions(sm);
            if (sm->recovery_from_fault) {
                transition(sm, DART_STATE_FAULT_LATCHED, now_ms);
            } else {
                sm->status.fault = DART_FAULT_NONE;
                sm->status.recovery_required_confirmation = false;
                transition(sm, DART_STATE_STANDBY, now_ms);
            }
            break;
        default:
            report_fault(sm, DART_FAULT_ACTION_TIMEOUT, false, now_ms);
            break;
    }
}

/**
 * @brief 建立状态机初态并绑定平台、换弹策略和参数对象。
 *
 * 函数只初始化内存，不访问硬件；调用方必须保证所有依赖对象长期有效。
 */
void dart_sm_init(dart_sm_t *sm,
                  const dart_platform_ops_t *platform,
                  const dart_reload_strategy_t *reload_ops,
                  void *reload_context,
                  const dart_parameters_t *parameters,
                  uint32_t now_ms)
{
    /* init 只建立纯内存初态，不直接驱动硬件；首次 step 才检查反馈并进入 HOMING。 */
    memset(sm, 0, sizeof(*sm));
    sm->platform = platform;
    sm->reload_ops = reload_ops;
    sm->reload_context = reload_context;
    sm->parameters = parameters;
    sm->status.goal = DART_GOAL_NONE;
    sm->status.fault = DART_FAULT_NONE;
    transition(sm, DART_STATE_BOOT, now_ms);
}

/**
 * @brief 按当前状态校验并消费一条可靠业务命令。
 *
 * 目标选择、开始循环、发射、复位、中止和故障确认均在此统一约束，防止外部模块直接
 * 改写状态机内部字段。
 */
void dart_sm_command(dart_sm_t *sm, const dart_command_t *command, uint32_t now_ms)
{
    if (sm == NULL || command == NULL) return;

    switch (command->type) {
        /* 目标选择可以预先保存，真正开始循环仍需 START_CYCLE。 */
        case DART_COMMAND_SELECT_FRONT: sm->status.goal = DART_GOAL_FRONT; break;
        case DART_COMMAND_SELECT_BASE: sm->status.goal = DART_GOAL_BASE; break;
        case DART_COMMAND_FIRE: sm->fire_requested = true; break;
        case DART_COMMAND_RESET:
        case DART_COMMAND_ABORT:
            /* 任意工作状态都走同一受控恢复，不能通过清变量瞬间“假复位”。 */
            start_recovery(sm,
                           sm->status.state == DART_STATE_FAULT_LATCHED ||
                           sm->status.fault != DART_FAULT_NONE,
                           now_ms);
            break;
        case DART_COMMAND_RETRY_RECOVERY:
            if (sm->status.state == DART_STATE_FAULT_LATCHED) start_recovery(sm, true, now_ms);
            break;
        case DART_COMMAND_CONFIRM_FAULT:
            /* 只有机械恢复已经完成的锁定故障才接受人工确认。 */
            if (sm->status.state == DART_STATE_FAULT_LATCHED && sm->recovery_complete) {
                sm->status.fault = DART_FAULT_NONE;
                sm->status.recovery_required_confirmation = false;
                sm->recovery_from_fault = false;
                transition(sm, DART_STATE_STANDBY, now_ms);
            }
            break;
        case DART_COMMAND_ENTER_MANUAL:
            if (sm->status.state == DART_STATE_STANDBY) transition(sm, DART_STATE_MANUAL, now_ms);
            break;
        case DART_COMMAND_EXIT_MANUAL:
            if (sm->status.state == DART_STATE_MANUAL) transition(sm, DART_STATE_STANDBY, now_ms);
            break;
        case DART_COMMAND_START_CYCLE:
            /* 新一轮总是从零号弹开始，且必须已经位于安全待机状态。 */
            if (command->goal != DART_GOAL_NONE) sm->status.goal = command->goal;
            if (sm->status.state == DART_STATE_STANDBY && sm->status.goal != DART_GOAL_NONE) {
                sm->status.shot_index = 0U;
                transition(sm, DART_STATE_PREPARE, now_ms);
            }
            break;
        default: break;
    }
}

/**
 * @brief 执行一次完整但非阻塞的状态机计算。
 *
 * 函数首先清理瞬时许可，再检查核心反馈，随后根据顶层状态写输出并判断真实完成条件。
 * 第四发只在发射保持时间结束后计数，并强制进入完整恢复。
 */
void dart_sm_step(dart_sm_t *sm,
                  const dart_feedback_t *feedback,
                  const vision_target_t *vision,
                  const referee_status_t *referee,
                  uint32_t now_ms)
{
    if (sm == NULL || feedback == NULL || vision == NULL || referee == NULL) return;

    if (sm->status.state == DART_STATE_BOOT) {
        /* 给 CAN 上电和首帧反馈 1 s 宽限；缺少核心轴时禁止开始回零。 */
        if (!required_feedback_online(feedback)) {
            if (elapsed(now_ms, sm->status.state_entered_ms, INPUT_STARTUP_GRACE_MS)) {
                report_fault(sm, DART_FAULT_FEEDBACK_STALE, false, now_ms);
            }
            return;
        }
        transition(sm, DART_STATE_HOMING, now_ms);
    }

    /* 所有可运动状态持续要求核心轴在线；可选机构由其策略自行检查。 */
    if (sm->status.state != DART_STATE_FAULT_LATCHED &&
        !required_feedback_online(feedback)) {
        report_fault(sm, DART_FAULT_FEEDBACK_STALE, false, now_ms);
        return;
    }

    switch (sm->status.state) {
        case DART_STATE_HOMING:
            /* 上电自动回零，完成后才置 homed=true。 */
            homing_step(sm, feedback, now_ms);
            break;
        case DART_STATE_STANDBY:
            /* 安全待机持续闭环保持位置，等待目标和启动命令。 */
            hold_safe_positions(sm);
            break;
        case DART_STATE_MANUAL:
            /* 当前仅预留模式接口，至少强制关闭发射 PWM。 */
            sm->output.launcher_open = false;
            break;
        case DART_STATE_PREPARE:
            /* 验证业务目标后，把具体备弹动作交给换弹策略。 */
            if (sm->status.goal == DART_GOAL_NONE) {
                report_fault(sm, DART_FAULT_PARAMETER_INVALID, true, now_ms);
            } else {
                transition(sm, DART_STATE_RELOAD, now_ms);
            }
            break;
        case DART_STATE_RELOAD: {
            /* start_prepare 只调用一次，随后每毫秒 step，直到 DONE/FAULT。 */
            if (!sm->action_started) {
                if (!sm->reload_ops->start_prepare(sm->reload_context, sm->status.shot_index)) {
                    report_fault(sm, DART_FAULT_LOADER, true, now_ms);
                    break;
                }
                sm->action_started = true;
            }
            sm->reload_ops->step(sm->reload_context, now_ms);
            dart_reload_status_t status = sm->reload_ops->get_status(sm->reload_context);
            sm->status.substate = status.step;
            if (status.result == DART_RELOAD_DONE) transition(sm, DART_STATE_AIM, now_ms);
            else if (status.result == DART_RELOAD_FAULT) report_fault(sm, status.fault, true, now_ms);
            break;
        }
        case DART_STATE_AIM: {
            /* 扳机采用“目标类型 × 发序号”的标定位置，Yaw 在基础值上叠加视觉误差。 */
            command_position(sm, DART_AXIS_TRIGGER,
                             sm->parameters->trigger_distance[sm->status.goal][sm->status.shot_index]);
            float yaw_target = sm->parameters->yaw_position[sm->status.goal];
            bool vision_fresh = !elapsed(now_ms, vision->timestamp_ms, VISION_STALE_MS);
            if (vision_fresh && vision->target_locked) {
                yaw_target = feedback->position[DART_AXIS_YAW] + vision->yaw_error;
                if (fabsf(vision->yaw_error) <= AIM_ERROR_EPSILON) {
                    /* 误差必须连续稳定 AIM_STABLE_MS，单次命中不算瞄准完成。 */
                    if (!sm->aim_stable) {
                        sm->aim_stable = true;
                        sm->aim_stable_started_ms = now_ms;
                    } else if (elapsed(now_ms, sm->aim_stable_started_ms, AIM_STABLE_MS)) {
                        transition(sm, DART_STATE_WAIT_LAUNCH, now_ms);
                    }
                } else {
                    sm->aim_stable = false;
                }
            }
            command_position(sm, DART_AXIS_YAW, yaw_target);
            if (elapsed(now_ms, sm->status.state_entered_ms, sm->parameters->aim_timeout_ms)) {
                report_fault(sm, DART_FAULT_VISION_STALE, true, now_ms);
            }
            break;
        }
        case DART_STATE_WAIT_LAUNCH: {
            /* 裁判与视觉都必须在 200 ms 内更新；旧数据不能授权发射。 */
            bool referee_fresh = !elapsed(now_ms, referee->timestamp_ms, REFEREE_STALE_MS);
            bool vision_fresh = !elapsed(now_ms, vision->timestamp_ms, VISION_STALE_MS);
            bool inputs_fresh = referee_fresh && vision_fresh;
            sm->status.launch_permitted = inputs_fresh && referee->launch_granted;
            if ((sm->status.launch_permitted || (sm->fire_requested && inputs_fresh)) &&
                required_feedback_online(feedback)) {
                sm->fire_requested = false;
                transition(sm, DART_STATE_FIRE, now_ms);
            } else if (!inputs_fresh) {
                sm->fire_requested = false;
            }
            if (elapsed(now_ms, sm->status.state_entered_ms,
                        sm->parameters->launch_wait_timeout_ms)) {
                report_fault(sm, DART_FAULT_REFEREE_STALE, true, now_ms);
            }
            break;
        }
        case DART_STATE_FIRE:
            /* 发射 PWM 保持完整 fire_hold_ms 后才计为一发成功完成。 */
            sm->output.launcher_open = true;
            if (elapsed(now_ms, sm->status.state_entered_ms, sm->parameters->fire_hold_ms)) {
                sm->output.launcher_open = false;
                sm->status.shot_index++;
                transition(sm, DART_STATE_POST_SHOT, now_ms);
            }
            break;
        case DART_STATE_POST_SHOT:
            /* 第 2 发后等待下一裁判窗口；第 4 发进入整机恢复；其余准备下一发。 */
            if (sm->status.shot_index >= DART_SHOT_COUNT) {
                transition(sm, DART_STATE_CYCLE_COMPLETE, now_ms);
            } else if (sm->status.shot_index == 2U) {
                transition(sm, DART_STATE_WAIT_NEXT_WINDOW, now_ms);
            } else {
                transition(sm, DART_STATE_PREPARE, now_ms);
            }
            break;
        case DART_STATE_WAIT_NEXT_WINDOW:
            /* 等待期间核心机构保持安全位置，不保留旧发射许可。 */
            hold_safe_positions(sm);
            if (!elapsed(now_ms, referee->timestamp_ms, REFEREE_STALE_MS) &&
                referee->launch_granted && referee->launch_window >= 2U) {
                transition(sm, DART_STATE_PREPARE, now_ms);
            } else if (elapsed(now_ms, sm->status.state_entered_ms,
                               sm->parameters->next_window_timeout_ms)) {
                report_fault(sm, DART_FAULT_REFEREE_STALE, true, now_ms);
            }
            break;
        case DART_STATE_CYCLE_COMPLETE:
            /* 四发闭环的唯一出口：开始完整恢复并最终清零业务上下文。 */
            start_recovery(sm, false, now_ms);
            break;
        case DART_STATE_RECOVERING:
            /* 逐步恢复，任何失败都不会继续驱动后续轴。 */
            recovery_step(sm, feedback, now_ms);
            break;
        case DART_STATE_FAULT_LATCHED:
            /* 锁定态禁止发射，等待 RETRY_RECOVERY 或恢复后的人工确认。 */
            sm->output.launcher_open = false;
            break;
        default:
            report_fault(sm, DART_FAULT_ACTION_TIMEOUT, false, now_ms);
            break;
    }
}

const dart_status_t *dart_sm_status(const dart_sm_t *sm)
{
    return &sm->status;
}

const dart_actuator_command_t *dart_sm_output(const dart_sm_t *sm)
{
    return &sm->output;
}
