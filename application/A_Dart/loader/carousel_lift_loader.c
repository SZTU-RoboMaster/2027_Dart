#include "carousel_lift_loader.h"

#include <math.h>
#include <string.h>

/*
 * 原可替换换弹总成驱动
 * --------------------
 * 总成由一个 DM6006 转盘和两个同向升降轴组成。左右升降电机被视为一个逻辑执行器：
 * 同时启动、分别命中限位、两侧共同到位；任一侧离线、不同步或超时都同时停止两侧。
 * 当前机构已拆除，因此默认固件不会编译本文件。
 */
#define POSITION_EPSILON       0.30f
#define TURN_POSITION_EPSILON  0.10f
#define PAIR_LIMIT_SKEW_MS     250U
#define LIFT_HOME_SPEED       -4000.0f
#define LIFT_HOME_POSITION        3.0f

/**
 * @brief 判断机构单轴位置是否进入目标容差。
 *
 * 本函数只比较位置误差，不判断反馈在线状态和动作超时；这些条件由具体机构步骤统一处理。
 *
 * @param actual 当前反馈位置。
 * @param target 当前步骤目标位置。
 * @param tolerance 允许的最大绝对位置误差。
 *
 * @return true 表示已经到位，否则返回 false。
 */
static bool position_reached(float actual, float target, float tolerance)
{
    /* 机构驱动只按绝对误差判断到位，不依赖电机控制器内部状态。 */
    return fabsf(actual - target) <= tolerance;
}

/**
 * @brief 检查左右升降轴的位置差是否处于允许范围。
 *
 * 当前机构两侧同向安装，因此理想状态下左右位置相等。该检查贯穿所有成对升降动作，任意
 * 时刻超差都必须停止两侧，而不能只停落后的一侧。
 *
 * @param loader 换弹机构上下文。
 *
 * @return true 表示两侧同步正常，false 表示位置差超限。
 */
static bool lift_pair_synchronized(const carousel_lift_loader_t *loader)
{
    /* 两侧机械方向相同，正常情况下左右位置之差应接近零。 */
    return fabsf(loader->feedback->position[DART_AXIS_LIFT_LEFT] -
                 loader->feedback->position[DART_AXIS_LIFT_RIGHT]) <=
           loader->parameters->lift_sync_tolerance;
}

/**
 * @brief 同时写入左右升降轴的位置闭环目标。
 *
 * @param loader 换弹机构上下文。
 * @param position 左右升降轴共同的目标位置。
 */
static void lift_position(carousel_lift_loader_t *loader, float position)
{
    /* 左右轴在同一个调用中写入相同目标，禁止上层分别启动。 */
    loader->output->mode[DART_AXIS_LIFT_LEFT] = DART_AXIS_MODE_POSITION;
    loader->output->mode[DART_AXIS_LIFT_RIGHT] = DART_AXIS_MODE_POSITION;
    loader->output->target[DART_AXIS_LIFT_LEFT] = position;
    loader->output->target[DART_AXIS_LIFT_RIGHT] = position;
}

/**
 * @brief 同时撤销左右升降轴的运动输出。
 *
 * @param loader 换弹机构上下文。
 */
static void lift_stop(carousel_lift_loader_t *loader)
{
    loader->output->mode[DART_AXIS_LIFT_LEFT] = DART_AXIS_MODE_DISABLED;
    loader->output->mode[DART_AXIS_LIFT_RIGHT] = DART_AXIS_MODE_DISABLED;
}

/**
 * @brief 停止换弹机构并锁存故障与失败步骤。
 *
 * 发生故障后同时禁用升降轴对和转盘输出，但保留 `status.step`，供主状态机和操作员定位
 * 失败位置。本函数不会自动重试或跳到后续步骤。
 *
 * @param loader 换弹机构上下文。
 * @param fault 需要向上层报告的具体故障码。
 */
static void loader_fail(carousel_lift_loader_t *loader, dart_fault_code_t fault)
{
    /* 机构故障必须同时撤销整个成对轴的输出，不能只停故障侧。 */
    lift_stop(loader);
    loader->output->loader_turn_enabled = false;
    loader->status.result = DART_LOADER_FAULT;
    loader->status.fault = fault;
}

/**
 * @brief 判断当前机构子步骤是否超过普通动作时限。
 *
 * @param loader 换弹机构上下文。
 * @param now_ms 当前单调毫秒时刻。
 *
 * @return true 表示步骤已超时，否则返回 false。
 */
static bool step_timed_out(const carousel_lift_loader_t *loader, uint32_t now_ms)
{
    return (uint32_t)(now_ms - loader->step_started_ms) > loader->parameters->action_timeout_ms;
}

/**
 * @brief 进入下一个机构子步骤并清除成对轴临时状态。
 *
 * @param loader 换弹机构上下文。
 * @param now_ms 新步骤的独立超时起点。
 */
static void next_step(carousel_lift_loader_t *loader, uint32_t now_ms)
{
    loader->status.step++;
    loader->step_started_ms = now_ms;
    loader->pair_skew_active = false;
    loader->left_homed = false;
    loader->right_homed = false;
}

/**
 * @brief 检查左右升降轴反馈是否同时在线。
 *
 * @param loader 换弹机构上下文。
 *
 * @return true 表示两侧均在线，false 表示至少一侧反馈缺失。
 */
static bool lift_feedback_online(const carousel_lift_loader_t *loader)
{
    return loader->feedback->online[DART_AXIS_LIFT_LEFT] &&
           loader->feedback->online[DART_AXIS_LIFT_RIGHT];
}

/**
 * @brief 判断指定机构动作是否依赖 DM 转盘实时反馈。
 *
 * 单独的升降回零不依赖转盘在线状态；选弹、交接退让和恢复安全姿态需要转盘参与，因此
 * 必须在动作前确认其反馈有效。
 *
 * @param action 待判断的机构动作类型。
 *
 * @return true 表示动作需要转盘反馈，否则返回 false。
 */
static bool action_requires_turn_feedback(dart_loader_action_t action)
{
    return action == LOADER_ACTION_PRESENT_DART ||
           action == LOADER_ACTION_TRANSFER_COMPLETE ||
           action == LOADER_ACTION_MAKE_SAFE;
}

/**
 * @brief 校验并初始化原换弹机构驱动。
 *
 * 本函数只检查依赖指针并建立软件初态，不驱动升降轴或转盘，也不执行机械回零。
 *
 * @param context 指向 carousel_lift_loader_t 的私有上下文。
 *
 * @return
 * - true：依赖完整，驱动已进入初始化完成状态；
 * - false：上下文或任一必要依赖无效。
 */
static bool loader_init(void *context)
{
    carousel_lift_loader_t *loader = context;
    if (loader == NULL || loader->platform == NULL || loader->feedback == NULL ||
        loader->output == NULL || loader->parameters == NULL) {
        return false;
    }
    loader->action = LOADER_ACTION_INIT;
    loader->status.result = DART_LOADER_DONE;
    loader->status.fault = DART_FAULT_NONE;
    loader->status.step = 0U;
    return true;
}

/**
 * @brief 启动一项非阻塞机构动作。
 *
 * 本函数只登记动作、发序号和计时起点，不等待电机运动。已有动作处于忙状态时拒绝覆盖。
 *
 * @param context 机构私有上下文。
 * @param action 要启动的粗粒度机构动作。
 * @param shot_index 当前零基发序号。
 *
 * @return
 * - true：动作已登记，后续可由周期函数推进；
 * - false：上下文无效或已有动作正在运行。
 */
static bool loader_start(void *context, dart_loader_action_t action, uint8_t shot_index)
{
    /* 只接受一个在途动作；每次启动都会清除上一动作的步骤和回零边沿记录。 */
    carousel_lift_loader_t *loader = context;
    if (loader == NULL || loader->status.result == DART_LOADER_BUSY) {
        return false;
    }
    loader->action = action;
    loader->shot_index = shot_index;
    loader->status.result = (action == LOADER_ACTION_ABORT) ? DART_LOADER_DONE : DART_LOADER_BUSY;
    loader->status.fault = DART_FAULT_NONE;
    loader->status.step = 0U;
    loader->step_started_ms = loader->platform->now_ms();
    loader->pair_skew_active = false;
    loader->left_homed = false;
    loader->right_homed = false;
    if (action == LOADER_ACTION_ABORT) {
        lift_stop(loader);
        loader->output->loader_turn_enabled = false;
    }
    return true;
}

/**
 * @brief 推进“选择并呈送指定飞镖”动作。
 *
 * 先等待转盘到对应弹位，再让左右升降轴同步到交接高度。任一反馈离线、位置差超限或步骤
 * 超时都会同时停止成对轴并进入机构故障。
 *
 * @param loader 换弹机构上下文。
 * @param now_ms 当前单调毫秒时刻。
 */
static void present_step(carousel_lift_loader_t *loader, uint32_t now_ms)
{
    /* 首发预装时无需移动机构，直接确认准备完成。 */
    if (loader->shot_index == 0U && loader->parameters->initial_dart_preloaded) {
        loader->status.result = DART_LOADER_DONE;
        return;
    }

    const uint8_t turn_index = loader->shot_index == 0U
                                   ? 0U
                                   : (uint8_t)(loader->shot_index * 2U - 1U);
    if (turn_index >= DART_LOADER_POSITION_COUNT) {
        loader_fail(loader, DART_FAULT_LOADER);
        return;
    }

    if (loader->status.step == 0U) {
        /* 第一步：转盘选择本发对应的储弹位置，并等待真实角度到达。 */
        loader->output->loader_turn_enabled = true;
        loader->output->loader_turn_target = loader->parameters->loader_turn_position[turn_index];
        if (position_reached(loader->feedback->loader_turn_position,
                             loader->output->loader_turn_target,
                             TURN_POSITION_EPSILON)) {
            next_step(loader, now_ms);
        }
    } else {
        /* 第二步：左右升降轴同步上升到交接高度，任何时刻都检查位置差。 */
        lift_position(loader, loader->parameters->loader_lift_position);
        if (!lift_pair_synchronized(loader)) {
            loader_fail(loader, DART_FAULT_LIFT_SYNC);
            return;
        }
        if (position_reached(loader->feedback->position[DART_AXIS_LIFT_LEFT],
                             loader->parameters->loader_lift_position,
                             POSITION_EPSILON) &&
            position_reached(loader->feedback->position[DART_AXIS_LIFT_RIGHT],
                             loader->parameters->loader_lift_position,
                             POSITION_EPSILON)) {
            loader->status.result = DART_LOADER_DONE;
        }
    }

    if (loader->status.result == DART_LOADER_BUSY && step_timed_out(loader, now_ms)) {
        loader_fail(loader, DART_FAULT_ACTION_TIMEOUT);
    }
}

/**
 * @brief 推进“飞镖交接完成后的机构退让”动作。
 *
 * 转盘和双升降轴可以同步运动，但只有两部分都到位且升降同步正常时才返回完成。
 *
 * @param loader 换弹机构上下文。
 * @param now_ms 当前单调毫秒时刻。
 */
static void transfer_complete_step(carousel_lift_loader_t *loader, uint32_t now_ms)
{
    /* 飞镖交接后，转盘切到下一安全位置，同时让双升降轴同步回到低位。 */
    uint8_t turn_index = (uint8_t)(loader->shot_index * 2U);
    if (turn_index >= DART_LOADER_POSITION_COUNT) {
        turn_index = DART_LOADER_POSITION_COUNT - 1U;
    }
    loader->output->loader_turn_enabled = true;
    loader->output->loader_turn_target = loader->parameters->loader_turn_position[turn_index];
    lift_position(loader, 3.0f);

    if (!lift_pair_synchronized(loader)) {
        loader_fail(loader, DART_FAULT_LIFT_SYNC);
        return;
    }
    if (position_reached(loader->feedback->loader_turn_position,
                         loader->output->loader_turn_target,
                         TURN_POSITION_EPSILON) &&
        position_reached(loader->feedback->position[DART_AXIS_LIFT_LEFT], 3.0f, POSITION_EPSILON) &&
        position_reached(loader->feedback->position[DART_AXIS_LIFT_RIGHT], 3.0f, POSITION_EPSILON)) {
        loader->status.result = DART_LOADER_DONE;
    } else if (step_timed_out(loader, now_ms)) {
        loader_fail(loader, DART_FAULT_ACTION_TIMEOUT);
    }
}

/**
 * @brief 把换弹机构移动到允许整机恢复的安全姿态。
 *
 * 严格先将双升降轴同步送到交接高度，再把转盘转回零号弹位，禁止颠倒步骤。
 *
 * @param loader 换弹机构上下文。
 * @param now_ms 当前单调毫秒时刻。
 */
static void make_safe_step(carousel_lift_loader_t *loader, uint32_t now_ms)
{
    /* 恢复前先把升降轴送到交接高度，确认到位后再将转盘回到零号位置。 */
    if (loader->status.step == 0U) {
        lift_position(loader, loader->parameters->loader_lift_position);
        if (!lift_pair_synchronized(loader)) {
            loader_fail(loader, DART_FAULT_LIFT_SYNC);
            return;
        }
        if (position_reached(loader->feedback->position[DART_AXIS_LIFT_LEFT],
                             loader->parameters->loader_lift_position,
                             POSITION_EPSILON) &&
            position_reached(loader->feedback->position[DART_AXIS_LIFT_RIGHT],
                             loader->parameters->loader_lift_position,
                             POSITION_EPSILON)) {
            next_step(loader, now_ms);
        }
    } else {
        loader->output->loader_turn_enabled = true;
        loader->output->loader_turn_target = loader->parameters->loader_turn_position[0];
        if (position_reached(loader->feedback->loader_turn_position,
                             loader->output->loader_turn_target,
                             TURN_POSITION_EPSILON)) {
            loader->status.result = DART_LOADER_DONE;
        }
    }
    if (loader->status.result == DART_LOADER_BUSY && step_timed_out(loader, now_ms)) {
        loader_fail(loader, DART_FAULT_ACTION_TIMEOUT);
    }
}

/**
 * @brief 推进左右升降轴的成对限位回零。
 *
 * 两侧同时寻找各自限位，先到侧单独停止并记录零点；另一侧必须在允许时间差内到达。两侧
 * 都成功后同步离开限位并到达待机高度，才算机构回零完成。
 *
 * @param loader 换弹机构上下文。
 * @param now_ms 当前单调毫秒时刻。
 */
static void home_step(carousel_lift_loader_t *loader, uint32_t now_ms)
{
    if (loader->status.step == 0U) {
        /* 左右同时寻找各自限位；先到的一侧立即停止，另一侧只能在有限时间内继续寻找。 */
        if (loader->feedback->limit[DART_AXIS_LIFT_LEFT] && !loader->left_homed) {
            loader->platform->zero_axis(DART_AXIS_LIFT_LEFT);
            loader->left_homed = true;
        }
        if (loader->feedback->limit[DART_AXIS_LIFT_RIGHT] && !loader->right_homed) {
            loader->platform->zero_axis(DART_AXIS_LIFT_RIGHT);
            loader->right_homed = true;
        }

        loader->output->mode[DART_AXIS_LIFT_LEFT] =
            loader->left_homed ? DART_AXIS_MODE_DISABLED : DART_AXIS_MODE_SPEED;
        loader->output->mode[DART_AXIS_LIFT_RIGHT] =
            loader->right_homed ? DART_AXIS_MODE_DISABLED : DART_AXIS_MODE_SPEED;
        loader->output->target[DART_AXIS_LIFT_LEFT] = LIFT_HOME_SPEED;
        loader->output->target[DART_AXIS_LIFT_RIGHT] = LIFT_HOME_SPEED;

        if (loader->left_homed != loader->right_homed && !loader->pair_skew_active) {
            loader->pair_skew_active = true;
            loader->pair_skew_started_ms = now_ms;
        }
        if (loader->pair_skew_active && loader->left_homed != loader->right_homed &&
            (uint32_t)(now_ms - loader->pair_skew_started_ms) > PAIR_LIMIT_SKEW_MS) {
            loader_fail(loader, DART_FAULT_LIFT_SYNC);
            return;
        }
        if (loader->left_homed && loader->right_homed) {
            lift_stop(loader);
            next_step(loader, now_ms);
        } else if ((uint32_t)(now_ms - loader->step_started_ms) >
                   loader->parameters->homing_timeout_ms) {
            loader_fail(loader, DART_FAULT_HOME_LIFT);
        }
        return;
    }

    if (loader->status.step == 1U) {
        /* 两侧同步离开限位并到达标定待机高度。该步骤留在驱动内部，主状态机不依赖
         * 当前机构的具体回零方式。 */
        lift_position(loader, LIFT_HOME_POSITION);
        if (!lift_pair_synchronized(loader)) {
            loader_fail(loader, DART_FAULT_LIFT_SYNC);
        } else if (position_reached(loader->feedback->position[DART_AXIS_LIFT_LEFT],
                                    LIFT_HOME_POSITION, POSITION_EPSILON) &&
                   position_reached(loader->feedback->position[DART_AXIS_LIFT_RIGHT],
                                    LIFT_HOME_POSITION, POSITION_EPSILON)) {
            loader->status.result = DART_LOADER_DONE;
        } else if ((uint32_t)(now_ms - loader->step_started_ms) >
                   loader->parameters->homing_timeout_ms) {
            loader_fail(loader, DART_FAULT_HOME_LIFT);
        }
        return;
    }

    loader_fail(loader, DART_FAULT_HOME_LIFT);
}

/**
 * @brief 检查机构反馈并推进当前动作一个控制周期。
 *
 * 仅在动作状态为运行中时工作。函数先检查当前动作所需反馈，再分派到唯一动作步骤；
 * 反馈缺失会在产生进一步运动前转换为机构故障。
 *
 * @param context 机构私有上下文。
 * @param now_ms 当前单调毫秒时刻。
 */
static void loader_step(void *context, uint32_t now_ms)
{
    carousel_lift_loader_t *loader = context;
    if (loader == NULL || loader->status.result != DART_LOADER_BUSY) {
        return;
    }

    /* 可选机构反馈由本驱动自己负责检查；主状态机不会因为拆机后缺少这些反馈而拒绝启动。 */
    if (loader->action != LOADER_ACTION_INIT &&
        loader->action != LOADER_ACTION_ABORT &&
        !lift_feedback_online(loader)) {
        loader_fail(loader, DART_FAULT_FEEDBACK_STALE);
        return;
    }
    if (action_requires_turn_feedback(loader->action) &&
        !loader->feedback->loader_turn_online) {
        loader_fail(loader, DART_FAULT_FEEDBACK_STALE);
        return;
    }
    switch (loader->action) {
        case LOADER_ACTION_INIT:
            loader->status.result = DART_LOADER_DONE;
            break;
        case LOADER_ACTION_PRESENT_DART:
            present_step(loader, now_ms);
            break;
        case LOADER_ACTION_TRANSFER_COMPLETE:
            transfer_complete_step(loader, now_ms);
            break;
        case LOADER_ACTION_MAKE_SAFE:
            make_safe_step(loader, now_ms);
            break;
        case LOADER_ACTION_HOME:
            home_step(loader, now_ms);
            break;
        case LOADER_ACTION_ABORT:
        default:
            loader->status.result = DART_LOADER_DONE;
            break;
    }
}

/**
 * @brief 立即中止当前机构动作并撤销升降与转盘输出。
 *
 * 中止不会尝试自动回位，因为反馈异常时继续运动可能不安全。后续恢复必须由主状态机
 * 重新发起明确事务。
 *
 * @param context 机构私有上下文；空指针会被安全忽略。
 */
static void loader_abort(void *context)
{
    carousel_lift_loader_t *loader = context;
    if (loader == NULL) {
        return;
    }
    /* 中止不尝试继续回位，只立即撤销本驱动拥有的升降和转盘输出。 */
    lift_stop(loader);
    loader->output->loader_turn_enabled = false;
    loader->status.result = DART_LOADER_IDLE;
    loader->status.fault = DART_FAULT_NONE;
}

/**
 * @brief 按值读取换弹机构当前动作状态。
 *
 * @param context 机构私有上下文。
 *
 * @return 当前动作状态副本；上下文为空时返回机构故障。
 */
static dart_loader_status_t loader_status(void *context)
{
    carousel_lift_loader_t *loader = context;
    dart_loader_status_t invalid = { DART_LOADER_FAULT, DART_FAULT_LOADER, 0U };
    return loader == NULL ? invalid : loader->status;
}

static const dart_loader_ops_t ops = {
    .init = loader_init,
    .start = loader_start,
    .step = loader_step,
    .abort = loader_abort,
    .get_status = loader_status,
};

/**
 * @brief 清零机构上下文并绑定全部长期依赖。
 *
 * 本函数不启动运动，调用后驱动处于空闲状态，必须再通过操作表启动具体动作。
 * 传入依赖均由 Dart 运行时持有，必须在驱动整个生命周期内保持有效。
 *
 * @param loader 待初始化的换弹机构上下文。
 * @param platform 当前板级操作表。
 * @param feedback 每周期刷新的硬件反馈快照。
 * @param output 状态机执行器输出对象。
 * @param parameters 当前有效参数对象。
 */
void carousel_lift_loader_setup(carousel_lift_loader_t *loader,
                                const dart_platform_ops_t *platform,
                                const dart_feedback_t *feedback,
                                dart_actuator_command_t *output,
                                const dart_parameters_t *parameters)
{
    memset(loader, 0, sizeof(*loader));
    loader->platform = platform;
    loader->feedback = feedback;
    loader->output = output;
    loader->parameters = parameters;
    loader->status.result = DART_LOADER_IDLE;
}

/**
 * @brief 获取转盘升降换弹机构的静态操作表。
 *
 * @return 静态只读操作表地址，程序运行期间始终有效且无需释放。
 */
const dart_loader_ops_t *carousel_lift_loader_ops(void)
{
    return &ops;
}
