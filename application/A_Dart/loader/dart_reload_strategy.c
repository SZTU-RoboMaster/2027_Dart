#include "dart_reload_strategy.h"

#include <math.h>
#include <string.h>

/*
 * 原转盘升降机构的换弹策略
 * ------------------------
 * 下层机构驱动负责升降轴和 DM 转盘，本策略把这些粗粒度动作与发射机构的同步推板对、
 * 舵机门组合成完整换弹流程。所有步骤非阻塞：每个编号步骤只进入一次，等待真实反馈，
 * 并具有明确超时。
 */
#define PUSH_POSITION_EPSILON 7.0f
#define GATE_MOVE_TIME_MS     1500U
#define TRANSFER_SETTLE_MS    2000U

/**
 * @brief 判断左右推板是否同时到达一组镜像位置目标。
 *
 * @param strategy 换弹策略上下文，用于读取两侧实时位置。
 * @param left_target 左推板目标，右推板目标自动取相反数。
 *
 * @return
 * - true：左右推板均进入允许的位置误差；
 * - false：至少一侧尚未到位。
 */
static bool push_pair_reached(const carousel_reload_strategy_context_t *strategy, float left_target)
{
    return fabsf(strategy->feedback->position[DART_AXIS_PUSH_LEFT] - left_target) <= PUSH_POSITION_EPSILON &&
           fabsf(strategy->feedback->position[DART_AXIS_PUSH_RIGHT] + left_target) <= PUSH_POSITION_EPSILON;
}

/**
 * @brief 检查左右推板当前镜像同步误差。
 *
 * @param strategy 换弹策略上下文，用于读取反馈和同步容差。
 *
 * @return
 * - true：左右位置绝对和未超过同步容差；
 * - false：成对轴同步误差超限，必须同时停止两侧。
 */
static bool push_pair_synchronized(const carousel_reload_strategy_context_t *strategy)
{
    return fabsf(strategy->feedback->position[DART_AXIS_PUSH_LEFT] +
                 strategy->feedback->position[DART_AXIS_PUSH_RIGHT]) <=
           strategy->parameters->push_sync_tolerance;
}

/**
 * @brief 在同一控制周期写入左右推板镜像位置目标。
 *
 * 两侧必须由同一个函数同时切换控制模式和目标，避免代码路径差异导致单侧先启动。
 * 本函数只修改状态机输出，实际电机命令由平台层统一下发。
 *
 * @param strategy 换弹策略上下文。
 * @param left_target 左推板目标，右推板自动使用其相反数。
 */
static void command_push_pair(carousel_reload_strategy_context_t *strategy, float left_target)
{
    /* 左右推板安装方向相反，相同物理行程对应互为相反数的目标。 */
    strategy->output->mode[DART_AXIS_PUSH_LEFT] = DART_AXIS_MODE_POSITION;
    strategy->output->mode[DART_AXIS_PUSH_RIGHT] = DART_AXIS_MODE_POSITION;
    strategy->output->target[DART_AXIS_PUSH_LEFT] = left_target;
    strategy->output->target[DART_AXIS_PUSH_RIGHT] = -left_target;
}

/**
 * @brief 同时撤销左右推板的运动输出。
 *
 * @param strategy 换弹策略上下文。
 */
static void stop_push_pair(carousel_reload_strategy_context_t *strategy)
{
    strategy->output->mode[DART_AXIS_PUSH_LEFT] = DART_AXIS_MODE_DISABLED;
    strategy->output->mode[DART_AXIS_PUSH_RIGHT] = DART_AXIS_MODE_DISABLED;
}

/**
 * @brief 判断当前换弹子步骤是否超过普通动作时限。
 *
 * 无符号时间差允许系统毫秒计数器自然回绕。每次进入新步骤时都会重设独立起点。
 *
 * @param strategy 换弹策略上下文。
 * @param now_ms 当前单调毫秒时刻。
 *
 * @return true 表示当前步骤已经超时，否则返回 false。
 */
static bool timed_out(const carousel_reload_strategy_context_t *strategy, uint32_t now_ms)
{
    return (uint32_t)(now_ms - strategy->step_started_ms) > strategy->parameters->action_timeout_ms;
}

/**
 * @brief 停止换弹事务并锁存失败原因和当前步骤。
 *
 * 故障处理会同时停止推板对并中止下层转盘升降机构，但不会清除步骤号，便于上层记录实际
 * 失败位置。故障后必须由主状态机执行受控恢复或人工处理。
 *
 * @param strategy 换弹策略上下文。
 * @param code 需要向主状态机传递的故障码。
 */
static void fail(carousel_reload_strategy_context_t *strategy, dart_fault_code_t code)
{
    /* 策略故障后必须同时停止推板对，并中止下层换弹机构。 */
    stop_push_pair(strategy);
    strategy->loader_ops->abort(strategy->loader_context);
    strategy->status.result = DART_RELOAD_FAULT;
    strategy->status.fault = code;
}

/**
 * @brief 进入下一个换弹子步骤并重置该步骤运行状态。
 *
 * @param strategy 换弹策略上下文。
 * @param now_ms 新步骤的超时起点。
 */
static void next_step(carousel_reload_strategy_context_t *strategy, uint32_t now_ms)
{
    strategy->status.step++;
    strategy->step_started_ms = now_ms;
    strategy->loader_action_started = false;
}

/**
 * @brief 根据发序号和首发预装配置计算舵机门编号。
 *
 * 首发预装时，零号发次不消耗存储门，后续发次直接使用自身序号；未预装时所有发次向后
 * 偏移一个门号。该映射只负责编号，不会立即驱动舵机。
 *
 * @param strategy 换弹策略上下文。
 * @param shot_index 当前零基发序号。
 *
 * @return 当前发次对应的舵机门编号。
 */
static uint8_t gate_index_for_shot(const carousel_reload_strategy_context_t *strategy,
                                   uint8_t shot_index)
{
    /* 首发预装时零号弹不占用存储门，后续发序号需要向前映射一个门号。 */
    if (strategy->parameters->initial_dart_preloaded) {
        return shot_index;
    }
    return (uint8_t)(shot_index + 1U);
}

/**
 * @brief 启动指定发序号的正常换弹准备事务。
 *
 * 本函数只建立事务初态，不在调用时执行机械运动。随后必须由 DartTask 每毫秒调用
 * `strategy_step()`，直到状态变为完成或故障。
 *
 * @param context 策略私有上下文。
 * @param shot_index 待准备飞镖的零基序号。
 *
 * @return
 * - true：事务已接受并进入运行状态；
 * - false：上下文无效、序号越界或已有事务正在运行。
 */
static bool start_prepare(void *context, uint8_t shot_index)
{
    /* 建立一笔新的正常换弹事务，活动舵机门由发序号和首发预装配置共同决定。 */
    carousel_reload_strategy_context_t *strategy = context;
    if (strategy == NULL || strategy->status.result == DART_RELOAD_BUSY || shot_index >= DART_SHOT_COUNT) {
        return false;
    }
    strategy->shot_index = shot_index;
    strategy->active_gate_index = gate_index_for_shot(strategy, shot_index);
    strategy->recovering = false;
    strategy->home_only = false;
    strategy->status.result = DART_RELOAD_BUSY;
    strategy->status.fault = DART_FAULT_NONE;
    strategy->status.step = 0U;
    strategy->step_started_ms = strategy->platform->now_ms();
    strategy->loader_action_started = false;
    return true;
}

/**
 * @brief 从第一步启动换弹总成的安全退让事务。
 *
 * 恢复始终从步骤零重新开始，不复用被中止事务的步骤和计时信息。
 *
 * @param context 策略私有上下文。
 *
 * @return
 * - true：恢复事务已启动；
 * - false：上下文无效或已有事务正在运行。
 */
static bool start_recover(void *context)
{
    /* 恢复事务总是从安全退让第一步开始，不复用中断时残留的步骤号。 */
    carousel_reload_strategy_context_t *strategy = context;
    if (strategy == NULL || strategy->status.result == DART_RELOAD_BUSY) {
        return false;
    }
    strategy->recovering = true;
    strategy->home_only = false;
    strategy->status.result = DART_RELOAD_BUSY;
    strategy->status.fault = DART_FAULT_NONE;
    strategy->status.step = 0U;
    strategy->step_started_ms = strategy->platform->now_ms();
    strategy->loader_action_started = false;
    return true;
}

/**
 * @brief 启动仅针对换弹机构的上电回零事务。
 *
 * 本事务仅委托下层机构完成转盘和升降轴回零，不执行推板交接与舵机门动作。
 *
 * @param context 策略私有上下文。
 *
 * @return
 * - true：回零事务已启动；
 * - false：上下文无效或已有事务正在运行。
 */
static bool start_home(void *context)
{
    /* 上电回零只委托下层机构回零，不执行推板交接和舵机门流程。 */
    carousel_reload_strategy_context_t *strategy = context;
    if (strategy == NULL || strategy->status.result == DART_RELOAD_BUSY) {
        return false;
    }
    strategy->recovering = false;
    strategy->home_only = true;
    strategy->status.result = DART_RELOAD_BUSY;
    strategy->status.fault = DART_FAULT_NONE;
    strategy->status.step = 0U;
    strategy->step_started_ms = strategy->platform->now_ms();
    strategy->loader_action_started = false;
    return true;
}

/**
 * @brief 启动或推进一个下层机构动作，并把下层故障传递给换弹策略。
 * @param strategy 换弹策略上下文。
 * @param action 需要执行的下层机构动作。
 * @param now_ms 当前单调毫秒时刻。
 * @return true 仅表示下层动作已经完整结束；运行中或故障均返回 false。
 */
static bool run_loader_action(carousel_reload_strategy_context_t *strategy,
                              dart_loader_action_t action,
                              uint32_t now_ms)
{
    /* 保证下层动作只启动一次，此后每周期推进并把具体故障原样传回主状态机。 */
    if (!strategy->loader_action_started) {
        if (!strategy->loader_ops->start(strategy->loader_context, action, strategy->shot_index)) {
            fail(strategy, DART_FAULT_LOADER);
            return false;
        }
        strategy->loader_action_started = true;
    }
    strategy->loader_ops->step(strategy->loader_context, now_ms);
    dart_loader_status_t loader_status = strategy->loader_ops->get_status(strategy->loader_context);
    if (loader_status.result == DART_LOADER_FAULT) {
        fail(strategy, loader_status.fault);
        return false;
    }
    return loader_status.result == DART_LOADER_DONE;
}

/**
 * @brief 推进一发完整正常换弹流程。
 *
 * 流程严格协调推板对、下层换弹机构和舵机门。每一步都等待真实反馈或明确时间条件，
 * 任一步超时、不同步或下层故障都会停止推板并中止机构。
 */
static void prepare_step(carousel_reload_strategy_context_t *strategy, uint32_t now_ms)
{
    /*
     * 正常换弹事务：推板到交接位 -> 转盘送出目标弹 -> 打开舵机门 -> 推板到 654.5 毫米
     * -> 等待交接稳定并完成机构动作 -> 推板同步返回后位。
     */
    if (strategy->shot_index == 0U && strategy->parameters->initial_dart_preloaded) {
        command_push_pair(strategy, strategy->parameters->push_back_position);
        if (push_pair_reached(strategy, strategy->parameters->push_back_position)) {
            strategy->status.result = DART_RELOAD_DONE;
        } else if (timed_out(strategy, now_ms)) {
            fail(strategy, DART_FAULT_ACTION_TIMEOUT);
        }
        return;
    }

    switch (strategy->status.step) {
        case 0:
            /* 推板对先同步到交接等待位置，给升降机构留出运动空间。 */
            command_push_pair(strategy, strategy->parameters->push_exchange_position);
            if (!push_pair_synchronized(strategy)) {
                fail(strategy, DART_FAULT_PUSH_SYNC);
            } else if (push_pair_reached(strategy, strategy->parameters->push_exchange_position)) {
                next_step(strategy, now_ms);
            }
            break;
        case 1:
            /* 下层机构选择目标弹位并让双升降轴到达交接高度。 */
            if (run_loader_action(strategy, LOADER_ACTION_PRESENT_DART, now_ms)) {
                next_step(strategy, now_ms);
            }
            break;
        case 2:
            /* 打开本发对应舵机门，并给舵机保留完整动作时间。 */
            strategy->platform->gate_set(strategy->active_gate_index, true);
            if ((uint32_t)(now_ms - strategy->step_started_ms) >= GATE_MOVE_TIME_MS) {
                next_step(strategy, now_ms);
            }
            break;
        case 3:
            /* 左右推板同步下降到接弹位置，目标即原 set_drive_distance() 的位置。 */
            command_push_pair(strategy, strategy->parameters->push_load_position);
            if (!push_pair_synchronized(strategy)) {
                fail(strategy, DART_FAULT_PUSH_SYNC);
            } else if (push_pair_reached(strategy, strategy->parameters->push_load_position)) {
                strategy->output->launcher_open = false;
                next_step(strategy, now_ms);
            }
            break;
        case 4:
            /* 机械稳定后通知下层交接结束，由其退回下一安全位置。 */
            if ((uint32_t)(now_ms - strategy->step_started_ms) >= TRANSFER_SETTLE_MS &&
                run_loader_action(strategy, LOADER_ACTION_TRANSFER_COMPLETE, now_ms)) {
                next_step(strategy, now_ms);
            }
            break;
        case 5:
            /* 推板对同步返回后位，随后关闭全部舵机门并结束事务。 */
            command_push_pair(strategy, strategy->parameters->push_back_position);
            if (!push_pair_synchronized(strategy)) {
                fail(strategy, DART_FAULT_PUSH_SYNC);
            } else if (push_pair_reached(strategy, strategy->parameters->push_back_position)) {
                strategy->platform->gate_close_all();
                strategy->status.result = DART_RELOAD_DONE;
            }
            break;
        default:
            fail(strategy, DART_FAULT_LOADER);
            break;
    }
    if (strategy->status.result == DART_RELOAD_BUSY && timed_out(strategy, now_ms)) {
        fail(strategy, DART_FAULT_ACTION_TIMEOUT);
    }
}

/**
 * @brief 推进当前转盘升降机构的严格安全恢复流程。
 *
 * 顺序为机构安全退让、推板同步下降到接弹位置、打开有效舵机门、推板同步返回后位并关闭
 * 全部门。函数不自动重试，也不会跳过失败步骤。
 */
static void recover_step(carousel_reload_strategy_context_t *strategy, uint32_t now_ms)
{
    /*
     * 当前机构的严格恢复顺序：
     *  0. 升降轴对到交接高度，然后 DM 转盘回零号弹位；
     *  1. 推板对到原 set_drive_distance() 对应位置，即正负 654.5 毫米；
     *  2. 打开当前活动舵机门；
     *  3. 两侧推板同步返回后位，然后关闭全部换弹舵机门。
     * 完成后，主状态机继续执行扳机、推板对、升降轴对和 Yaw 限位回零。
     */
    strategy->output->launcher_open = false;
    switch (strategy->status.step) {
        case 0:
            /* 双升降先到交接高度，再由下层把转盘回到零号弹位。 */
            if (run_loader_action(strategy, LOADER_ACTION_MAKE_SAFE, now_ms)) {
                next_step(strategy, now_ms);
            }
            break;
        case 1:
            /* 左右推板同步下降到接弹位置，不使用转盘距离换算函数。 */
            command_push_pair(strategy, strategy->parameters->push_load_position);
            if (!push_pair_synchronized(strategy)) {
                fail(strategy, DART_FAULT_PUSH_SYNC);
            } else if (push_pair_reached(strategy, strategy->parameters->push_load_position)) {
                next_step(strategy, now_ms);
            }
            break;
        case 2:
            /* 仅在记录过有效活动门时打开该门，并等待舵机动作完成。 */
            if (strategy->active_gate_index > 0U) {
                strategy->platform->gate_set(strategy->active_gate_index, true);
            }
            if ((uint32_t)(now_ms - strategy->step_started_ms) >= GATE_MOVE_TIME_MS) {
                next_step(strategy, now_ms);
            }
            break;
        case 3:
            /* 推板对同步退回后位，关闭全部舵机门并清除门号。 */
            command_push_pair(strategy, strategy->parameters->push_back_position);
            if (!push_pair_synchronized(strategy)) {
                fail(strategy, DART_FAULT_PUSH_SYNC);
            } else if (push_pair_reached(strategy, strategy->parameters->push_back_position)) {
                strategy->platform->gate_close_all();
                strategy->active_gate_index = 0U;
                strategy->status.result = DART_RELOAD_DONE;
            }
            break;
        default:
            fail(strategy, DART_FAULT_LOADER);
            break;
    }
    if (strategy->status.result == DART_RELOAD_BUSY && timed_out(strategy, now_ms)) {
        fail(strategy, DART_FAULT_ACTION_TIMEOUT);
    }
}

/**
 * @brief 根据当前事务类型推进换弹策略一个控制周期。
 *
 * 非运行状态下本函数不产生动作。运行状态下根据“仅回零、受控恢复、正常准备”三个标志
 * 选择唯一流程，保证同一周期不会推进两条机械路径。
 *
 * @param context 策略私有上下文。
 * @param now_ms 当前单调毫秒时刻。
 */
static void strategy_step(void *context, uint32_t now_ms)
{
    carousel_reload_strategy_context_t *strategy = context;
    if (strategy == NULL || strategy->status.result != DART_RELOAD_BUSY) {
        return;
    }
    if (strategy->home_only) {
        if (run_loader_action(strategy, LOADER_ACTION_HOME, now_ms)) {
            strategy->status.result = DART_RELOAD_DONE;
        }
    } else if (strategy->recovering) {
        recover_step(strategy, now_ms);
    } else {
        prepare_step(strategy, now_ms);
    }
}

/**
 * @brief 中止策略和下层机构，并同时撤销左右推板输出。
 *
 * 本函数可重复调用。中止后状态回到空闲，后续必须显式启动新事务，不会从旧步骤续跑。
 *
 * @param context 策略私有上下文；空指针会被安全忽略。
 */
static void strategy_abort(void *context)
{
    carousel_reload_strategy_context_t *strategy = context;
    if (strategy == NULL) {
        return;
    }
    stop_push_pair(strategy);
    /* abort 必须同步撤销下层换弹机构拥有的全部运动输出。 */
    strategy->loader_ops->abort(strategy->loader_context);
    strategy->status.result = DART_RELOAD_IDLE;
    strategy->status.fault = DART_FAULT_NONE;
}

/**
 * @brief 按值读取当前换弹事务状态。
 *
 * @param context 策略私有上下文。
 *
 * @return 当前事务状态副本；上下文为空时返回换弹机构故障。
 */
static dart_reload_status_t strategy_status(void *context)
{
    carousel_reload_strategy_context_t *strategy = context;
    dart_reload_status_t invalid = { DART_RELOAD_FAULT, DART_FAULT_LOADER, 0U };
    return strategy == NULL ? invalid : strategy->status;
}

static const dart_reload_strategy_t ops = {
    .start_prepare = start_prepare,
    .start_recover = start_recover,
    .start_home = start_home,
    .step = strategy_step,
    .abort = strategy_abort,
    .get_status = strategy_status,
};

/**
 * @brief 清零换弹策略并绑定平台、机构、反馈、输出和参数对象。
 *
 * 本函数不启动物理动作；初始化后策略处于空闲状态。
 * 所有依赖对象都由 Dart 运行时持有，必须在策略使用期间保持有效。
 *
 * @param strategy 待初始化的策略上下文。
 * @param platform 当前板级操作表。
 * @param loader_ops 下层换弹机构操作表。
 * @param loader_context 下层换弹机构私有上下文。
 * @param feedback 每周期更新的硬件反馈快照。
 * @param output 状态机执行器输出对象。
 * @param parameters 当前有效参数对象。
 */
void carousel_reload_strategy_setup(carousel_reload_strategy_context_t *strategy,
                                    const dart_platform_ops_t *platform,
                                    const dart_loader_ops_t *loader_ops,
                                    void *loader_context,
                                    const dart_feedback_t *feedback,
                                    dart_actuator_command_t *output,
                                    const dart_parameters_t *parameters)
{
    memset(strategy, 0, sizeof(*strategy));
    strategy->platform = platform;
    strategy->loader_ops = loader_ops;
    strategy->loader_context = loader_context;
    strategy->feedback = feedback;
    strategy->output = output;
    strategy->parameters = parameters;
    strategy->status.result = DART_RELOAD_IDLE;
}

/**
 * @brief 获取转盘升降换弹策略的静态操作表。
 *
 * @return 静态只读操作表地址，程序运行期间始终有效且无需释放。
 */
const dart_reload_strategy_t *carousel_reload_strategy_ops(void)
{
    return &ops;
}
