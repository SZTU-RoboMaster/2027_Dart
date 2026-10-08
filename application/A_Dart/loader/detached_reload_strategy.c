#include "detached_reload_strategy.h"

#include <string.h>

/**
 * @brief 在换弹机构拆除时判断指定发次是否无需机构动作即可准备。
 *
 * 当前拆机策略没有任何可驱动的换弹执行器，因此只能接受“第一发已经由人工预装”这一
 * 特殊情况。发序号从零开始：当 `shot_index` 为 0 且参数明确允许首发预装时，本次准备
 * 立即完成；其他发次均记录为换弹机构不可用。返回值只表示请求是否成功提交，动作结果
 * 必须通过 `detached_get_status()` 读取，以保持和真实异步换弹策略相同的调用约定。
 *
 * @param context 拆机策略上下文，内部必须绑定有效参数对象。
 * @param shot_index 待准备飞镖的零基发序号，合法范围为 0 至 `DART_SHOT_COUNT - 1`。
 *
 * @return
 * - true：请求参数有效，结果已经写入策略状态；
 * - false：策略上下文为空，或尚未绑定参数对象。
 */
static bool detached_start_prepare(void *context, uint8_t shot_index)
{
    detached_reload_strategy_context_t *strategy = context;
    if (strategy == NULL || strategy->parameters == NULL) {
        return false;
    }

    strategy->status.step = 0U;
    if (shot_index == 0U && strategy->parameters->initial_dart_preloaded) {
        strategy->status.result = DART_RELOAD_DONE;
        strategy->status.fault = DART_FAULT_NONE;
    } else {
        strategy->status.result = DART_RELOAD_FAULT;
        strategy->status.fault = DART_FAULT_LOADER_UNAVAILABLE;
    }
    return true;
}

/**
 * @brief 完成拆机配置下的换弹机构安全退让。
 *
 * 拆机配置不拥有转盘、升降轴或舵机门，因此没有需要执行的机械退让动作。本函数直接将
 * 状态置为完成，使主状态机能够继续执行扳机、推板和 Yaw 等仍然存在的核心轴恢复流程。
 *
 * @param context 拆机策略上下文。
 *
 * @return
 * - true：上下文有效，退让事务已立即完成；
 * - false：上下文为空，无法保存事务结果。
 */
static bool detached_start_recover(void *context)
{
    detached_reload_strategy_context_t *strategy = context;
    if (strategy == NULL) return false;
    strategy->status = (dart_reload_status_t){ DART_RELOAD_DONE, DART_FAULT_NONE, 0U };
    return true;
}

/**
 * @brief 完成拆机配置下的换弹机构回零请求。
 *
 * 未安装换弹机构时不存在可校零的机构轴，所以其安全位置与回零位置等价。本函数复用安全
 * 退让逻辑并立即完成，不会访问已经拆除的 GPIO、CAN 电机或舵机资源。
 *
 * @param context 拆机策略上下文。
 *
 * @return
 * - true：上下文有效，回零事务已立即完成；
 * - false：上下文为空。
 */
static bool detached_start_home(void *context)
{
    return detached_start_recover(context);
}

/**
 * @brief 保持与真实策略一致的周期推进接口。
 *
 * 拆机策略不拥有执行器和异步动作，因此本函数有意为空操作。
 *
 * @param context 拆机策略上下文，本实现无需读取。
 * @param now_ms 当前单调毫秒时刻，本实现无需使用。
 */
static void detached_step(void *context, uint32_t now_ms)
{
    (void)context;
    (void)now_ms;
}

/**
 * @brief 中止拆机策略当前事务并恢复为空闲状态。
 *
 * 本函数只清除软件状态，不产生任何硬件输出。重复调用是安全的，可供主状态机在复位、
 * 故障锁定或切换业务周期时统一调用。
 *
 * @param context 拆机策略上下文；为空时直接返回。
 */
static void detached_abort(void *context)
{
    detached_reload_strategy_context_t *strategy = context;
    if (strategy == NULL) return;
    strategy->status = (dart_reload_status_t){ DART_RELOAD_IDLE, DART_FAULT_NONE, 0U };
}

/**
 * @brief 按值读取拆机策略当前状态。
 *
 * 返回副本可以避免主状态机持有策略内部可变存储的指针。若上下文无效，则返回换弹机构
 * 不可用故障，使系统保持禁止发射的安全状态。
 *
 * @param context 拆机策略上下文。
 *
 * @return 当前状态快照；上下文为空时返回 `DART_RELOAD_FAULT`。
 */
static dart_reload_status_t detached_get_status(void *context)
{
    /* 对空上下文返回明确的不可用故障，避免调用者把无效对象误判为“空闲”。 */
    const detached_reload_strategy_context_t *strategy = context;
    const dart_reload_status_t invalid = {
        DART_RELOAD_FAULT, DART_FAULT_LOADER_UNAVAILABLE, 0U
    };
    return strategy == NULL ? invalid : strategy->status;
}

static const dart_reload_strategy_t detached_ops = {
    .start_prepare = detached_start_prepare,
    .start_recover = detached_start_recover,
    .start_home = detached_start_home,
    .step = detached_step,
    .abort = detached_abort,
    .get_status = detached_get_status,
};

/**
 * @brief 清零拆机策略并绑定当前参数对象。
 *
 * 本函数只建立软件上下文，不访问硬件，也不会启动换弹事务。参数对象由 Dart 运行时持有，
 * 策略仅保存只读指针，因此该对象必须在策略整个使用期间保持有效。
 *
 * @param strategy 待初始化策略上下文。
 * @param parameters 当前参数，必须在策略生命周期内保持有效。
 */
void detached_reload_strategy_setup(detached_reload_strategy_context_t *strategy,
                                    const dart_parameters_t *parameters)
{
    if (strategy == NULL) return;
    memset(strategy, 0, sizeof(*strategy));
    strategy->parameters = parameters;
    strategy->status.result = DART_RELOAD_IDLE;
}

/**
 * @brief 获取拆机换弹策略的操作表。
 *
 * 操作表保存在静态只读存储中，不需要释放。主状态机通过这张表使用拆机策略，从而无需在
 * 业务流程中加入机构是否安装的条件分支。
 *
 * @return 拆机策略操作表的只读地址，程序运行期间始终有效。
 */
const dart_reload_strategy_t *detached_reload_strategy_ops(void)
{
    return &detached_ops;
}
