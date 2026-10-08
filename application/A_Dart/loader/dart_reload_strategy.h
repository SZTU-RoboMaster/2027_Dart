#ifndef DART_RELOAD_STRATEGY_H
#define DART_RELOAD_STRATEGY_H

#include <stdbool.h>
#include <stdint.h>

#include "dart_loader.h"

typedef enum {
    DART_RELOAD_IDLE = 0, /* 当前没有换弹事务。 */
    DART_RELOAD_BUSY,    /* 策略正在协调推板、门和下层机构。 */
    DART_RELOAD_DONE,    /* 本次准备、恢复或回零事务完整结束。 */
    DART_RELOAD_FAULT    /* 任一步失败，后续步骤禁止继续。 */
} dart_reload_result_t;

typedef struct {
    dart_reload_result_t result; /* 当前事务结果。 */
    dart_fault_code_t fault;     /* 故障原因。 */
    uint8_t step;                /* 当前步骤号，也是故障定位依据。 */
} dart_reload_status_t;

typedef struct {
    /**
     * @brief 启动指定发次的飞镖准备事务。
     * @param context 具体策略的私有上下文。
     * @param shot_index 待准备飞镖的零基发序号。
     * @return true 表示事务已接受；false 表示参数无效或策略正忙。
     */
    bool (*start_prepare)(void *context, uint8_t shot_index);
    /**
     * @brief 启动换弹机构安全退让事务。
     * @param context 具体策略的私有上下文。
     * @return true 表示事务已接受；false 表示上下文无效或策略正忙。
     */
    bool (*start_recover)(void *context);
    /**
     * @brief 启动换弹机构完整回零事务。
     * @param context 具体策略的私有上下文。
     * @return true 表示事务已接受；false 表示上下文无效或策略正忙。
     */
    bool (*start_home)(void *context);
    /**
     * @brief 推进当前换弹事务一个控制周期。
     * @param context 具体策略的私有上下文。
     * @param now_ms 当前单调毫秒时刻。
     */
    void (*step)(void *context, uint32_t now_ms);
    /**
     * @brief 中止策略及其下层机构，并撤销策略拥有的输出。
     * @param context 具体策略的私有上下文。
     */
    void (*abort)(void *context);
    /**
     * @brief 按值读取当前换弹事务状态。
     * @param context 具体策略的私有上下文。
     * @return 当前结果、故障码和事务步骤号组成的状态副本。
     */
    dart_reload_status_t (*get_status)(void *context);
} dart_reload_strategy_t;

typedef struct {
    /* 生命周期覆盖整个任务的依赖对象；策略不拥有这些对象的存储。 */
    const dart_platform_ops_t *platform;
    const dart_loader_ops_t *loader_ops;
    void *loader_context;
    const dart_feedback_t *feedback;
    dart_actuator_command_t *output;
    const dart_parameters_t *parameters;
    dart_reload_status_t status;
    /* 每次启动新事务时都会重置的步骤、计时和动作标志。 */
    uint8_t shot_index;            /* 当前需要准备或恢复的零基发序号。 */
    uint32_t step_started_ms;       /* 当前步骤的独立超时起点。 */
    bool loader_action_started;     /* 防止每毫秒重复启动同一下层动作。 */
    bool recovering;               /* 当前事务是否为工作中受控恢复。 */
    bool home_only;                 /* 当前事务是否只执行换弹机构回零。 */
    uint8_t active_gate_index;      /* 当前发次对应的舵机门编号。 */
} carousel_reload_strategy_context_t;

/**
 * @brief 装配原换弹机构对应的完整换弹策略上下文。
 *
 * 本函数把推板、舵机门和下层转盘升降驱动连接起来，只初始化内存状态，不启动任何动作。
 * 所有依赖对象必须由 DartTask 运行时长期持有。
 *
 * @param strategy 待初始化的策略上下文。
 * @param platform 当前板级操作表。
 * @param loader_ops 下层换弹机构操作表。
 * @param loader_context 下层换弹机构私有上下文。
 * @param feedback 每周期更新的硬件反馈快照。
 * @param output 状态机执行器命令，策略负责填写推板和机构相关字段。
 * @param parameters 当前有效参数。
 */
void carousel_reload_strategy_setup(carousel_reload_strategy_context_t *strategy,
                                    const dart_platform_ops_t *platform,
                                    const dart_loader_ops_t *loader_ops,
                                    void *loader_context,
                                    const dart_feedback_t *feedback,
                                    dart_actuator_command_t *output,
                                    const dart_parameters_t *parameters);
/**
 * @brief 获取原机构换弹策略的只读操作表。
 * @return 静态操作表地址，程序运行期间始终有效。
 */
const dart_reload_strategy_t *carousel_reload_strategy_ops(void);

#endif
