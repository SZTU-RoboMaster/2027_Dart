#ifndef DETACHED_RELOAD_STRATEGY_H
#define DETACHED_RELOAD_STRATEGY_H

#include "dart_reload_strategy.h"

/*
 * 未安装物理换弹机构时使用的安全替代策略。
 * 该策略不是永远返回成功的假机构：参数明确声明首发已预装时，只允许零号首发通过；
 * 后续发次一旦需要真实换弹，立即报告“换弹机构不可用”。由于它不拥有执行器，机构回零
 * 和机构安全退让会立即完成；主状态机仍继续回零扳机、推板对和 Yaw。
 */
typedef struct {
    const dart_parameters_t *parameters;
    dart_reload_status_t status;
} detached_reload_strategy_context_t;

/**
 * @brief 初始化物理换弹机构拆除时使用的安全策略。
 *
 * 该策略不拥有任何执行器。参数声明首发已预装时只允许第一发准备成功；后续需要真实换弹
 * 的请求会明确返回机构不可用故障，绝不会伪装成换弹完成。
 *
 * @param strategy 待初始化的拆机策略上下文。
 * @param parameters 当前有效参数，生命周期必须长于策略。
 */
void detached_reload_strategy_setup(detached_reload_strategy_context_t *strategy,
                                    const dart_parameters_t *parameters);

/**
 * @brief 获取拆机安全策略的只读操作表。
 * @return 静态操作表地址，程序运行期间始终有效。
 */
const dart_reload_strategy_t *detached_reload_strategy_ops(void);

#endif /* DETACHED_RELOAD_STRATEGY_H */
