#ifndef CAROUSEL_LIFT_LOADER_H
#define CAROUSEL_LIFT_LOADER_H

#include "dart_loader.h"

/*
 * DM6006 转盘与同步升降轴驱动的私有上下文。
 * 调用者先分配该对象并执行 setup，之后只通过 carousel_lift_loader_ops() 操作。驱动
 * 内部没有隐藏的动态内存。
 */
typedef struct {
    /* 由飞镖主任务运行时提供的长期依赖对象。 */
    const dart_platform_ops_t *platform;
    const dart_feedback_t *feedback;
    dart_actuator_command_t *output;
    const dart_parameters_t *parameters;
    dart_loader_action_t action;
    dart_loader_status_t status;
    /* 当前异步动作、发序号和步骤超时记录。 */
    uint8_t shot_index;
    uint32_t step_started_ms;
    uint32_t pair_skew_started_ms;
    bool pair_skew_active;
    /* 两侧限位分别记录，只有左右都成功才允许返回完成。 */
    bool left_homed;
    bool right_homed;
} carousel_lift_loader_t;

/**
 * @brief 装配原转盘与双升降换弹机构的驱动上下文。
 *
 * 本函数只清零私有状态并保存依赖指针，不使能电机、不发送舵机命令，也不执行回零。
 * 所有依赖对象必须在整个 DartTask 生命周期内保持有效。
 *
 * @param loader 待初始化的机构驱动上下文。
 * @param platform 当前板级操作表。
 * @param feedback DartTask 每周期更新的只读反馈快照。
 * @param output 状态机本周期执行器命令，由驱动填写升降和转盘部分。
 * @param parameters 当前有效机构参数。
 */
void carousel_lift_loader_setup(carousel_lift_loader_t *loader,
                                const dart_platform_ops_t *platform,
                                const dart_feedback_t *feedback,
                                dart_actuator_command_t *output,
                                const dart_parameters_t *parameters);
/**
 * @brief 获取原转盘双升降机构的只读操作表。
 * @return 静态操作表地址，程序运行期间始终有效，调用方不得修改。
 */
const dart_loader_ops_t *carousel_lift_loader_ops(void);

#endif
