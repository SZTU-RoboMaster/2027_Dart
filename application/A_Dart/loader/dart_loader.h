#ifndef DART_LOADER_H
#define DART_LOADER_H

#include <stdbool.h>
#include <stdint.h>

#include "dart_platform.h"

typedef enum {
    LOADER_ACTION_INIT = 0,        /* 校验依赖并建立机构内部初态。 */
    LOADER_ACTION_PRESENT_DART,    /* 选择指定弹位并送到推板交接位置。 */
    LOADER_ACTION_TRANSFER_COMPLETE, /* 飞镖已接走，机构退到下一安全位置。 */
    LOADER_ACTION_MAKE_SAFE,       /* 在整机恢复前为推板和回零动作让出空间。 */
    LOADER_ACTION_HOME,            /* 独立完成该换弹机构自身的机械回零。 */
    LOADER_ACTION_ABORT            /* 立即撤销机构拥有的全部输出。 */
} dart_loader_action_t;

typedef enum {
    DART_LOADER_IDLE = 0, /* 当前没有机构动作。 */
    DART_LOADER_BUSY,    /* 动作正在由周期步骤函数推进。 */
    DART_LOADER_DONE,    /* 当前动作已满足全部真实完成条件。 */
    DART_LOADER_FAULT    /* 动作失败，故障码和失败步骤保持可读。 */
} dart_loader_result_t;

typedef struct {
    dart_loader_result_t result; /* 当前异步动作结果。 */
    dart_fault_code_t fault;     /* 失败原因，无故障时为零。 */
    uint8_t step;                /* 当前物理步骤号，故障后不自动清除。 */
} dart_loader_status_t;

typedef struct {
    /**
     * @brief 校验依赖并初始化机构私有上下文。
     *
     * 回调不得阻塞、不得创建任务，也不得在初始化阶段启动机械运动。
     *
     * @param context 具体机构的私有上下文。
     * @return true 表示初始化成功；false 表示上下文或依赖无效。
     */
    bool (*init)(void *context);
    /**
     * @brief 启动一个非阻塞机构动作。
     *
     * 同一时刻只能存在一个运行中动作。返回成功后，调用者必须周期调用 `step` 并通过
     * `get_status` 判断最终完成或故障。
     *
     * @param context 具体机构的私有上下文。
     * @param action 要启动的机构动作。
     * @param shot_index 当前零基发序号。
     * @return true 表示请求已接受；false 表示参数无效或机构正忙。
     */
    bool (*start)(void *context, dart_loader_action_t action, uint8_t shot_index);
    /**
     * @brief 推进当前机构动作一个控制周期。
     * @param context 具体机构的私有上下文。
     * @param now_ms 当前单调毫秒时刻。
     */
    void (*step)(void *context, uint32_t now_ms);
    /**
     * @brief 立即撤销机构拥有的全部运动命令。
     * @param context 具体机构的私有上下文。
     */
    void (*abort)(void *context);
    /**
     * @brief 按值读取当前动作状态快照。
     * @param context 具体机构的私有上下文。
     * @return 当前结果、故障码和物理步骤号组成的状态副本。
     */
    dart_loader_status_t (*get_status)(void *context);
} dart_loader_ops_t;

#endif
