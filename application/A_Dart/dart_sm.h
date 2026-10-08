#ifndef DART_SM_H
#define DART_SM_H

#include <stdbool.h>
#include <stdint.h>

#include "dart_reload_strategy.h"

/*
 * 一份 Dart 状态机实例的完整可变上下文。
 * 正式固件由 DartTask 持有一份，仿真环境也可以创建独立实例。所有指针成员都引用
 * 生命周期长于状态机的运行时、平台层或参数对象。
 */
typedef struct {
    /* 外部依赖均由运行时持有，本结构只保存指针，不负责释放。 */
    const dart_platform_ops_t *platform;
    const dart_reload_strategy_t *reload_ops;
    void *reload_context;
    const dart_parameters_t *parameters;
    /* 对外状态快照与本周期执行器输出。 */
    dart_status_t status;
    dart_actuator_command_t output;
    /* 各个短时、非阻塞子状态共用的计时和单次进入标志。 */
    uint32_t substate_started_ms;
    uint32_t aim_stable_started_ms;
    bool aim_stable;            /* 视觉误差已经进入连续稳定计时。 */
    bool action_started;        /* 当前子步骤的一次性启动动作已经执行。 */
    bool fire_requested;        /* 已收到操作员发射命令，等待裁判许可。 */
    bool recovery_from_fault;   /* 当前恢复由故障触发，完成后仍需人工确认。 */
    bool recovery_complete;     /* 机械恢复已完成，可安全清除故障锁存。 */
    /* 左右推板限位回零时使用的成对同步记录。 */
    bool pair_skew_active;
    uint32_t pair_skew_started_ms;
    bool pair_left_homed;
    bool pair_right_homed;
} dart_sm_t;

/**
 * @brief 初始化一份独立的飞镖主状态机上下文。
 *
 * 本函数只绑定长期依赖、清零运行历史并进入 BOOT，不读取反馈、不输出电机命令。首次
 * dart_sm_step 调用才会检查核心轴在线状态并开始上电回零。
 *
 * @param sm 待初始化的状态机对象。
 * @param platform 板级操作表，生命周期必须长于状态机。
 * @param reload_ops 当前换弹策略操作表。
 * @param reload_context 传给换弹策略的私有上下文。
 * @param parameters 当前有效参数，运行期间不能被半结构修改。
 * @param now_ms 初始化时的单调毫秒时刻。
 */
void dart_sm_init(dart_sm_t *sm,
                  const dart_platform_ops_t *platform,
                  const dart_reload_strategy_t *reload_ops,
                  void *reload_context,
                  const dart_parameters_t *parameters,
                  uint32_t now_ms);
/**
 * @brief 处理一条操作员或系统命令。
 *
 * 命令处理不会阻塞。复位和中止只会启动受控恢复，不会直接清空位置与回零状态；非法
 * 状态下的命令会被安全忽略。
 *
 * @param sm 已初始化的状态机对象。
 * @param command 待处理命令，函数不会保存其指针。
 * @param now_ms 命令被状态机消费的单调毫秒时刻。
 */
void dart_sm_command(dart_sm_t *sm, const dart_command_t *command, uint32_t now_ms);

/**
 * @brief 根据一组输入快照推进一次非阻塞状态机周期。
 *
 * 每次调用只写本周期执行器目标、检查真实完成条件并推进必要状态。函数内部不延时、
 * 不轮询外设，预期由 DartTask 每一毫秒调用一次。
 *
 * @param sm 已初始化的状态机对象。
 * @param feedback 本周期板级硬件反馈快照。
 * @param vision 最近一份视觉目标快照。
 * @param referee 最近一份裁判状态快照。
 * @param now_ms 本控制周期的单调毫秒时刻。
 */
void dart_sm_step(dart_sm_t *sm,
                  const dart_feedback_t *feedback,
                  const vision_target_t *vision,
                  const referee_status_t *referee,
                  uint32_t now_ms);
/**
 * @brief 获取状态机当前对外状态的只读地址。
 *
 * 本函数不复制数据，返回地址指向状态机对象内部。调用者应在当前控制周期内读取，不得
 * 修改内容，也不得在状态机对象销毁后继续持有该地址。
 *
 * @param sm 状态机对象。
 *
 * @return 状态地址；内容会在下一次状态机调用时变化，调用方不得修改。
 */
const dart_status_t *dart_sm_status(const dart_sm_t *sm);

/**
 * @brief 获取状态机本周期执行器命令的只读地址。
 *
 * 平台层应在 `dart_sm_step()` 返回后立即读取并应用这份命令。返回对象仍由状态机拥有，
 * 下一控制周期可能被覆盖。
 *
 * @param sm 状态机对象。
 *
 * @return 命令地址；平台层应在当前周期内读取并应用，调用方不得修改。
 */
const dart_actuator_command_t *dart_sm_output(const dart_sm_t *sm);

#endif
