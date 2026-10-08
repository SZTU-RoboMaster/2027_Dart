#include "dart_runtime.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "dart.h"
#include "dart_build_config.h"
#include "dart_parameters.h"
#include "dart_platform.h"
#include "dart_sm.h"
#if DART_ENABLE_CAROUSEL_LOADER
#include "loader/carousel_lift_loader.h"
#include "loader/dart_reload_strategy.h"
#else
#include "loader/detached_reload_strategy.h"
#endif
#include "operator_interface.h"
#include "topic_bus.h"
#include "../Operate/remote.h"

/*
 * 飞镖主任务运行时
 * ----------------
 * 本文件是业务状态机与 FreeRTOS、主题总线及具体板级适配层之间的唯一装配点。它按照
 * 固定顺序完成“读取输入主题 -> 消费可靠命令 -> 采集反馈 -> 推进状态机 -> 应用输出
 * -> 发布状态与故障”，从而保证每个 1 毫秒周期看到的是一组自洽数据。
 *
 * 运行时本身不实现换弹机械流程。编译开关启用时绑定原转盘升降策略；机构拆除时绑定
 * 安全空实现。两种配置都使用相同状态机接口，因此重新安装或更换机构时不需要改主循环。
 */
extern RC_ctrl_t rc_ctrl;

typedef struct {
    /* 当前芯片的板级操作表，初始化后在任务生命周期内保持不变。 */
    const dart_platform_ops_t *platform;
    /* 当前生效参数与等待安全时机切换的新参数。 */
    dart_parameters_t parameters;
    dart_parameters_t pending_parameters;
    /* 本周期使用的硬件、视觉和裁判输入快照。 */
    dart_feedback_t feedback;
    vision_target_t vision;
    referee_status_t referee;
    /* 四发循环、核心轴回零和故障恢复的唯一业务状态所有者。 */
    dart_sm_t sm;
#if DART_ENABLE_CAROUSEL_LOADER
    /* 只有安装原 DM 转盘与双升降机构时才编译这些上下文。 */
    carousel_lift_loader_t loader;
    carousel_reload_strategy_context_t reload;
#else
    /* 不产生任何换弹硬件输出的安全拆机策略。 */
    detached_reload_strategy_context_t reload;
#endif
    /* 每个输入主题都有独立消费者游标；读者之间不会互相消费或覆盖进度。 */
    topic_subscriber_t vision_subscriber;
    topic_subscriber_t referee_subscriber;
    topic_subscriber_t parameter_subscriber;
    /* 工作状态收到参数更新时先暂存，进入安全状态后再整体替换。 */
    bool parameters_pending;
    /* 防止相同故障在每个控制周期重复写入可靠事件队列。 */
    dart_fault_code_t last_fault;
    /* 遥控器兼容桥接使用的上一帧值，只用于检测边沿。 */
    uint8_t last_left_switch;
    uint8_t last_right_switch;
    int16_t last_wheel;
    /* 用于识别一次完整恢复事务的结束边界。 */
    dart_state_t previous_state;
} dart_runtime_t;

static dart_runtime_t runtime;

/**
 * @brief 把遥控器动作转换为状态机命令并立即交给状态机处理。
 *
 * @param type 命令类型。
 * @param goal 命令携带的目标；与目标无关的命令传入无目标。
 * @param now_ms 命令生成和消费时刻。
 */
static void submit_local_command(dart_command_type_t type, dart_goal_t goal, uint32_t now_ms)
{
    /* 本地遥控器也走状态机命令入口，不允许绕过命令校验直接改状态字段。 */
    dart_command_t command = {
        .sequence = now_ms,
        .timestamp_ms = now_ms,
        .type = type,
        .goal = goal,
    };
    dart_sm_command(&runtime.sm, &command, now_ms);
}

/**
 * @brief 把仍在使用的遥控器输入临时适配为强类型业务命令。
 *
 * 函数只检测拨杆和拨轮边沿，不直接修改状态字段。遥控器数据超过二百毫秒未更新时整帧
 * 忽略，防止断连前最后一个按键被持续执行。
 *
 * @param now_ms 当前控制周期时刻。
 */
static void legacy_operator_bridge(uint32_t now_ms)
{
    /*
     * 临时遥控器适配器：把拨杆边沿转换成未来串口屏也会提交的同一种强类型命令。
     * 遥控器全局数据永远不会直接修改业务状态。
     */
    if ((uint32_t)(now_ms - remote_last_update_ms) > 200U) return;
    uint8_t left = rc_ctrl.rc.s[RC_s_L];
    uint8_t right = rc_ctrl.rc.s[RC_s_R];

    if (right != runtime.last_right_switch && switch_is_up(right)) {
        submit_local_command(DART_COMMAND_SELECT_FRONT, DART_GOAL_FRONT, now_ms);
    } else if (left != runtime.last_left_switch && switch_is_up(left)) {
        submit_local_command(DART_COMMAND_SELECT_BASE, DART_GOAL_BASE, now_ms);
    }

    if (runtime.sm.status.state == DART_STATE_STANDBY &&
        runtime.sm.status.goal != DART_GOAL_NONE &&
        switch_is_mid(left) && switch_is_mid(right)) {
        submit_local_command(DART_COMMAND_START_CYCLE, runtime.sm.status.goal, now_ms);
    }

    if (rc_ctrl.rc.ch[4] < -500 && runtime.last_wheel >= -500) {
        submit_local_command(DART_COMMAND_FIRE, DART_GOAL_NONE, now_ms);
    }

    runtime.last_left_switch = left;
    runtime.last_right_switch = right;
    runtime.last_wheel = rc_ctrl.rc.ch[4];
}

/**
 * @brief 更新旧通信模块仍在读取的只写兼容镜像。
 *
 * 镜像只用于显示或旧协议发送，任何业务决策都不能再从这些全局对象反向写回状态机。
 */
static void update_legacy_observers(void)
{
    /*
     * 裁判和 USB 旧代码仍会读取这两个结构，它们只是只写兼容镜像。发射循环状态和发数
     * 始终只由 dart_sm 拥有。
     */
    dart_goal_set.launcherable_num = runtime.sm.status.shot_index;
    dart_goal_set.dart_goal = runtime.sm.status.goal;
    switch (runtime.sm.status.state) {
        case DART_STATE_STANDBY: gimbal_dart.mode = DART_BACK; break;
        case DART_STATE_MANUAL: gimbal_dart.mode = DART_CONTROL; break;
        case DART_STATE_AIM: gimbal_dart.mode = DART_SCAN; break;
        case DART_STATE_FIRE: gimbal_dart.mode = DART_LAUNCH; break;
        case DART_STATE_FAULT_LATCHED: gimbal_dart.mode = DART_RELAX; break;
        default: gimbal_dart.mode = DART_READY; break;
    }
}

/**
 * @brief 刷新最新值输入并按顺序消费全部可靠命令。
 *
 * 参数更新在工作状态下先暂存，只有进入启动、待机或故障锁定状态才整体替换，避免机构
 * 动作中途使用到一半新参数和一半旧参数。
 *
 * @param now_ms 当前控制周期时刻，用作命令消费时间。
 */
static void refresh_topics(uint32_t now_ms)
{
    /* 最新值输入允许被新快照覆盖，可靠命令事件则必须逐条处理。 */
    /* 视觉和裁判状态只关心最新快照；中间到达的旧样本不会在控制循环中排队回放。 */
    (void)topic_subscriber_read_latest(&runtime.vision_subscriber,
                                       TOPIC_VISION_TARGET,
                                       &runtime.vision);
    (void)topic_subscriber_read_latest(&runtime.referee_subscriber,
                                       TOPIC_REFEREE_STATUS,
                                       &runtime.referee);
    dart_parameters_t candidate;
    if (topic_subscriber_read_latest(&runtime.parameter_subscriber,
                                     TOPIC_DART_PARAMETERS,
                                     &candidate) &&
        dart_parameters_validate(&candidate)) {
        runtime.pending_parameters = candidate;
        runtime.parameters_pending = true;
    }

    /* 换弹和控制模块长期保存参数指针，因此只能在没有业务动作时整块原子替换参数。 */
    if (runtime.parameters_pending &&
        (runtime.sm.status.state == DART_STATE_BOOT ||
         runtime.sm.status.state == DART_STATE_STANDBY ||
         runtime.sm.status.state == DART_STATE_FAULT_LATCHED)) {
        runtime.parameters = runtime.pending_parameters;
        runtime.parameters_pending = false;
    }

    dart_command_t command;
    while (topic_event_take(TOPIC_DART_COMMAND, &command, 0U)) {
        dart_sm_command(&runtime.sm, &command, now_ms);
    }
}

/**
 * @brief 在故障码发生变化时发布一次可靠故障事件。
 *
 * 相同故障保持期间不会每毫秒重复入队；故障清零只更新本地记录，不发布空故障事件。
 *
 * @param now_ms 故障事件发布时间。
 */
static void publish_fault_if_changed(uint32_t now_ms)
{
    /* 每次故障变化只发布一个可靠事件，禁止每毫秒重复灌入同一故障。 */
    if (runtime.sm.status.fault == runtime.last_fault) return;
    runtime.last_fault = runtime.sm.status.fault;
    if (runtime.last_fault == DART_FAULT_NONE) return;
    fault_event_t event = {
        .code = runtime.last_fault,
        .state = runtime.sm.status.state,
        .substate = runtime.sm.status.substate,
        .timestamp_ms = now_ms,
    };
    (void)topic_event_push(TOPIC_FAULT_EVENT, &event, 0U);
}

/**
 * @brief 装配并持续运行完整飞镖控制事务。
 *
 * 主循环顺序固定为：等待周期、采集反馈、刷新输入、处理遥控器、推进状态机、应用目标、
 * 运行控制器、发布状态和故障。恢复完成后才清空旧命令与 PID 历史。函数不会返回。
 *
 * @param argument 任务保留参数，当前实现不使用。
 */
void dart_runtime_task(void const *argument)
{
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(DART_TASK_INIT_TIME));
    memset(&runtime, 0, sizeof(runtime));
    runtime.platform = dart_platform_stm32_get();

    /* 每个最新值输入建立独立消费者游标，后续只接收各自尚未处理的最新快照。 */
    topic_subscriber_init(&runtime.vision_subscriber);
    topic_subscriber_init(&runtime.referee_subscriber);
    topic_subscriber_init(&runtime.parameter_subscriber);

    /* AdjustTask 正常情况下已经发布参数；若它尚未运行或数据损坏，则在此再次兜底。 */
    if (!topic_copy_latest(TOPIC_DART_PARAMETERS, &runtime.parameters) ||
        !dart_parameters_validate(&runtime.parameters)) {
        dart_parameters_defaults(&runtime.parameters);
        (void)topic_publish(TOPIC_DART_PARAMETERS, &runtime.parameters);
    }

    /* 必须先初始化 F427 适配层，再采集第一份电机反馈。 */
    runtime.platform->init();
    runtime.platform->sample_feedback(&runtime.feedback);
#if DART_ENABLE_CAROUSEL_LOADER
    /*
     * 安装机构版本：把具体机构驱动绑定到换弹策略。升降、DM 和舵机门动作全部封装在
     * 这两层接口之后。
     */
    carousel_lift_loader_setup(&runtime.loader,
                               runtime.platform,
                               &runtime.feedback,
                               &runtime.sm.output,
                               &runtime.parameters);
    carousel_reload_strategy_setup(&runtime.reload,
                                   runtime.platform,
                                   carousel_lift_loader_ops(),
                                   &runtime.loader,
                                   &runtime.feedback,
                                   &runtime.sm.output,
                                   &runtime.parameters);
    (void)carousel_lift_loader_ops()->init(&runtime.loader);
    const dart_reload_strategy_t *reload_ops = carousel_reload_strategy_ops();
#else
    /*
     * 默认拆机版本：不调用升降、DM 或舵机门。配置为首发预装时允许第一发继续；真正
     * 需要换弹时明确报告“换弹机构不可用”故障。
     */
    detached_reload_strategy_setup(&runtime.reload, &runtime.parameters);
    const dart_reload_strategy_t *reload_ops = detached_reload_strategy_ops();
#endif
    dart_sm_init(&runtime.sm,
                 runtime.platform,
                 reload_ops,
                 &runtime.reload,
                 &runtime.parameters,
                  runtime.platform->now_ms());
    runtime.previous_state = runtime.sm.status.state;

    runtime.last_left_switch = rc_ctrl.rc.s[RC_s_L];
    runtime.last_right_switch = rc_ctrl.rc.s[RC_s_R];
    runtime.last_wheel = rc_ctrl.rc.ch[4];
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* 每毫秒完成一次确定顺序的完整控制事务。 */
        uint32_t now_ms = runtime.platform->now_ms();
        runtime.platform->sample_feedback(&runtime.feedback);
        refresh_topics(now_ms);
        legacy_operator_bridge(now_ms);
        dart_sm_step(&runtime.sm, &runtime.feedback, &runtime.vision, &runtime.referee, now_ms);
        if ((runtime.previous_state == DART_STATE_RECOVERING ||
             runtime.previous_state == DART_STATE_HOMING) &&
            runtime.sm.status.state != runtime.previous_state) {
            /* 恢复完成是事务边界：只有机械序列真正结束后才丢弃旧命令和控制器历史。 */
            topic_event_clear(TOPIC_DART_COMMAND);
            runtime.platform->reset_control_state();
        }
        runtime.previous_state = runtime.sm.status.state;
        runtime.platform->apply(dart_sm_output(&runtime.sm));
        runtime.platform->service();
        update_legacy_observers();
        (void)topic_publish(TOPIC_DART_FEEDBACK, &runtime.feedback);
        (void)topic_publish(TOPIC_DART_STATUS, dart_sm_status(&runtime.sm));
        publish_fault_if_changed(now_ms);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1U));
    }
}
