#ifndef DART_TOPICS_H
#define DART_TOPICS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 与通信协议无关的共享数据模型
 * ----------------------------
 * 本文件中的结构通过主题总线跨任务、跨模块传递，不包含硬件句柄、指针或变长数组。因此
 * 同一套定义可以用于 STM32 固件、仿真环境和未来的串口屏协议适配器。
 */
#define DART_SHOT_COUNT             4U
#define DART_LOADER_POSITION_COUNT  7U
#define DART_PARAMETER_MAGIC        0x44525450UL
#define DART_PARAMETER_VERSION      2U
#define DART_LOADER_TYPE_CAROUSEL_LIFT 1U

typedef enum {
    /* 发射核心轴和可选换弹轴共用固定长度向量，顺序属于模块间接口。 */
    DART_AXIS_TRIGGER = 0, /* 扳机直线轴。 */
    DART_AXIS_PUSH_LEFT,  /* 左侧推板轴。 */
    DART_AXIS_PUSH_RIGHT, /* 右侧推板轴，目标方向与左侧相反。 */
    DART_AXIS_LIFT_LEFT,  /* 当前可选换弹机构的左升降轴。 */
    DART_AXIS_LIFT_RIGHT, /* 当前可选换弹机构的右升降轴。 */
    DART_AXIS_YAW,        /* 发射架水平瞄准轴。 */
    DART_AXIS_COUNT       /* 轴数量，只用于数组长度，不能作为有效轴号。 */
} dart_axis_t;

typedef enum {
    /* 对外可观察的顶层生命周期；更细的子状态只由所属模块解释。 */
    DART_STATE_BOOT = 0,       /* 上电初始化，等待首批核心轴反馈。 */
    DART_STATE_HOMING,         /* 按固定顺序执行整机自动回零。 */
    DART_STATE_STANDBY,        /* 已回零安全待机，可选择目标并开始新一轮。 */
    DART_STATE_MANUAL,         /* 预留的人工维护模式，当前只保证发射关闭。 */
    DART_STATE_PREPARE,        /* 校验目标和发序号，准备启动换弹事务。 */
    DART_STATE_RELOAD,         /* 由当前换弹策略准备指定序号的飞镖。 */
    DART_STATE_AIM,            /* 扳机和水平轴到位，并等待视觉稳定。 */
    DART_STATE_WAIT_LAUNCH,    /* 等待操作员请求与裁判系统许可同时满足。 */
    DART_STATE_FIRE,           /* 保持发射机构开启，完整结束后才累计发数。 */
    DART_STATE_POST_SHOT,      /* 根据已完成发数决定下一条业务分支。 */
    DART_STATE_WAIT_NEXT_WINDOW, /* 第二发后等待下一次裁判发射窗口。 */
    DART_STATE_CYCLE_COMPLETE, /* 第四发完成，一律转入完整恢复。 */
    DART_STATE_RECOVERING,     /* 按严格步骤恢复机构并重新回零。 */
    DART_STATE_FAULT_LATCHED   /* 输出保持安全，等待恢复重试或人工确认。 */
} dart_state_t;

typedef enum {
    /* 命令属于可靠事件，必须严格按照到达顺序处理。 */
    DART_COMMAND_NONE = 0,       /* 空命令，不产生状态变化。 */
    DART_COMMAND_START_CYCLE,    /* 从安全待机开始一轮四发流程。 */
    DART_COMMAND_FIRE,           /* 记录一次人工发射请求。 */
    DART_COMMAND_ABORT,          /* 中止当前动作并进入受控恢复。 */
    DART_COMMAND_RESET,          /* 执行整机业务复位，不清除标定参数。 */
    DART_COMMAND_CONFIRM_FAULT,  /* 在机械恢复完成后解除故障锁存。 */
    DART_COMMAND_RETRY_RECOVERY, /* 从失败步骤重新发起一整套恢复。 */
    DART_COMMAND_ENTER_MANUAL,   /* 从待机进入人工维护模式。 */
    DART_COMMAND_EXIT_MANUAL,    /* 退出人工模式并执行安全恢复。 */
    DART_COMMAND_SELECT_FRONT,   /* 选择前哨站目标。 */
    DART_COMMAND_SELECT_BASE     /* 选择基地目标。 */
} dart_command_type_t;

typedef enum {
    DART_FAULT_NONE = 0,             /* 当前没有故障。 */
    DART_FAULT_COMMAND_QUEUE_OVERFLOW, /* 可靠命令队列已满，存在命令丢失风险。 */
    DART_FAULT_FEEDBACK_STALE,       /* 核心电机反馈离线或超出有效时限。 */
    DART_FAULT_VISION_STALE,         /* 视觉数据超过有效时限。 */
    DART_FAULT_REFEREE_STALE,        /* 裁判数据超过有效时限。 */
    DART_FAULT_ACTION_TIMEOUT,       /* 当前动作未在参数规定时间内完成。 */
    DART_FAULT_PUSH_SYNC,            /* 左右推板的位置镜像误差过大。 */
    DART_FAULT_LIFT_SYNC,            /* 左右升降轴的位置差或限位时间差过大。 */
    DART_FAULT_LOADER,               /* 换弹驱动或换弹策略报告一般故障。 */
    DART_FAULT_LOADER_UNAVAILABLE,   /* 机构已拆除，但流程请求了真实换弹。 */
    DART_FAULT_HOME_TRIGGER,         /* 扳机轴回零失败。 */
    DART_FAULT_HOME_PUSH,            /* 左右推板成对回零失败。 */
    DART_FAULT_HOME_LIFT,            /* 左右升降轴成对回零失败。 */
    DART_FAULT_HOME_YAW,             /* 水平轴回零失败。 */
    DART_FAULT_PARAMETER_INVALID     /* 参数版本、范围或校验值无效。 */
} dart_fault_code_t;

typedef enum {
    DART_GOAL_NONE = 0, /* 尚未选择有效目标。 */
    DART_GOAL_FRONT,    /* 前哨站目标。 */
    DART_GOAL_BASE      /* 基地目标。 */
} dart_goal_t;

typedef struct {
    uint32_t sequence;          /* 发送方递增序号，用于诊断重复或乱序。 */
    uint32_t timestamp_ms;      /* 命令生成时刻，不是状态机消费时刻。 */
    dart_command_type_t type;   /* 命令类型。 */
    dart_goal_t goal;           /* 仅目标选择或启动命令使用。 */
} dart_command_t;

typedef struct {
    float yaw_error;       /* 水平瞄准误差，正负方向由视觉协议统一约定。 */
    bool target_locked;    /* 视觉是否确认当前帧存在可信目标。 */
    uint32_t timestamp_ms; /* 原始视觉帧到达时间，用于判断数据是否过期。 */
} vision_target_t;

typedef struct {
    bool launch_granted;   /* 裁判系统当前是否允许发射。 */
    uint8_t launch_window; /* 裁判发射窗口编号，用于识别下一窗口。 */
    uint8_t door_status;   /* 官方协议中的闸门状态原始值。 */
    uint32_t timestamp_ms; /* 原始裁判帧到达时间，用于判断数据是否过期。 */
} referee_status_t;

typedef struct {
    float position[DART_AXIS_COUNT]; /* 各轴换算后的业务位置，单位由轴定义约定。 */
    float speed[DART_AXIS_COUNT];    /* 各轴换算后的业务速度。 */
    bool limit[DART_AXIS_COUNT];     /* 各轴机械回零限位的实时电平。 */
    bool online[DART_AXIS_COUNT];    /* 各轴反馈是否处于允许的更新时间内。 */
    float loader_turn_position;      /* 当前可选转盘的位置反馈。 */
    bool loader_turn_online;         /* 当前可选转盘反馈是否有效。 */
    uint32_t timestamp_ms;           /* 整份板级反馈快照的采样时间。 */
} dart_feedback_t;

typedef struct {
    /* 可持久化、带版本号并由 CRC 整体保护的标定与策略参数块。 */
    uint32_t magic;        /* 参数块固定标识，排除空白存储和其他数据。 */
    uint16_t version;      /* 参数结构版本，布局变化时必须递增。 */
    uint16_t size;         /* 写入存储时的结构总字节数。 */
    uint16_t loader_type;  /* 参数所对应的换弹机构类型。 */
    uint16_t format_reserved; /* 为后续格式扩展预留并保持四字节对齐。 */
    float trigger_distance[3][DART_SHOT_COUNT]; /* 三类目标下四发各自的扳机标定位置。 */
    float yaw_position[3]; /* 三类目标的水平轴基础位置。 */
    float loader_turn_position[DART_LOADER_POSITION_COUNT]; /* 原转盘各弹位角度。 */
    float loader_lift_position;   /* 原双升降机构的交接高度。 */
    float push_load_position;     /* 推板下降到接弹位置的目标值。 */
    float push_exchange_position; /* 推板与换弹机构开始交接的位置。 */
    float push_back_position;     /* 推板安全后位。 */
    float push_sync_tolerance;    /* 左右推板允许的最大镜像误差。 */
    float lift_sync_tolerance;    /* 左右升降轴允许的最大位置差。 */
    uint32_t action_timeout_ms;   /* 普通机械动作统一超时时间。 */
    uint32_t homing_timeout_ms;   /* 每个回零步骤的最大允许时间。 */
    uint32_t aim_timeout_ms;      /* 一发瞄准的最大等待时间。 */
    uint32_t launch_wait_timeout_ms; /* 等待本次发射许可的最大时间。 */
    uint32_t next_window_timeout_ms; /* 第二发后等待下一窗口的最大时间。 */
    uint32_t fire_hold_ms;        /* 发射机构保持开启的时间。 */
    bool initial_dart_preloaded;  /* 上电前是否已由人工预装第一发。 */
    uint8_t reserved[3];          /* 保持结构对齐，并为布尔配置扩展预留。 */
    uint32_t crc32;               /* 除本字段外整个参数块的循环冗余校验值。 */
} dart_parameters_t;

typedef struct {
    /* 面向界面、遥测和故障诊断的精简状态快照。 */
    dart_state_t state;       /* 当前顶层业务状态。 */
    uint8_t substate;         /* 当前状态内的物理步骤号，故障时保留现场。 */
    uint8_t shot_index;       /* 零基发序号，只在完整发射后递增。 */
    dart_goal_t goal;         /* 当前一轮锁定的目标。 */
    dart_fault_code_t fault;  /* 当前锁存故障码。 */
    bool homed;               /* 核心机构是否已经完整回零。 */
    bool launch_permitted;    /* 本周期视觉与裁判条件是否共同允许发射。 */
    bool recovery_required_confirmation; /* 恢复完成后是否仍需人工确认故障。 */
    uint32_t state_entered_ms; /* 进入当前顶层状态的时刻。 */
} dart_status_t;

typedef struct {
    dart_fault_code_t code; /* 故障原因。 */
    dart_state_t state;     /* 故障发生时的顶层状态。 */
    uint8_t substate;       /* 故障发生时的物理步骤号。 */
    uint32_t timestamp_ms;  /* 故障首次锁存时刻。 */
} fault_event_t;

typedef struct {
    dart_parameters_t parameters; /* 已完成格式校验并准备保存的完整参数。 */
    uint32_t timestamp_ms;        /* 操作端提交更新时间。 */
} parameter_update_t;

typedef enum {
    DART_AXIS_MODE_DISABLED = 0, /* 该轴本周期停止输出。 */
    DART_AXIS_MODE_POSITION,     /* 该轴闭环跟踪位置目标。 */
    DART_AXIS_MODE_SPEED         /* 该轴闭环跟踪速度目标，主要用于找限位。 */
} dart_axis_mode_t;

typedef struct {
    /* 状态机每次运行产生的一份与具体电机型号无关的执行器命令。 */
    dart_axis_mode_t mode[DART_AXIS_COUNT];
    float target[DART_AXIS_COUNT]; /* 与各轴模式对应的位置或速度目标。 */
    float loader_turn_target;      /* 当前可选转盘的位置目标。 */
    bool loader_turn_enabled;      /* 是否允许当前可选转盘输出。 */
    bool launcher_open;            /* 是否打开实际发射机构。 */
} dart_actuator_command_t;

#endif
