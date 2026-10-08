#include "Referee.h"

#include <string.h>

#include "CRC8_CRC16.h"
#include "FreeRTOS.h"
#include "dart_platform.h"
#include "decode.h"
#include "queue.h"
#include "topic_bus.h"
#include "../A_Dart/dart.h"

/*
 * 裁判系统接收与飞镖业务数据提取
 * --------------------------------
 * 串口空闲中断只负责冻结本次直接存储器访问接收长度，并把原始字节复制到静态队列；
 * DecodeTask 在任务上下文中完成帧长检查、两级循环冗余校验、命令码分发和主题发布。
 * 这样可以避免在中断中执行大段协议解析，也避免 DartTask 直接读取正在变化的协议结构。
 *
 * 原裁判界面绘图任务与当前飞镖业务无关，已经从本文件删除。未来串口屏应通过
 * operator_interface 提交命令和读取状态，不再向本裁判协议模块添加界面业务。
 */

extern UART_HandleTypeDef huart6;

/* 原始接收区由串口六的循环直接存储器访问持续写入。 */
uint8_t usart6_buf[REFEREE_BUFFER_SIZE];

/* 保留完整协议快照便于调试；飞镖状态机只订阅裁判状态主题。 */
Referee_info_t Referee;
bool launch_grant;
uint8_t dart_launch_mode = 0xFFU;
uint8_t dart_progress_mod = 0x01U;
volatile uint32_t referee_last_update_ms;

typedef struct {
    uint16_t length;                 /* 本次串口空闲中断实际收到的字节数。 */
    uint8_t data[REFEREE_BUFFER_SIZE]; /* 与直接存储器访问区隔离的完整字节副本。 */
} referee_rx_frame_t;

#define REFEREE_RX_QUEUE_DEPTH 3U

/* 队列控制块、数据区和中断暂存帧全部静态分配，不依赖堆。 */
static QueueHandle_t referee_rx_queue;
static StaticQueue_t referee_rx_queue_control;
static uint8_t referee_rx_queue_storage[REFEREE_RX_QUEUE_DEPTH * sizeof(referee_rx_frame_t)];
static referee_rx_frame_t referee_irq_frame;

/* 第二发后识别下一发射窗口所需的历史记录。 */
static uint8_t dart_launch_count;
static bool dart_launch_counted;

static bool referee_read_frame(const uint8_t *frame, uint16_t available_length);
static void update_launch_permission(void);
static void update_launch_window(void);
static void update_door_status(void);

/**
 * @brief 处理裁判串口空闲中断并把本批原始字节送入静态队列。
 *
 * 中断内不做协议校验和业务判断。队列满时丢弃本批数据，避免覆盖尚未解析的旧帧。
 */
void USART6_IRQHandler(void)
{
    if ((USART6->SR & UART_FLAG_IDLE) == 0U) {
        return;
    }

    /* 依次读取状态寄存器和数据寄存器，清除串口空闲中断标志。 */
    __HAL_UART_CLEAR_PEFLAG(&huart6);
    __HAL_DMA_DISABLE(huart6.hdmarx);

    const uint16_t received =
        (uint16_t)(REFEREE_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
    if (referee_rx_queue != NULL && received > 0U) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        referee_irq_frame.length = received;
        memcpy(referee_irq_frame.data, usart6_buf, received);

        /* 队列满时丢弃整批数据，绝不覆盖 DecodeTask 尚未解析的旧帧。 */
        if (xQueueSendFromISR(referee_rx_queue,
                              &referee_irq_frame,
                              &higher_priority_task_woken) == pdTRUE) {
            decode_task_wake_from_isr();
        }
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }

    /* 重新装载接收长度并恢复直接存储器访问，等待下一批串口数据。 */
    __HAL_DMA_CLEAR_FLAG(huart6.hdmarx, DMA_LISR_TCIF1);
    __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_BUFFER_SIZE);
    __HAL_DMA_ENABLE(huart6.hdmarx);
}

/**
 * @brief 初始化裁判接收静态队列和串口接收通道。
 *
 * 函数可重复调用；资源已经存在时直接返回。
 */
void referee_transport_init(void)
{
    /* 初始化函数保持幂等，避免 DecodeTask 重入时重复创建队列或重复启动接收。 */
    if (referee_rx_queue != NULL) {
        return;
    }

    referee_rx_queue = xQueueCreateStatic(REFEREE_RX_QUEUE_DEPTH,
                                          sizeof(referee_rx_frame_t),
                                          referee_rx_queue_storage,
                                          &referee_rx_queue_control);
    const dart_platform_ops_t *platform = dart_platform_stm32_get();
    configASSERT(referee_rx_queue != NULL);
    configASSERT(platform->referee_rx_start != NULL &&
                 platform->referee_rx_start(usart6_buf, REFEREE_BUFFER_SIZE));
}

/* 仅在协议声明的载荷长度足够时复制，防止异常命令长度导致越过当前帧边界。 */
#define COPY_REFEREE_FIELD(field, expected_length)                              \
    do {                                                                        \
        if (payload_length >= (expected_length)) {                              \
            memcpy(&(field), frame + DATA, (expected_length));                 \
        }                                                                       \
    } while (0)

/**
 * @brief 校验并分发一帧完整裁判协议数据。
 *
 * @param frame 候选帧首地址。
 * @param available_length 从该地址开始可安全读取的字节数。
 * @return true 表示帧头、长度和两级校验均有效；false 表示该帧被拒绝。
 */
static bool referee_read_frame(const uint8_t *frame, uint16_t available_length)
{
    const uint16_t minimum_length =
        Referee_LEN_FRAME_HEAD + Referee_LEN_CMD_ID + Referee_LEN_FRAME_TAIL;
    if (frame == NULL || available_length < minimum_length || frame[SOF] != REFREE_HEADER_SOF) {
        return false;
    }
    if (!verify_CRC8_check_sum((uint8_t *)frame, LEN_HEADER)) {
        return false;
    }

    const uint16_t payload_length =
        (uint16_t)frame[DATA_LENGTH] | ((uint16_t)frame[DATA_LENGTH + 1U] << 8U);
    const uint16_t frame_length =
        (uint16_t)(minimum_length + payload_length);
    if (frame_length > available_length ||
        !verify_CRC16_check_sum((uint8_t *)frame, frame_length)) {
        return false;
    }

    const uint16_t command_id = (uint16_t)frame[CMD_ID] |
                                ((uint16_t)frame[CMD_ID + 1U] << 8U);

    /* 受击标志只表示当前解析帧是否为新的受击事件。 */
    Referee.RobotHurt.being_hurt = false;
    switch (command_id) {
        case Referee_ID_game_state:
            COPY_REFEREE_FIELD(Referee.GameState, Referee_LEN_game_state);
            break;
        case Referee_ID_game_result:
            COPY_REFEREE_FIELD(Referee.GameResult, Referee_LEN_game_result);
            Referee.GameResult.game_over = true;
            break;
        case Referee_ID_game_robot_HP:
            COPY_REFEREE_FIELD(Referee.GameRobotHP, Referee_LEN_game_robot_HP);
            break;
        case Referee_ID_event_data:
            COPY_REFEREE_FIELD(Referee.EventData, Referee_LEN_event_data);
            break;
        case Referee_ID_supply_projectile_action:
            COPY_REFEREE_FIELD(Referee.SupplyProjectileAction,
                               Referee_LEN_supply_projectile_action);
            break;
        case Referee_ID_supply_warm:
            COPY_REFEREE_FIELD(Referee.RefereeWarning, Referee_LEN_supply_warm);
            break;
        case Referee_ID_dart_info:
            COPY_REFEREE_FIELD(Referee.DartRemainingTime, Referee_LEN_dart_info);
            break;
        case Referee_ID_game_robot_state:
            COPY_REFEREE_FIELD(Referee.GameRobotStat, Referee_LEN_game_robot_state);
            break;
        case Referee_ID_power_heat_data:
            COPY_REFEREE_FIELD(Referee.PowerHeatData, Referee_LEN_power_heat_data);
            break;
        case Referee_ID_game_robot_pos:
            COPY_REFEREE_FIELD(Referee.GameRobotPos, Referee_LEN_game_robot_pos);
            break;
        case Referee_ID_buff_musk:
            COPY_REFEREE_FIELD(Referee.Buff, Referee_LEN_buff_musk);
            break;
        case Referee_ID_aerial_robot_energy:
            COPY_REFEREE_FIELD(Referee.AerialRobotEnergy, Referee_LEN_aerial_robot_energy);
            break;
        case Referee_ID_robot_hurt:
            COPY_REFEREE_FIELD(Referee.RobotHurt, Referee_LEN_robot_hurt);
            Referee.RobotHurt.being_hurt = true;
            break;
        case Referee_ID_shoot_data:
            COPY_REFEREE_FIELD(Referee.ShootData, Referee_LEN_shoot_data);
            break;
        case Referee_ID_bullet_remaining:
            COPY_REFEREE_FIELD(Referee.BulletRemaining, Referee_LEN_bullet_remaining);
            break;
        case Referee_ID_rfid_status:
            COPY_REFEREE_FIELD(Referee.RfidStatus, Referee_LEN_rfid_status);
            break;
        case Referee_ID_dart_client_directive:
            COPY_REFEREE_FIELD(Referee.DartClient, Referee_LEN_dart_client_directive);
            break;
        case Referee_ID_dart_all_robot_position:
            COPY_REFEREE_FIELD(Referee.RobotPosition, Referee_LEN_dart_all_robot_position);
            break;
        case Referee_ID_radar_mark:
            COPY_REFEREE_FIELD(Referee.RadarMark, Referee_LEN_radar_mark);
            break;
        case Referee_ID_entry_info:
            COPY_REFEREE_FIELD(Referee.SentryInfo, Referee_LEN_entry_info);
            break;
        case Referee_ID_radar_info:
            COPY_REFEREE_FIELD(Referee.RadarInfo, Referee_LEN_radar_info);
            break;
        case Referee_ID_robot_interactive_header_data:
            COPY_REFEREE_FIELD(Referee.StudentInteractive,
                               Referee_LEN_robot_interactive_header_data);
            break;
        case Referee_ID_map_command:
            COPY_REFEREE_FIELD(Referee.MapCommand, Referee_LEN_map_command);
            break;
        case Referee_ID_keyboard_information:
            COPY_REFEREE_FIELD(Referee.keyboard, Referee_LEN_keyboard_information);
            break;
        case Referee_ID_robot_map_robot_data:
            COPY_REFEREE_FIELD(Referee.EnemyPosition, Referee_LEN_robot_map_robot_data);
            break;
        case Referee_ID_robot_custom_client:
            COPY_REFEREE_FIELD(Referee.Custom, Referee_LEN_robot_custom_client);
            break;
        case Referee_ID_robot_entry_info_receive:
            COPY_REFEREE_FIELD(Referee.SentryMapData, Referee_LEN_robot_entry_info_receive);
            break;
        case Referee_ID_robot_custom_info_receive:
            COPY_REFEREE_FIELD(Referee.SendData, Referee_LEN_robot_custom_info_receive);
            break;
        default:
            /* 合法但当前版本未知的命令不影响整批后续帧解析。 */
            break;
    }
    return true;
}

#undef COPY_REFEREE_FIELD

/**
 * @brief 排空并解析当前所有裁判接收批次。
 *
 * 一个接收批次可包含多帧。只要批次中存在合法帧，就更新时间并重新发布裁判状态主题。
 */
void referee_decode_pending(void)
{
    static referee_rx_frame_t frame;
    while (referee_rx_queue != NULL &&
           xQueueReceive(referee_rx_queue, &frame, 0U) == pdTRUE) {
        uint16_t offset = 0U;
        bool batch_contains_valid_frame = false;

        /* 一次串口空闲中断可能包含多帧，按每帧头部声明长度顺序解析。 */
        while ((uint16_t)(frame.length - offset) >=
               (Referee_LEN_FRAME_HEAD + Referee_LEN_CMD_ID + Referee_LEN_FRAME_TAIL)) {
            const uint16_t payload_length =
                (uint16_t)frame.data[offset + DATA_LENGTH] |
                ((uint16_t)frame.data[offset + DATA_LENGTH + 1U] << 8U);
            const uint16_t packet_length =
                (uint16_t)(Referee_LEN_FRAME_HEAD + Referee_LEN_CMD_ID +
                           payload_length + Referee_LEN_FRAME_TAIL);
            if (packet_length > (uint16_t)(frame.length - offset) ||
                packet_length > REFEREE_BUFFER_SIZE) {
                break;
            }
            if (referee_read_frame(&frame.data[offset], packet_length)) {
                batch_contains_valid_frame = true;
            }
            offset = (uint16_t)(offset + packet_length);
        }

        if (!batch_contains_valid_frame) {
            continue;
        }

        referee_last_update_ms = HAL_GetTick();
        update_launch_permission();
        update_launch_window();
        update_door_status();

        /* 只发布状态机真正需要的裁判字段，隔离庞大的协议结构。 */
        const referee_status_t status = {
            .launch_granted = launch_grant,
            .launch_window = dart_launch_mode == 0x01U ? 1U :
                             dart_launch_mode == 0x02U ? 2U : 0U,
            .door_status = dart_progress_mod,
            .timestamp_ms = referee_last_update_ms,
        };
        (void)topic_publish(TOPIC_REFEREE_STATUS, &status);
    }
}

/**
 * @brief 根据比赛阶段、发射口状态和倒计时计算当前发射许可。
 *
 * 任何禁止比赛阶段或末段保护条件都会覆盖先前结果并强制撤销许可。
 */
static void update_launch_permission(void)
{
    /*
     * 只有比赛进行阶段、官方发射口处于允许状态且倒计时位于有效区间时才授权。
     * 阶段剩余时间不足十秒或比赛处于准备、结算等阶段时强制撤销授权。
     */
    launch_grant = Referee.DartClient.dart_launch_opening_status == 0U &&
                   Referee.GameState.game_progress == 4U &&
                   Referee.DartRemainingTime.dart_remaining_time > 0U &&
                   Referee.DartRemainingTime.dart_remaining_time <= 30U;

    if ((Referee.GameState.stage_remain_time < 10U &&
         Referee.GameState.game_progress == 4U) ||
        Referee.GameState.game_progress == 1U ||
        Referee.GameState.game_progress == 2U ||
        Referee.GameState.game_progress == 3U ||
        Referee.GameState.game_progress == 5U) {
        launch_grant = false;
    }
}

/**
 * @brief 根据倒计时和已完成发数识别第一或第二发射窗口。
 *
 * 倒计时末三秒只计数一次，离开许可窗口后才允许下一窗口重新计数。
 */
static void update_launch_window(void)
{
    /* 倒计时末三秒只对当前发射窗口计数一次。 */
    if (Referee.DartRemainingTime.dart_remaining_time <= 3U && launch_grant) {
        if (!dart_launch_counted) {
            dart_launch_count++;
            dart_launch_counted = true;
        }
        return;
    }

    /* 离开发射窗口后允许下一窗口再次计数。 */
    if (!launch_grant) {
        dart_launch_counted = false;
    }

    if (Referee.DartRemainingTime.dart_remaining_time >= 3U &&
        Referee.DartRemainingTime.dart_remaining_time <= 30U &&
        launch_grant) {
        if (dart_goal_set.launcherable_num <= 1U) {
            dart_launch_mode = 0x01U;
        } else if (dart_goal_set.launcherable_num >= 2U && dart_launch_count > 0U) {
            dart_launch_mode = 0x02U;
        }
    } else {
        dart_launch_mode = 0xFFU;
    }
}

/** @brief 把官方发射口原始状态归一化为业务层可用或不可用状态。 */
static void update_door_status(void)
{
    /* 零或二表示官方发射口处于本系统认可的可用状态，其余值统一视为不可用。 */
    const uint8_t opening = Referee.DartClient.dart_launch_opening_status;
    dart_progress_mod = (opening == 0U || opening == 2U) ? 0x01U : 0xFFU;
}
