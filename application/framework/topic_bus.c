#include "topic_bus.h"

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/*
 * 轻量、无动态分配的强类型主题总线
 * --------------------------------
 * 最新值结构通常大于单个处理器原子字，因此使用一个互斥量保护完整复制。可靠事件使用
 * FreeRTOS 静态队列，确保复位和中止命令不会被后来的值覆盖。主题编号在编译期固定；
 * 新增主题时必须在本文件显式绑定准确大小的存储，从而让 RAM 成本始终可见、可审查。
 */
#define TOPIC_EVENT_DEPTH 8U
/* 事件队列的单元素容量必须覆盖所有可靠事件类型，后续新增事件也要同步检查此处。 */
#define MAX_EVENT_SIZE ((sizeof(parameter_update_t) > sizeof(fault_event_t)) ? \
                        ((sizeof(parameter_update_t) > sizeof(dart_command_t)) ? \
                         sizeof(parameter_update_t) : sizeof(dart_command_t)) : \
                        ((sizeof(fault_event_t) > sizeof(dart_command_t)) ? \
                         sizeof(fault_event_t) : sizeof(dart_command_t)))

typedef struct {
    /* 对外数据实际存放在下方具有明确类型的静态对象中。 */
    void *storage;
    uint16_t size;
    uint32_t generation;
    bool valid;
} latest_topic_t;

typedef struct {
    /* 每种事件分别拥有静态队列控制块和固定字节存储。 */
    QueueHandle_t handle;
    StaticQueue_t control;
    uint8_t storage[TOPIC_EVENT_DEPTH * MAX_EVENT_SIZE];
    uint16_t item_size;
} event_topic_t;

static vision_target_t vision_target_storage;
static referee_status_t referee_status_storage;
static dart_feedback_t dart_feedback_storage;
static dart_parameters_t dart_parameters_storage;
static dart_status_t dart_status_storage;

static latest_topic_t latest_topics[TOPIC_COUNT];
static event_topic_t command_events;
static event_topic_t parameter_events;
static event_topic_t fault_events;
static SemaphoreHandle_t latest_lock;
static StaticSemaphore_t latest_lock_storage;

/**
 * @brief 根据主题编号取得可靠事件队列对象。
 *
 * 仅命令、参数更新和故障事件拥有队列。最新值主题传入本函数时返回空指针，防止调用者
 * 误用事件接口破坏主题语义。
 *
 * @param id 待查找的主题编号。
 *
 * @return 对应静态事件对象地址；不是事件主题时返回空指针。
 */
static event_topic_t *event_topic(topic_id_t id)
{
    /* 拒绝把最新值主题误当成事件队列使用。 */
    switch (id) {
        case TOPIC_DART_COMMAND: return &command_events;
        case TOPIC_PARAMETER_UPDATE: return &parameter_events;
        case TOPIC_FAULT_EVENT: return &fault_events;
        default: return NULL;
    }
}

/**
 * @brief 把一个最新值主题绑定到固定类型的静态存储。
 *
 * 绑定时同步清除有效标志和发布代数，保证初始化后只有真正发布过的数据才能被读取。
 * 本函数只应由 `topic_bus_init()` 在调度器启动前调用。
 *
 * @param id 待绑定的最新值主题编号。
 * @param storage 该主题对应的静态数据对象地址。
 * @param size 单个主题样本的字节数。
 */
static void latest_bind(topic_id_t id, void *storage, uint16_t size)
{
    latest_topics[id].storage = storage;
    latest_topics[id].size = size;
    latest_topics[id].generation = 0U;
    latest_topics[id].valid = false;
}

/**
 * @brief 使用预分配内存创建一个静态事件队列。
 *
 * 队列深度由 `TOPIC_EVENT_DEPTH` 固定，创建过程不申请堆内存。不同事件主题可使用不同
 * 元素大小，但都不能超过事件存储区按最大事件结构预留的容量。
 *
 * @param topic 待初始化的事件主题对象。
 * @param item_size 每个队列元素的准确字节数。
 */
static void event_bind(event_topic_t *topic, uint16_t item_size)
{
    topic->item_size = item_size;
    topic->handle = xQueueCreateStatic(TOPIC_EVENT_DEPTH,
                                       item_size,
                                       topic->storage,
                                       &topic->control);
}

/**
 * @brief 初始化全部最新值主题、事件队列和复制互斥量。
 *
 * 本函数应在 FreeRTOS 调度器启动前调用一次。它会清除所有最新值主题的有效状态，并
 * 重新创建基于静态内存的互斥量和事件队列。运行过程中不得调用，否则会清除尚未处理的
 * 命令、参数更新和故障事件。
 */
void topic_bus_init(void)
{
    /* 只允许在调度器启动前初始化或重新初始化。 */
    memset(latest_topics, 0, sizeof(latest_topics));
    latest_lock = xSemaphoreCreateMutexStatic(&latest_lock_storage);

    latest_bind(TOPIC_VISION_TARGET, &vision_target_storage, sizeof(vision_target_storage));
    latest_bind(TOPIC_REFEREE_STATUS, &referee_status_storage, sizeof(referee_status_storage));
    latest_bind(TOPIC_DART_FEEDBACK, &dart_feedback_storage, sizeof(dart_feedback_storage));
    latest_bind(TOPIC_DART_PARAMETERS, &dart_parameters_storage, sizeof(dart_parameters_storage));
    latest_bind(TOPIC_DART_STATUS, &dart_status_storage, sizeof(dart_status_storage));

    event_bind(&command_events, sizeof(dart_command_t));
    event_bind(&parameter_events, sizeof(parameter_update_t));
    event_bind(&fault_events, sizeof(fault_event_t));
}

/**
 * @brief 初始化一个最新值主题消费者的本地游标。
 *
 * 订阅者状态完全由消费者保存，Topic Bus 不登记订阅者列表，因此不会引入广播遍历、动态
 * 注册或额外任务开销。初始化后第一次读取会主动寻找当前已经发布的有效快照。
 *
 * @param subscriber 消费者独占的订阅对象；传入空指针时安全返回。
 */
void topic_subscriber_init(topic_subscriber_t *subscriber)
{
    if (subscriber == NULL) {
        return;
    }
    subscriber->generation = 0U;
    subscriber->initialized = false;
}

/**
 * @brief 复制指定消费者尚未处理的最新主题快照。
 *
 * 首次读取使用内部特殊游标获取当前最新有效样本，之后使用订阅者自己的 generation 判断
 * 是否出现新发布。多个消费者读取同一主题时各自维护游标，因此一个消费者读取不会消耗
 * 另一个消费者的数据。
 *
 * @param subscriber 消费者本地订阅对象。
 * @param id 最新值主题编号。
 * @param out 用于接收主题快照的输出对象。
 *
 * @return
 * - true：已经复制一份新的最新快照；
 * - false：没有新数据、主题尚未发布，或输入参数无效。
 */
bool topic_subscriber_read_latest(topic_subscriber_t *subscriber,
                                  topic_id_t id,
                                  void *out)
{
    if (subscriber == NULL || out == NULL) {
        return false;
    }

    /* 总线初始化时发布代数从零开始，第一次有效发布会递增到一；从零开始即可获取当前值。 */
    if (!subscriber->initialized) {
        if (!topic_copy_if_updated(id, &subscriber->generation, out)) {
            return false;
        }
        subscriber->initialized = true;
        return true;
    }

    /* 后续读取只接受代数变化，多个消费者之间不会互相清除数据。 */
    return topic_copy_if_updated(id, &subscriber->generation, out);
}

/**
 * @brief 原子发布一个最新值主题样本。
 *
 * 数据复制、发布代数递增和有效标志更新在同一个互斥区完成，读取者不会看到一半新数据、
 * 一半旧数据。该接口不适用于命令等可靠事件主题。
 *
 * @param id 待发布的最新值主题编号。
 * @param data 与该主题类型和尺寸完全匹配的数据地址。
 *
 * @return
 * - true：样本已经完整复制并发布；
 * - false：编号或数据无效、主题未绑定，或互斥量尚未初始化。
 */
bool topic_publish(topic_id_t id, const void *data)
{
    /* 数据、有效标志和代数必须在同一个临界区内完成更新。 */
    if (id >= TOPIC_COUNT || data == NULL || latest_topics[id].storage == NULL || latest_lock == NULL) {
        return false;
    }
    if (xSemaphoreTake(latest_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    memcpy(latest_topics[id].storage, data, latest_topics[id].size);
    latest_topics[id].generation++;
    latest_topics[id].valid = true;
    xSemaphoreGive(latest_lock);
    return true;
}

/**
 * @brief 仅在主题出现新代数时复制最新样本。
 *
 * 调用者保存自己的 `generation` 游标。复制成功后本函数会把游标更新为当前发布代数；
 * 若没有新数据则保持输出缓冲区和游标不变。该方式适合周期任务避免重复处理同一快照。
 *
 * @param id 待读取的最新值主题编号。
 * @param generation 调用者持有的代数游标，成功复制时会被更新。
 * @param out 用于接收完整主题样本的存储地址。
 *
 * @return
 * - true：存在新代数，样本和游标均已更新；
 * - false：没有新样本，或参数、主题、互斥量无效。
 */
bool topic_copy_if_updated(topic_id_t id, uint32_t *generation, void *out)
{
    bool updated = false;
    if (id >= TOPIC_COUNT || generation == NULL || out == NULL ||
        latest_topics[id].storage == NULL || latest_lock == NULL) {
        return false;
    }
    if (xSemaphoreTake(latest_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    if (latest_topics[id].valid && latest_topics[id].generation != *generation) {
        memcpy(out, latest_topics[id].storage, latest_topics[id].size);
        *generation = latest_topics[id].generation;
        updated = true;
    }
    xSemaphoreGive(latest_lock);
    return updated;
}

/**
 * @brief 无条件读取一个主题当前最近的有效样本。
 *
 * 本函数不要求调用者维护代数，也不会等待未来发布。若主题从未发布过有效数据，则返回
 * false 并保持输出内容不变。
 *
 * @param id 待读取的最新值主题编号。
 * @param out 用于接收完整样本的存储地址。
 *
 * @return
 * - true：已经复制当前最新有效样本；
 * - false：主题尚无数据，或参数、主题、互斥量无效。
 */
bool topic_copy_latest(topic_id_t id, void *out)
{
    /* 代数从零开始，因此以 UINT32_MAX 作为初值一定能复制首个有效样本。 */
    uint32_t ignored_generation = UINT32_MAX;
    return topic_copy_if_updated(id, &ignored_generation, out);
}

/**
 * @brief 把一条可靠事件复制到指定事件队列尾部。
 *
 * 与最新值发布不同，已入队事件不会被后续事件覆盖，适合复位、确认和中止等必须逐条
 * 处理的命令。等待时间由调用者决定；中断上下文不得调用本接口。
 *
 * @param id 可靠事件主题编号。
 * @param event 与该事件主题类型完全匹配的数据地址。
 * @param timeout_ms 队列满时允许等待的毫秒数，0 表示立即返回。
 *
 * @return
 * - true：事件已经完整复制到队列；
 * - false：编号或数据无效、队列未初始化，或等待后仍无空位。
 */
bool topic_event_push(topic_id_t id, const void *event, uint32_t timeout_ms)
{
    event_topic_t *topic = event_topic(id);
    if (topic == NULL || topic->handle == NULL || event == NULL) {
        return false;
    }
    return xQueueSend(topic->handle, event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

/**
 * @brief 从指定可靠事件队列取出最早的一条事件。
 *
 * 读取成功后该事件即从队列移除，保证事件只被消费一次。等待期间当前任务可能进入阻塞，
 * 因此 1 毫秒控制循环通常应使用零等待。
 *
 * @param id 可靠事件主题编号。
 * @param event 用于接收事件副本的存储地址。
 * @param timeout_ms 队列为空时允许等待的毫秒数，0 表示立即返回。
 *
 * @return
 * - true：成功取出一条事件；
 * - false：编号或输出地址无效、队列未初始化，或等待后仍为空。
 */
bool topic_event_take(topic_id_t id, void *event, uint32_t timeout_ms)
{
    event_topic_t *topic = event_topic(id);
    if (topic == NULL || topic->handle == NULL || event == NULL) {
        return false;
    }
    return xQueueReceive(topic->handle, event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

/**
 * @brief 丢弃指定可靠事件主题中所有尚未消费的事件。
 *
 * 该操作主要用于完整业务复位后清除旧命令，避免恢复到待机后立即执行复位前残留请求。
 * 调用者必须确保清空行为符合业务语义；传入最新值主题或未初始化队列时不会产生动作。
 *
 * @param id 需要清空的可靠事件主题编号。
 */
void topic_event_clear(topic_id_t id)
{
    event_topic_t *topic = event_topic(id);
    if (topic != NULL && topic->handle != NULL) {
        xQueueReset(topic->handle);
    }
}
