#ifndef TOPIC_BUS_H
#define TOPIC_BUS_H

#include <stdbool.h>
#include <stdint.h>

#include "dart_topics.h"

typedef enum {
    /* 最新值主题：订阅者只需要最近一份完整快照。 */
    TOPIC_VISION_TARGET = 0,
    TOPIC_REFEREE_STATUS,
    TOPIC_DART_FEEDBACK,
    TOPIC_DART_PARAMETERS,
    TOPIC_DART_STATUS,
    /* 事件主题：每个已接收的命令、参数更新和故障都按顺序保留。 */
    TOPIC_DART_COMMAND,
    TOPIC_PARAMETER_UPDATE,
    TOPIC_FAULT_EVENT,
    TOPIC_COUNT
} topic_id_t;

/**
 * @brief 一个最新值主题的独立消费者游标。
 *
 * 订阅者对象属于消费者任务，由消费者自己静态持有。Topic Bus 不为每个订阅者创建任务、
 * 队列或堆对象，只保存该消费者已经处理到的发布代数。多个任务可以分别持有自己的订阅者
 * 对象，并独立读取同一个主题，互不抢数据、互不影响处理进度。
 */
typedef struct {
    uint32_t generation; /* 最近一次成功复制的主题发布代数。 */
    bool initialized;    /* 是否已经成功读取过第一份有效快照。 */
} topic_subscriber_t;

/**
 * @brief 初始化所有最新值主题、可靠事件队列和同步对象。
 *
 * 所有存储均为静态对象。本函数会清除旧有效标志和事件内容，只能在调度器启动前调用，
 * 不应在系统运行中用它代替业务复位。
 */
void topic_bus_init(void);

/**
 * @brief 初始化一个最新值主题消费者。
 *
 * 初始化不会读取主题，也不会阻塞。第一次调用 `topic_subscriber_read_latest()` 时，订阅者
 * 会从当前主题读取一份最新有效快照；如果主题尚未发布数据，则保持未初始化并在后续调用
 * 继续等待第一份有效数据。
 *
 * @param subscriber 消费者独占的订阅对象，必须在后续读取期间保持有效。
 */
void topic_subscriber_init(topic_subscriber_t *subscriber);

/**
 * @brief 读取订阅者尚未处理的最新主题快照。
 *
 * 这是“消费者订阅最新值”接口：如果发布者在两次读取之间发布了多份数据，本函数只复制
 * 最后一份，不会回放中间样本。该语义适用于视觉目标、裁判状态、反馈和参数快照等状态型
 * 数据；复位、发射和故障等不可丢失动作不能使用本接口，应使用可靠事件队列。
 *
 * @param subscriber 消费者自己的订阅对象。
 * @param id 最新值主题编号，不能传入可靠事件主题。
 * @param out 用于接收完整主题快照的输出对象。
 *
 * @return
 * - true：本次复制到了一份该消费者尚未处理的最新快照；
 * - false：当前没有新快照、主题尚未发布，或参数、主题、总线状态无效。
 */
bool topic_subscriber_read_latest(topic_subscriber_t *subscriber,
                                  topic_id_t id,
                                  void *out);

/**
 * @brief 发布一份最新值主题快照。
 *
 * @param id 最新值主题编号，不能传入可靠事件主题编号。
 * @param data 与主题编号严格匹配的数据对象。
 *
 * @return
 * - true：完整数据已复制，主题代数已递增；
 * - false：编号、指针或主题类型无效，或者总线尚未初始化。
 */
bool topic_publish(topic_id_t id, const void *data);

/**
 * @brief 仅在主题代数变化时复制最新值。
 *
 * @param id 最新值主题编号。
 * @param generation 调用方长期保存的代数游标；成功复制后自动更新。
 * @param out 接收完整主题副本的输出缓冲区。
 *
 * @return
 * - true：自上次读取后存在新数据，已完成复制；
 * - false：没有新数据、尚无有效数据或参数无效。
 */
bool topic_copy_if_updated(topic_id_t id, uint32_t *generation, void *out);

/**
 * @brief 复制当前最近一份有效主题快照。
 *
 * @param id 最新值主题编号。
 * @param out 接收数据副本的输出缓冲区。
 *
 * @return true 表示复制成功；false 表示参数无效或该主题尚未发布。
 */
bool topic_copy_latest(topic_id_t id, void *out);

/**
 * @brief 按到达顺序向可靠事件主题写入一个事件。
 *
 * @param id 可靠事件主题编号。
 * @param event 与主题编号匹配的事件对象。
 * @param timeout_ms 队列满时允许等待的毫秒数，零表示立即返回。
 *
 * @return true 表示事件已复制入队；false 表示参数无效或等待后仍无空间。
 */
bool topic_event_push(topic_id_t id, const void *event, uint32_t timeout_ms);

/**
 * @brief 按先进先出顺序取出一个可靠事件。
 *
 * @param id 可靠事件主题编号。
 * @param event 接收事件副本的输出对象。
 * @param timeout_ms 队列为空时允许等待的毫秒数，零表示立即返回。
 *
 * @return true 表示取得一个事件；false 表示参数无效或等待后仍无事件。
 */
bool topic_event_take(topic_id_t id, void *event, uint32_t timeout_ms);

/**
 * @brief 清除指定可靠事件主题中尚未消费的全部旧事件。
 *
 * 只能在恢复完成等明确事务边界调用，避免上一轮发射命令影响下一轮流程。
 *
 * @param id 需要清空的可靠事件主题编号；无效编号会被安全忽略。
 */
void topic_event_clear(topic_id_t id);

#endif
