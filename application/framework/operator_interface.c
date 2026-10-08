#include "operator_interface.h"

#include "topic_bus.h"

/*
 * 操作员接口只做 Topic Bus 路由，不包含串口帧、页面号或控件 ID。
 * 因此未来更换串口屏协议时，只需新增协议适配器，不需要修改 Dart 状态机。
 */
/**
 * @brief 向 DartTask 提交一条操作员命令。
 *
 * 本函数只把命令复制到可靠事件队列，不在调用者上下文中执行回零、发射或复位动作。
 * 接口采用零等待方式，适合由串口屏解析任务或其他低耦合输入模块调用；队列已满时由
 * 调用者决定是否提示、记录或稍后重试。
 *
 * @param command 待提交的完整命令，函数返回后调用者可立即复用原存储。
 *
 * @return
 * - true：命令已经复制到 Dart 命令队列；
 * - false：参数为空、Topic Bus 尚未初始化或命令队列已满。
 */
bool operator_command_submit(const dart_command_t *command)
{
    /* 0 ms 表示不阻塞调用者；队列满时明确返回 false，由上层决定提示或重试。 */
    return topic_event_push(TOPIC_DART_COMMAND, command, 0U);
}

/**
 * @brief 读取 DartTask 最近发布的整机状态快照。
 *
 * 本函数不会等待下一次状态更新，也不会直接读取状态机内部对象。读取过程由 Topic Bus
 * 互斥保护，因此调用者得到的是同一发布周期内的一致快照。
 *
 * @param status 用于接收状态快照的有效存储地址。
 *
 * @return
 * - true：已经复制一份有效状态；
 * - false：参数为空、Topic Bus 尚未初始化或 DartTask 尚未发布状态。
 */
bool operator_status_read(dart_status_t *status)
{
    /* 返回最近一次由 DartTask 发布的完整状态快照。 */
    return topic_copy_latest(TOPIC_DART_STATUS, status);
}

/**
 * @brief 读取 AdjustTask 最近确认有效的参数快照。
 *
 * 返回的是内存中的当前生效参数，不会在调用过程中访问 Flash。调用者只能修改自己的
 * 副本；需要更新系统参数时，应构造更新事件并调用 `operator_parameters_update()`。
 *
 * @param parameters 用于接收完整参数块的有效存储地址。
 *
 * @return
 * - true：已经复制一份有效参数；
 * - false：参数为空、Topic Bus 尚未初始化或参数尚未发布。
 */
bool operator_parameters_read(dart_parameters_t *parameters)
{
    /* 返回 AdjustTask 最近一次确认有效的完整参数块。 */
    return topic_copy_latest(TOPIC_DART_PARAMETERS, parameters);
}

/**
 * @brief 请求 AdjustTask 校验并保存一组新参数。
 *
 * 本函数不会直接写 Flash，只把完整更新请求复制到参数事件队列。AdjustTask 将串行执行
 * 字段校验、CRC 更新、持久化和重新发布，避免多个输入源并发修改参数存储。
 *
 * @param update 待提交的完整参数更新请求。
 *
 * @return
 * - true：更新请求已经进入参数事件队列；
 * - false：参数为空、Topic Bus 尚未初始化或参数队列已满。
 */
bool operator_parameters_update(const parameter_update_t *update)
{
    /* 更新事件由 AdjustTask 串行校验并保存，接口本身不直接写 Flash。 */
    return topic_event_push(TOPIC_PARAMETER_UPDATE, update, 0U);
}
