#ifndef OPERATOR_INTERFACE_H
#define OPERATOR_INTERFACE_H

#include <stdbool.h>

#include "dart_topics.h"

/*
 * 与具体协议无关的操作员接口，预留给后续串口屏。
 *
 * 串口屏解析器只能调用这些函数，不允许直接修改 Dart 全局变量、任务句柄或状态机：
 * - 命令和参数更新使用可靠事件队列，保证 RESET/ABORT 不会被后来的值覆盖；
 * - 状态和参数读取使用最新值快照，不阻塞 DartTask 的 1 ms 控制循环。
 */
/**
 * @brief 提交一条操作员命令到可靠事件队列。
 *
 * 本函数只复制命令，不直接推进状态机，也不等待命令执行完成。未来串口屏、遥控器适配器
 * 或维护工具均应通过此入口提交复位、中止、目标选择和发射命令。
 *
 * @param command 待提交的完整命令；函数返回后调用方可立即复用其存储。
 *
 * @return
 * - true：命令已进入队列；
 * - false：参数为空、主题总线未初始化或可靠事件队列已满。
 */
bool operator_command_submit(const dart_command_t *command);

/**
 * @brief 读取最近一次发布的飞镖业务状态。
 *
 * 读取过程复制完整快照，不返回主题总线内部指针，也不会阻塞 DartTask 的状态推进。
 *
 * @param status 接收状态快照的输出对象。
 *
 * @return
 * - true：已经复制一份有效状态；
 * - false：输出参数为空、总线未初始化或尚未发布过状态。
 */
bool operator_status_read(dart_status_t *status);

/**
 * @brief 读取最近一次生效的完整参数快照。
 *
 * @param parameters 接收参数副本的输出对象。
 *
 * @return
 * - true：已经复制一份有效参数；
 * - false：输出参数为空或系统尚未发布参数。
 */
bool operator_parameters_read(dart_parameters_t *parameters);

/**
 * @brief 请求校验、保存并切换一份完整参数。
 *
 * 本函数只把更新请求放入 AdjustTask 的可靠队列。AdjustTask 保存成功后才发布新参数；
 * DartTask 若处于工作状态，还会等到安全状态再原子切换。
 *
 * @param update 包含完整参数块和提交时刻的更新请求。
 *
 * @return
 * - true：更新请求已进入队列；
 * - false：参数为空、队列未初始化或队列已满。
 */
bool operator_parameters_update(const parameter_update_t *update);

#endif
