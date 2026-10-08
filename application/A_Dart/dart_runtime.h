#ifndef DART_RUNTIME_H
#define DART_RUNTIME_H

/**
 * @brief 运行飞镖主状态机与执行器控制循环。
 *
 * 任务完成平台初始化、换弹策略绑定和状态机初始化后，以一毫秒固定周期执行输入刷新、
 * 反馈采样、状态推进、命令应用、控制器计算以及状态发布。函数不会返回。
 *
 * @param argument 静态任务表保留参数，当前实现忽略该值并要求传入空指针。
 */
void dart_runtime_task(void const *argument);

#endif
