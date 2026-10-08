#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdbool.h>

/**
 * @brief 按统一静态描述表创建全部业务任务。
 *
 * 本函数只负责创建 DartTask、DecodeTask、UsbTask 和 AdjustTask，不启动调度器。任务栈、
 * 控制块和句柄均由应用层静态持有，因此调用过程不申请堆内存。应在主题总线及通信队列
 * 初始化完成后、FreeRTOS 调度器启动前调用一次。
 *
 * @return
 * - true：所有已启用任务均创建成功；
 * - false：至少一个任务创建失败，调用方不得继续启动不完整的业务系统。
 */
bool app_tasks_init(void);

#endif
