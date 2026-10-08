#include "app_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#include "Adjust_Board.h"
#include "dart.h"
#include "decode.h"
#include "usb_task.h"

/*
 * 应用任务统一注册表
 * ------------------
 * 任务栈、任务控制块和句柄全部使用静态存储，因此创建任务不消耗堆内存，内存占用也能
 * 直接从链接映射文件检查。以后新增业务任务只能修改此表，禁止在其他模块零散创建任务。
 */
typedef void (*app_task_entry_t)(void const *argument);

typedef struct {
    /* 任务名称会显示在支持 FreeRTOS 的调试器中，便于观察栈和运行状态。 */
    const char *name;
    app_task_entry_t entry;
    void *argument;
    UBaseType_t priority;
    uint32_t stack_words;
    StackType_t *stack;
    StaticTask_t *control;
    TaskHandle_t *handle;
    bool enabled;
} app_task_desc_t;

static StackType_t dart_stack[512];
static StackType_t decode_stack[384];
static StackType_t usb_stack[384];
static StackType_t adjust_stack[384];
static StaticTask_t dart_control;
static StaticTask_t decode_control;
static StaticTask_t usb_control;
static StaticTask_t adjust_control;
static TaskHandle_t dart_handle;
static TaskHandle_t decode_handle;
static TaskHandle_t usb_handle;
static TaskHandle_t adjust_handle;

static const app_task_desc_t tasks[] = {
    /* 最高优先级留给 1 毫秒周期的机构控制与安全状态机。 */
    { "DartTask", dart_task, NULL, tskIDLE_PRIORITY + 4U, 512U,
      dart_stack, &dart_control, &dart_handle, true },
    { "DecodeTask", decode_task, NULL, tskIDLE_PRIORITY + 3U, 384U,
      decode_stack, &decode_control, &decode_handle, true },
    { "UsbTask", usb_task, NULL, tskIDLE_PRIORITY + 2U, 384U,
      usb_stack, &usb_control, &usb_handle, true },
    { "AdjustTask", adjust_task, NULL, tskIDLE_PRIORITY + 1U, 384U,
      adjust_stack, &adjust_control, &adjust_handle, true },
};

/**
 * @brief 按统一注册表创建全部 Dart 业务任务。
 *
 * 本函数应在 FreeRTOS 调度器启动前调用一次。任务栈和任务控制块均为静态存储，不使用
 * FreeRTOS 堆。创建顺序由 `tasks` 表决定；若任一必要任务创建失败，函数立即返回失败，
 * 防止系统在缺少解码、发送或参数任务的情况下进入不完整运行状态。
 *
 * @return
 * - true：注册表中所有启用任务均已成功创建；
 * - false：至少一个启用任务创建失败。
 */
bool app_tasks_init(void)
{
    /* 调度器启动前一次性创建全部任务；任一任务创建失败都终止初始化，避免系统缺少部分
     * 业务任务却继续运行。 */
    for (uint32_t index = 0U; index < (sizeof(tasks) / sizeof(tasks[0])); ++index) {
        const app_task_desc_t *task = &tasks[index];
        if (!task->enabled) continue;
        *task->handle = xTaskCreateStatic((TaskFunction_t)task->entry,
                                          task->name,
                                          task->stack_words,
                                          task->argument,
                                          task->priority,
                                          task->stack,
                                          task->control);
        if (*task->handle == NULL) return false;
    }
    return true;
}
