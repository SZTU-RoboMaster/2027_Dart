#include "Adjust_Board.h"

#include "dart_parameters.h"
#include "topic_bus.h"

/*
 * AdjustTask：参数的唯一持久化拥有者
 * ----------------------------------
 * 1. 上电时从板级存储读取完整参数块；若数据为空、CRC 错误或版本不匹配，参数模块会
 *    自动返回安全默认值。
 * 2. 将当前有效参数发布到 TOPIC_DART_PARAMETERS，DartTask 只订阅该快照。
 * 3. 阻塞等待参数更新事件；保存成功后才发布新参数，写 Flash 失败时继续保留旧值。
 *
 * 这种设计保证业务状态机永远不会看到“只修改了一半”的参数结构，也避免多个任务
 * 同时擦写同一 Flash 扇区。
 */
/**
 * @brief 运行参数加载、校验、保存和发布任务。
 *
 * 任务启动时从平台非易失存储读取完整参数；无效数据由参数模块替换为安全默认值。随后
 * 阻塞等待参数更新事件，只有新参数校验并持久化成功后才替换当前快照并重新发布。任务
 * 是参数写入的唯一拥有者，可避免多个输入源并发擦写 Flash。函数不会返回。
 *
 * @param argument 静态任务表保留参数，当前实现忽略该值。
 */
void adjust_task(void const *argument)
{
    (void)argument;
    const dart_platform_ops_t *platform = dart_platform_stm32_get();
    dart_parameters_t parameters;

    /* load 返回 false 仅表示使用了默认值；parameters 本身始终可安全使用。 */
    (void)dart_parameters_load(platform, &parameters);
    (void)topic_publish(TOPIC_DART_PARAMETERS, &parameters);

    for (;;) {
        parameter_update_t update;
        /* 100 ms 超时使任务以后仍可加入非阻塞维护逻辑，不影响当前事件响应。 */
        if (topic_event_take(TOPIC_PARAMETER_UPDATE, &update, 100U)) {
            /* 只有完整校验并写入成功后，才替换运行时参数快照。 */
            if (dart_parameters_save(platform, &update.parameters)) {
                parameters = update.parameters;
                (void)topic_publish(TOPIC_DART_PARAMETERS, &parameters);
            }
        }
    }
}
