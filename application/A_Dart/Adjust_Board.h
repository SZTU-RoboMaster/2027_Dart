#ifndef DART_2026_ADJUST_BOARD_H
#define DART_2026_ADJUST_BOARD_H

/**
 * @brief 运行参数加载、校验、持久化和发布任务。
 *
 * 上电先读取完整参数块并发布安全快照，随后阻塞等待参数更新事件。只有校验和写入都成功
 * 的参数才会重新发布。本任务是参数存储的唯一写入者，函数不会返回。
 *
 * 当前版本不绑定具体串口屏协议。后续界面只能通过操作员接口提交完整参数更新，不得直接
 * 擦写 Flash 或修改 DartTask 正在读取的参数对象。
 *
 * @param argument 静态任务表保留参数，当前实现忽略该值。
 */
void adjust_task(void const *argument);

#endif //DART_2026_ADJUST_BOARD_H
