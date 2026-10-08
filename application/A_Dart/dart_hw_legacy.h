#ifndef DART_HW_LEGACY_H
#define DART_HW_LEGACY_H

#include <stdbool.h>
#include <stdint.h>

#include "dart_topics.h"

/*
 * dart_platform_ops_t 背后的 F427 电机与 PWM 兼容桥。
 * 本层允许访问 HAL 和历史电机数组；除 dart_platform_stm32.c 外，其他业务模块禁止
 * 直接调用这些函数。
 */
/**
 * @brief 初始化当前 F427 电机控制对象和发射机构安全输出。
 *
 * 函数建立 PID 控制器初值、关闭发射机构并初始化可用的舵机总线。它不创建任务，也不
 * 启动任何轴运动，应在 DartTask 进入周期循环前调用一次。
 *
 * @return true 表示初始化完成；当前实现没有可恢复失败分支。
 */
bool dart_hw_legacy_init(void);

/**
 * @brief 读取平台单调毫秒时钟。
 *
 * 返回值允许按 32 位无符号数自然回绕，业务代码必须使用无符号时间差进行超时判断。
 *
 * @return 当前 HAL 毫秒计数值。
 */
uint32_t dart_hw_legacy_now_ms(void);

/**
 * @brief 采集并换算一份完整执行器反馈快照。
 *
 * 函数在同一调用内读取历史 CAN 电机对象、限位 GPIO 和转盘反馈，并换算成平台无关的
 * 位置、速度、在线状态与采样时刻。输出对象会被完整覆盖。
 *
 * @param feedback 接收位置、速度、限位、在线状态和采样时刻的输出对象。
 */
void dart_hw_legacy_sample_feedback(dart_feedback_t *feedback);

/**
 * @brief 锁存状态机产生的抽象轴命令。
 *
 * 本函数不计算 PID、不发送 CAN 帧；实际输出由随后同周期调用的 service 函数完成。
 *
 * @param command 本周期完整执行器命令。
 */
void dart_hw_legacy_apply(const dart_actuator_command_t *command);

/**
 * @brief 计算全部启用轴的串级控制器并下发一周期硬件输出。
 *
 * 本函数使用最近一次 `dart_hw_legacy_apply()` 锁存的目标，计算 PID 电流并发送 CAN、PWM
 * 和可选转盘命令。应由 DartTask 每个控制周期调用一次，不得在其他任务并发调用。
 */
void dart_hw_legacy_service(void);

/**
 * @brief 清除控制器运行历史并撤销所有旧执行器目标。
 *
 * 清理范围包括 PID 积分、历史误差、旧电流输出和锁存目标；标定增益、参数与机械零点
 * 不会被修改。完整业务恢复结束后调用，可防止下一周期沿用恢复前输出。
 */
void dart_hw_legacy_reset_control_state(void);

/**
 * @brief 把指定轴当前累计编码器位置登记为机械零点。
 *
 * 只有状态机已经通过真实限位确认机械位置后才能调用。函数更新对应电机的编码器偏移，
 * 不会主动驱动轴离开限位。
 *
 * @param axis 已由状态机确认命中限位的轴；不支持的轴会被安全忽略。
 */
void dart_hw_legacy_zero_axis(dart_axis_t axis);

/**
 * @brief 设置原换弹机构的一扇舵机门。
 *
 * 当换弹机构通过编译开关禁用时，本函数为空操作，不访问不存在的舵机硬件。
 *
 * @param index 舵机门编号，零号表示无有效门并被忽略。
 * @param open true 表示打开，false 表示关闭。
 */
void dart_hw_legacy_gate_set(uint8_t index, bool open);

/**
 * @brief 关闭原换弹机构的全部舵机门。
 *
 * 恢复结束、中止或故障安全处理时调用。拆机配置下为空操作，不影响核心任务运行。
 */
void dart_hw_legacy_gate_close_all(void);

#endif
