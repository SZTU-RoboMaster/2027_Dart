#ifndef DART_PARAMETERS_H
#define DART_PARAMETERS_H

#include <stdbool.h>

#include "dart_topics.h"
#include "dart_platform.h"

/**
 * @brief 生成一份完整且可直接使用的安全默认参数。
 *
 * 函数会先清零整个结构，再填入机构标定、超时、版本、长度和校验值。
 *
 * @param parameters 接收默认参数的对象；调用方必须提供有效指针。
 */
void dart_parameters_defaults(dart_parameters_t *parameters);

/**
 * @brief 校验完整参数块能否安全用于当前机构。
 *
 * 校验范围包括魔数、结构版本、结构长度、机构类型、循环冗余校验值、所有浮点数的有限
 * 性以及同步阈值和超时下限。本函数不修改输入参数。
 *
 * @param parameters 待校验的完整参数块。
 *
 * @return true 表示全部校验通过；false 表示参数为空或任一检查失败。
 */
bool dart_parameters_validate(const dart_parameters_t *parameters);

/**
 * @brief 从板级非易失存储加载参数，并在失败时回退到默认值。
 *
 * 无论返回值如何，只要输出指针有效，函数返回后参数对象都包含可安全使用的内容。
 *
 * @param platform 提供持久化读取接口的板级操作表。
 * @param parameters 接收加载结果或默认值的对象。
 *
 * @return true 表示存储数据有效；false 表示读取或校验失败并已使用默认值。
 */
bool dart_parameters_load(const dart_platform_ops_t *platform, dart_parameters_t *parameters);

/**
 * @brief 校验并保存一份完整参数块。
 *
 * 保存前会统一更新魔数、版本、结构长度和校验值；校验失败时不会调用底层写入接口。
 *
 * @param platform 提供持久化写入接口的板级操作表。
 * @param parameters 待保存参数；元数据和校验字段会被本函数更新。
 *
 * @return true 表示底层写入成功；false 表示参数无效或底层写入失败。
 */
bool dart_parameters_save(const dart_platform_ops_t *platform, dart_parameters_t *parameters);

#endif
