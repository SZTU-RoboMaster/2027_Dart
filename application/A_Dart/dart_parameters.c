#include "dart_parameters.h"

#include <math.h>
#include <stddef.h>
#include <string.h>


extern uint32_t get_crc32_check_sum(uint32_t *data, uint32_t len);

/*
 * 参数持久化策略
 * --------------
 * 完整参数结构带版本号并由 CRC 保护。遇到空白 Flash、机构类型不匹配、结构尺寸或版本
 * 不匹配、浮点值为 NaN/Inf、超时值不安全时，统一回退到保守默认参数。保存前由本模块
 * 补齐元数据和 CRC；普通业务复位永远不会擦除标定参数。
 */
_Static_assert((sizeof(dart_parameters_t) % sizeof(uint32_t)) == 0U,
               "dart_parameters_t must be word aligned");

/**
 * @brief 计算参数结构中受保护区域的 CRC32。
 *
 * 校验范围从结构首字节开始，到 `crc32` 字段之前结束，避免把旧校验值参与新校验。
 * 参数结构已通过静态断言保证按 32 位字对齐，满足底层 CRC 函数的输入要求。
 *
 * @param parameters 待计算的完整参数对象，不能为空。
 *
 * @return 参数元数据和业务字段对应的 CRC32 值。
 */
static uint32_t parameter_crc(const dart_parameters_t *parameters)
{
    const uint32_t word_count = (uint32_t)(offsetof(dart_parameters_t, crc32) / sizeof(uint32_t));
    return get_crc32_check_sum((uint32_t *)(uintptr_t)parameters, word_count);
}

/**
 * @brief 用经过审核的安全默认值初始化完整参数块。
 *
 * 本函数会覆盖目标对象的全部字段，并补齐魔数、版本、结构尺寸和 CRC。默认推板接弹位置
 * 使用原 `set_drive_distance()` 的计算结果；超时、同步阈值和首发预装策略也集中在此
 * 处管理，避免业务状态中散落固定值。
 *
 * @param parameters 待初始化的参数对象，必须为有效可写地址。
 */
void dart_parameters_defaults(dart_parameters_t *parameters)
{
    /* 历史标定值集中在这里管理，不再散落于状态处理函数。push_load_position 对应原
     * set_drive_distance()：飞镖长度减去滑轨长度再减 10.5，结果为 654.5 毫米。 */
    static const float trigger[3][DART_SHOT_COUNT] = {
        { -68.2753f, -62.7224792f, -67.0299621f, -64.0862838f },
        { -57.6753f, -60.2224792f, -60.9299621f, -61.8862838f },
        { -77.30f, -78.00f, -78.50f, -80.00f },
    };
    static const float turn[DART_LOADER_POSITION_COUNT] = {
        0.95f, 2.038f, 3.05f, 4.128f, 5.15f, 6.228f, 5.15f,
    };

    memset(parameters, 0, sizeof(*parameters));
    parameters->magic = DART_PARAMETER_MAGIC;
    parameters->version = DART_PARAMETER_VERSION;
    parameters->size = sizeof(*parameters);
    parameters->loader_type = DART_LOADER_TYPE_CAROUSEL_LIFT;
    memcpy(parameters->trigger_distance, trigger, sizeof(trigger));
    memcpy(parameters->loader_turn_position, turn, sizeof(turn));
    parameters->yaw_position[DART_GOAL_NONE] = 73.944f;
    parameters->yaw_position[DART_GOAL_FRONT] = 73.944f;
    parameters->yaw_position[DART_GOAL_BASE] = 73.944f;
    parameters->loader_lift_position = 45.7069f;
    parameters->push_load_position = 770.0f - 105.0f - 10.5f;
    parameters->push_exchange_position = 770.0f - 105.0f - 10.0f - 108.95f;
    parameters->push_back_position = 5.0f;
    parameters->push_sync_tolerance = 20.0f;
    parameters->lift_sync_tolerance = 2.0f;
    parameters->action_timeout_ms = 5000U;
    parameters->homing_timeout_ms = 5000U;
    parameters->aim_timeout_ms = 10000U;
    parameters->launch_wait_timeout_ms = 60000U;
    parameters->next_window_timeout_ms = 180000U;
    parameters->fire_hold_ms = 3000U;
    parameters->initial_dart_preloaded = true;
    parameters->crc32 = parameter_crc(parameters);
}

/**
 * @brief 校验参数块的身份、完整性和安全取值范围。
 *
 * 校验顺序为魔数、版本、尺寸、机构类型和 CRC，然后检查所有浮点参数是否为有限值，
 * 同步阈值是否为正数，以及动作时限是否低于安全下限。任意数组元素异常都会使整个参数
 * 块失效，调用者不得部分采用其中字段。
 *
 * @param parameters 待校验参数块。
 *
 * @return
 * - true：参数块完整且全部业务字段满足当前版本约束；
 * - false：参数为空，或任一元数据、CRC、浮点值、阈值、时限不合法。
 */
bool dart_parameters_validate(const dart_parameters_t *parameters)
{
    /* 先验证身份和 CRC，再使用后续持久化字段。 */
    if (parameters == NULL || parameters->magic != DART_PARAMETER_MAGIC ||
        parameters->version != DART_PARAMETER_VERSION || parameters->size != sizeof(*parameters) ||
        parameters->loader_type != DART_LOADER_TYPE_CAROUSEL_LIFT ||
        parameters->crc32 != parameter_crc(parameters)) {
        return false;
    }
    if (!isfinite(parameters->loader_lift_position) ||
        !isfinite(parameters->push_load_position) ||
        !isfinite(parameters->push_exchange_position) ||
        !isfinite(parameters->push_back_position) ||
        !isfinite(parameters->push_sync_tolerance) ||
        !isfinite(parameters->lift_sync_tolerance) ||
        parameters->push_sync_tolerance <= 0.0f || parameters->lift_sync_tolerance <= 0.0f ||
        parameters->action_timeout_ms < 100U || parameters->homing_timeout_ms < 100U ||
        parameters->aim_timeout_ms < 100U || parameters->launch_wait_timeout_ms < 100U ||
        parameters->next_window_timeout_ms < 100U ||
        parameters->fire_hold_ms < 100U) {
        return false;
    }
    for (uint8_t goal = 0U; goal < 3U; ++goal) {
        if (!isfinite(parameters->yaw_position[goal])) return false;
        for (uint8_t shot = 0U; shot < DART_SHOT_COUNT; ++shot) {
            if (!isfinite(parameters->trigger_distance[goal][shot])) return false;
        }
    }
    for (uint8_t index = 0U; index < DART_LOADER_POSITION_COUNT; ++index) {
        if (!isfinite(parameters->loader_turn_position[index])) return false;
    }
    return true;
}

/**
 * @brief 从平台非易失存储加载参数，并在无效时回退到安全默认值。
 *
 * 读取成功后仍会执行完整校验。Flash 空白、读取失败、版本不兼容、CRC 错误或字段异常时，
 * 输出对象都会被安全默认值完整覆盖，因此调用者在返回 false 时仍可继续使用输出参数。
 * 本函数不会把默认值自动写回 Flash，避免因偶发读取故障覆盖原始数据。
 *
 * @param platform 当前板级操作表，必须提供参数读取回调。
 * @param parameters 用于接收参数的可写对象。
 *
 * @return
 * - true：已从非易失存储读到并通过校验；
 * - false：读取或校验失败，输出已替换为安全默认值。
 */
bool dart_parameters_load(const dart_platform_ops_t *platform, dart_parameters_t *parameters)
{
    /* 返回 false 表示 Flash 数据无效且已由安全默认值替代。 */
    if (platform == NULL || platform->parameters_read == NULL || parameters == NULL ||
        !platform->parameters_read(parameters, sizeof(*parameters))) {
        if (parameters != NULL) dart_parameters_defaults(parameters);
        return false;
    }
    if (dart_parameters_validate(parameters)) return true;
    dart_parameters_defaults(parameters);
    return false;
}

/**
 * @brief 校验并持久化一份完整参数块。
 *
 * 保存前会统一重写魔数、版本、尺寸和 CRC，随后再次执行完整参数校验。只有校验通过才会
 * 调用平台写入接口，避免将 NaN、错误机构类型或危险时限写入 Flash。函数会更新传入对象
 * 的元数据和 CRC 字段。
 *
 * @param platform 当前板级操作表，必须提供参数写入回调。
 * @param parameters 待保存的可写参数对象。
 *
 * @return
 * - true：参数合法且平台确认写入成功；
 * - false：参数或回调无效、参数校验失败，或底层写入失败。
 */
bool dart_parameters_save(const dart_platform_ops_t *platform, dart_parameters_t *parameters)
{
    /* 调用者只需修改业务字段，版本、尺寸、魔数和 CRC 均由本模块维护。 */
    if (platform == NULL || platform->parameters_write == NULL || parameters == NULL) return false;
    parameters->magic = DART_PARAMETER_MAGIC;
    parameters->version = DART_PARAMETER_VERSION;
    parameters->size = sizeof(*parameters);
    parameters->crc32 = parameter_crc(parameters);
    if (!dart_parameters_validate(parameters)) return false;
    return platform->parameters_write(parameters, sizeof(*parameters));
}
