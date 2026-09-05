// ============================================================
// 淘晶驰串口屏接收与参数备份模块
//
// 功能：
// 1. 上电读取串口屏掉电存储区中的触发距离备份；
// 2. 串口屏备份无效时使用固件默认参数；
// 3. 接收串口屏发来的参数并更新飞镖控制参数；
// 4. 将八个触发距离整组备份到串口屏；
// 5. 串口屏使用单份备份记录保存参数。
//
// 串口：UART7（PE8 TX / PE7 RX），DMA 空闲中断接收。
// ============================================================

#include "Adjust_Board.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "cmsis_os.h"
#include "Send_to_Screen.h"
#include "dart.h"

// DMA 接收缓冲区
static uint8_t rx_buff[BUFF_SIZE];

// 自定义参数帧可能因屏幕脚本中的多条指令跨越多个 DMA 空闲回调。
// 收到四个 FF FF 字段结束符后再一次性解析，避免半帧被丢弃。
static uint8_t custom_frame_buff[BUFF_SIZE];
static uint16_t custom_frame_size;
static uint8_t custom_frame_terminators;

// 前哨站四个距离加基地四个距离
fp32 outpost_base_value[8];

// 接收到 get sys0 数值返回帧后，由中断回调更新这两个变量
static volatile uint32_t screen_number_value;
static volatile uint8_t screen_number_ready;

// 每成功接收一组新参数，版本号增加一次
static volatile uint32_t parameter_revision;

// 参数初始化完成前不接受用户参数更新
static volatile uint8_t parameter_initialized;

// 串口屏已经保存到哪个参数版本
static uint32_t screen_saved_revision;

// 当前串口屏备份记录的保存序号
static uint32_t screen_sequence;

// 串口屏备份重试状态
static uint32_t screen_retry_revision;
static uint32_t screen_next_retry_tick;
static uint8_t screen_retry_count;

/**
 * @brief 串口屏中的一份完整参数记录
 */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    int32_t distance[8];
    uint32_t crc;
} Screen_Backup_Record_t;

static void start_uart7_receive(void);
static void parse_received_data(uint8_t *data, uint16_t size);
static void parse_custom_param(uint8_t *data, uint16_t size);
static void parse_tjc_touch(uint8_t *data, uint16_t size);
static void parse_tjc_number(uint8_t *data);

static bool Str_to_float(const uint8_t *msg,
                         uint16_t size,
                         uint16_t *position,
                         fp32 *value);

static bool distance_to_fixed(fp32 value, int32_t *fixed_value);
static bool parameters_are_valid(const fp32 *values);
static void apply_parameters(const fp32 *values);
static void read_runtime_defaults(fp32 *values);
static void copy_parameter_snapshot(fp32 *values, uint32_t *revision);

static uint32_t calculate_record_crc(const Screen_Backup_Record_t *record);
static bool screen_read_int32(uint16_t address, int32_t *value);
static bool screen_read_record(uint16_t base_address,
                               Screen_Backup_Record_t *record);
static bool screen_write_record(uint16_t base_address,
                                const Screen_Backup_Record_t *record);
static bool screen_record_equal(const Screen_Backup_Record_t *left,
                                const Screen_Backup_Record_t *right);
static bool screen_record_to_values(const Screen_Backup_Record_t *record,
                                    fp32 *values);
static void build_screen_record(const fp32 *values,
                                uint32_t sequence,
                                Screen_Backup_Record_t *record);
static bool restore_screen_backup(fp32 *values);
static bool save_screen_backup(const fp32 *values);

/**
 * @brief 调参屏任务
 * @param pvParameters FreeRTOS 任务参数，本任务不使用
 */
void adjust_task(void const *pvParameters)
{
    fp32 startup_values[8];
    uint32_t snapshot_revision;
    bool screen_backup_valid;

    (void)pvParameters;

    // 必须先启动接收，否则 MCU 无法收到 get sys0 的返回帧
    start_uart7_receive();

    // 等待串口屏完成上电初始化
    osDelay(SCREEN_STARTUP_DELAY_MS);

    // 读取串口屏中的单份备份记录
    screen_backup_valid = restore_screen_backup(startup_values);

    if (screen_backup_valid)
    {
        // 串口屏中的备份是唯一的掉电恢复来源
        apply_parameters(startup_values);
    }
    else
    {
        // 串口屏没有有效备份时，使用 dart_goal_set 中的固件默认值
        read_runtime_defaults(startup_values);
        apply_parameters(startup_values);
    }

    // 初始化完成后，当前参数版本从 1 开始
    parameter_revision = 1U;
    parameter_initialized = 1U;

    if (screen_backup_valid)
    {
        screen_saved_revision = 1U;
        screen_retry_revision = 1U;
        screen_retry_count = 0U;
    }
    else
    {
        // 串口屏没有有效备份时，立即建立第一份备份
        screen_retry_revision = 1U;

        if (save_screen_backup(outpost_base_value))
        {
            screen_saved_revision = 1U;
            screen_retry_count = 0U;
        }
        else
        {
            screen_saved_revision = 0U;
            screen_retry_count = 1U;
            screen_next_retry_tick =
                HAL_GetTick() + SCREEN_RETRY_INTERVAL_MS;
        }
    }

    // 用最终采用的八个参数刷新两个串口屏页面
    Send_to_Screen_Data(&outpost_base_value[0],
                        &outpost_base_value[4]);

    while (1)
    {
        uint32_t current_revision = parameter_revision;

        // 参数版本变化后，为新版本重新计算备份重试次数
        if (current_revision != screen_retry_revision)
        {
            screen_retry_revision = current_revision;
            screen_retry_count = 0U;
            screen_next_retry_tick = HAL_GetTick();
        }

        // 串口屏尚未保存当前版本时，执行有限次数的重试
        if (current_revision != screen_saved_revision &&
            screen_retry_count < SCREEN_MAX_RETRY_COUNT &&
            (int32_t)(HAL_GetTick() - screen_next_retry_tick) >= 0)
        {
            copy_parameter_snapshot(startup_values, &snapshot_revision);

            if (save_screen_backup(startup_values))
            {
                screen_saved_revision = snapshot_revision;
                screen_retry_count = 0U;
            }
            else
            {
                screen_retry_count++;
                screen_next_retry_tick =
                    HAL_GetTick() + SCREEN_RETRY_INTERVAL_MS;
            }
        }

        osDelay(10);
    }
}

/**
 * @brief 启动 UART7 DMA 空闲中断接收
 */
static void start_uart7_receive(void)
{
    memset(rx_buff, 0, sizeof(rx_buff));

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart7,
                                      rx_buff,
                                      sizeof(rx_buff));

    // 本模块只使用空闲事件，不需要 DMA 半传输事件
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
}

/**
 * @brief UART DMA 空闲事件回调
 * @param huart 触发回调的串口
 * @param size 本次接收的数据长度
 *
 * @note
 * 中断中只解析和更新内存，不执行串口屏掉电存储写入。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance != UART7)
    {
        return;
    }

    if (size > 0U && size <= BUFF_SIZE)
    {
        parse_received_data(rx_buff, size);
    }

    start_uart7_receive();
}

/**
 * @brief 在一段接收数据中查找并处理不同类型的串口屏帧
 */
static void parse_received_data(uint8_t *data, uint16_t size)
{
    uint16_t position = 0U;

    while (position < size)
    {
        // 自定义参数帧采用流式缓存，允许屏幕脚本的多条 prints/printh
        // 指令在不同的 DMA 空闲回调中到达。
        if (custom_frame_size > 0U || data[position] == 0x55U)
        {
            uint8_t current = data[position++];

            // 新帧头到来时丢弃无法完成的旧帧，避免缓存被错误数据卡住。
            if (custom_frame_size > 0U &&
                current == 0x55U &&
                custom_frame_size > 2U)
            {
                custom_frame_size = 0U;
                custom_frame_terminators = 0U;
            }

            if (custom_frame_size >= sizeof(custom_frame_buff))
            {
                custom_frame_size = 0U;
                custom_frame_terminators = 0U;
                continue;
            }

            custom_frame_buff[custom_frame_size++] = current;

            if (custom_frame_size >= 2U &&
                custom_frame_buff[custom_frame_size - 2U] == 0xFFU &&
                custom_frame_buff[custom_frame_size - 1U] == 0xFFU)
            {
                custom_frame_terminators++;
            }

            if (custom_frame_terminators >= 4U)
            {
                parse_custom_param(custom_frame_buff, custom_frame_size);
                custom_frame_size = 0U;
                custom_frame_terminators = 0U;
            }

            continue;
        }

        // 淘晶驰标准触摸事件固定为七个字节
        if (data[position] == 0x65U &&
            position + 6U < size &&
            data[position + 4U] == 0xFFU &&
            data[position + 5U] == 0xFFU &&
            data[position + 6U] == 0xFFU)
        {
            parse_tjc_touch(&data[position], 7U);
            position += 7U;
            continue;
        }

        // get 数值返回帧固定为八个字节
        if (data[position] == 0x71U &&
            position + 7U < size &&
            data[position + 5U] == 0xFFU &&
            data[position + 6U] == 0xFFU &&
            data[position + 7U] == 0xFFU)
        {
            parse_tjc_number(&data[position]);
            position += 8U;
            continue;
        }

        // 命令成功、命令错误等返回帧不参与业务处理
        position++;
    }
}

/**
 * @brief 解析淘晶驰 get 数值返回帧
 * @param data 指向 0x71 帧头
 */
static void parse_tjc_number(uint8_t *data)
{
    // 淘晶驰数值返回采用四字节小端格式
    screen_number_value =
        ((uint32_t)data[1]) |
        ((uint32_t)data[2] << 8U) |
        ((uint32_t)data[3] << 16U) |
        ((uint32_t)data[4] << 24U);

    // 必须最后设置完成标志，保证任务读取时数值已经更新
    screen_number_ready = 1U;
}

/**
 * @brief 解析自定义参数协议
 *
 * 格式：
 * 0x55 + 命令字 + [序号 + 浮点数字符串 + 0xFF 0xFF] × 4
 *
 * 命令字 0x01 表示前哨站，0x02 表示基地。
 */
static void parse_custom_param(uint8_t *data, uint16_t size)
{
    fp32 received_values[4];
    uint16_t position = 2U;
    uint8_t station;
    uint8_t base_index;

    if (!parameter_initialized || size < 4U)
    {
        return;
    }

    if (data[1] == 0x01U)
    {
        station = GOAL_FRONT_STATION;
        base_index = 0U;
    }
    else if (data[1] == 0x02U)
    {
        station = GOAL_BASE_STATION;
        base_index = 4U;
    }
    else
    {
        return;
    }

    for (uint8_t index = 0U; index < 4U; index++)
    {
        // 跳过无关字节，寻找当前参数应有的序号
        while (position < size && data[position] != index + 1U)
        {
            position++;
        }

        if (position >= size)
        {
            return;
        }

        position++;

        if (!Str_to_float(data,
                          size,
                          &position,
                          &received_values[index]))
        {
            return;
        }

        // 确保该数值能够按四位小数转换成 32 位定点整数
        if (!distance_to_fixed(received_values[index], NULL))
        {
            return;
        }
    }

    // 四个参数全部解析成功后再整组提交，避免只更新一部分数据
    for (uint8_t index = 0U; index < 4U; index++)
    {
        outpost_base_value[base_index + index] =
            received_values[index];

        dart_goal_set.trigger_distance_set[station][index] =
            received_values[index];
    }

    // 参数数组全部更新后再增加版本号
    parameter_revision++;

    // 避免整数回绕后出现版本号 0
    if (parameter_revision == 0U)
    {
        parameter_revision = 1U;
    }
}

/**
 * @brief 解析淘晶驰标准触摸事件
 */
static void parse_tjc_touch(uint8_t *data, uint16_t size)
{
    uint8_t page_id;
    uint8_t comp_id;
    uint8_t event;

    if (size < 7U)
    {
        return;
    }

    if (data[4] != 0xFFU ||
        data[5] != 0xFFU ||
        data[6] != 0xFFU)
    {
        return;
    }

    page_id = data[1];
    comp_id = data[2];
    event = data[3];

    // 当前没有仅靠触摸事件执行的业务
    if (event != 0x01U)
    {
        return;
    }

    // 后续可以在这里根据页面号和控件号添加按键功能
    (void)page_id;
    (void)comp_id;
}

/**
 * @brief 将 ASCII 浮点数字符串转换为 fp32
 * @param msg 接收缓冲区
 * @param size 接收缓冲区有效长度
 * @param position 当前解析位置，返回时指向下一字段
 * @param value 解析得到的浮点数
 * @retval true 解析成功，false 数据不完整或格式错误
 */
static bool Str_to_float(const uint8_t *msg,
                         uint16_t size,
                         uint16_t *position,
                         fp32 *value)
{
    double result = 0.0;
    double decimal_weight = 0.1;
    bool decimal_mode = false;
    bool has_digit = false;
    bool negative = false;

    if (msg == NULL || position == NULL || value == NULL)
    {
        return false;
    }

    while (*position < size)
    {
        uint8_t current = msg[*position];

        if (*position + 1U < size &&
            current == 0xFFU &&
            msg[*position + 1U] == 0xFFU)
        {
            *position += 2U;

            if (!has_digit)
            {
                return false;
            }

            *value = (fp32)(negative ? -result : result);
            return true;
        }

        if (current == '-' && !has_digit && !decimal_mode)
        {
            negative = true;
            (*position)++;
            continue;
        }

        if (current == '.' && !decimal_mode)
        {
            decimal_mode = true;
            (*position)++;
            continue;
        }

        if (current >= '0' && current <= '9')
        {
            uint8_t digit = (uint8_t)(current - '0');
            has_digit = true;

            if (!decimal_mode)
            {
                result = result * 10.0 + digit;
            }
            else
            {
                result += digit * decimal_weight;
                decimal_weight *= 0.1;
            }

            (*position)++;
            continue;
        }

        // 允许数字前后出现空格和换行
        if (current == ' ' || current == '\r' || current == '\n')
        {
            (*position)++;
            continue;
        }

        return false;
    }

    return false;
}

/**
 * @brief 将浮点距离转换为保留四位小数的 32 位定点整数
 * @param value 浮点距离
 * @param fixed_value 转换结果，传入 NULL 时只检查合法性
 */
static bool distance_to_fixed(fp32 value, int32_t *fixed_value)
{
    double scaled;
    double rounded;

    // NaN 是唯一一个不等于自身的浮点值
    if (value != value)
    {
        return false;
    }

    scaled = (double)value * SCREEN_DISTANCE_SCALE;
    rounded = scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5;

    if (rounded > (double)INT32_MAX ||
        rounded < (double)INT32_MIN)
    {
        return false;
    }

    if (fixed_value != NULL)
    {
        *fixed_value = (int32_t)rounded;
    }

    return true;
}

/**
 * @brief 检查八个参数是否都能安全写入串口屏
 */
static bool parameters_are_valid(const fp32 *values)
{
    if (values == NULL)
    {
        return false;
    }

    for (uint8_t index = 0U; index < 8U; index++)
    {
        if (!distance_to_fixed(values[index], NULL))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 将八个参数应用到缓存和飞镖控制结构体
 */
static void apply_parameters(const fp32 *values)
{
    for (uint8_t index = 0U; index < 4U; index++)
    {
        outpost_base_value[index] = values[index];
        outpost_base_value[index + 4U] = values[index + 4U];

        dart_goal_set
            .trigger_distance_set[GOAL_FRONT_STATION][index] =
            values[index];

        dart_goal_set
            .trigger_distance_set[GOAL_BASE_STATION][index] =
            values[index + 4U];
    }
}

/**
 * @brief 从 dart_goal_set 读取固件内置默认参数
 */
static void read_runtime_defaults(fp32 *values)
{
    for (uint8_t index = 0U; index < 4U; index++)
    {
        values[index] =
            dart_goal_set
                .trigger_distance_set[GOAL_FRONT_STATION][index];

        values[index + 4U] =
            dart_goal_set
                .trigger_distance_set[GOAL_BASE_STATION][index];
    }

    // 如果固件默认值本身无效，则使用全零值作为最后兜底
    if (!parameters_are_valid(values))
    {
        memset(values, 0, sizeof(fp32) * 8U);
    }
}

/**
 * @brief 取得一份版本一致的八参数快照
 */
static void copy_parameter_snapshot(fp32 *values, uint32_t *revision)
{
    uint32_t before;
    uint32_t after;

    do
    {
        before = parameter_revision;

        for (uint8_t index = 0U; index < 8U; index++)
        {
            values[index] = outpost_base_value[index];
        }

        after = parameter_revision;
    }
    while (before != after);

    *revision = after;
}

/**
 * @brief 计算备份记录的 CRC32
 *
 * @note
 * magic 不参与 CRC，因为它是整条记录最后写入的有效标志。
 */
static uint32_t calculate_record_crc(const Screen_Backup_Record_t *record)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t words[10];

    words[0] = record->version;
    words[1] = record->sequence;

    for (uint8_t index = 0U; index < 8U; index++)
    {
        words[index + 2U] = (uint32_t)record->distance[index];
    }

    for (uint8_t word_index = 0U; word_index < 10U; word_index++)
    {
        uint32_t word = words[word_index];

        for (uint8_t byte_index = 0U; byte_index < 4U; byte_index++)
        {
            crc ^= (word >> (byte_index * 8U)) & 0xFFU;

            for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++)
            {
                if ((crc & 1U) != 0U)
                {
                    crc = (crc >> 1U) ^ 0xEDB88320UL;
                }
                else
                {
                    crc >>= 1U;
                }
            }
        }
    }

    return (crc ^ 0xFFFFFFFFUL) & SCREEN_CRC_MASK;
}

/**
 * @brief 从串口屏掉电存储区读取一个 32 位整数
 */
static bool screen_read_int32(uint16_t address, int32_t *value)
{
    uint32_t start_tick;

    if (value == NULL)
    {
        return false;
    }

    // 清除上一条数值返回帧，防止误用旧数据
    screen_number_ready = 0U;

    Send_to_Screen_Eeprom_Read_Request(address);

    start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) <
           SCREEN_READ_TIMEOUT_MS)
    {
        if (screen_number_ready)
        {
            *value = (int32_t)screen_number_value;
            screen_number_ready = 0U;
            return true;
        }

        osDelay(1);
    }

    return false;
}

/**
 * @brief 读取并校验串口屏中的单份备份记录
 */
static bool screen_read_record(uint16_t base_address,
                               Screen_Backup_Record_t *record)
{
    int32_t value;

    if (!screen_read_int32(base_address + SCREEN_MAGIC_OFFSET, &value))
    {
        return false;
    }

    record->magic = (uint32_t)value;

    if (record->magic != SCREEN_BACKUP_MAGIC)
    {
        return false;
    }

    if (!screen_read_int32(base_address + SCREEN_VERSION_OFFSET, &value))
    {
        return false;
    }
    record->version = (uint32_t)value;

    if (!screen_read_int32(base_address + SCREEN_SEQUENCE_OFFSET, &value))
    {
        return false;
    }
    record->sequence = (uint32_t)value;

    for (uint8_t index = 0U; index < 8U; index++)
    {
        if (!screen_read_int32(
                (uint16_t)(base_address +
                           SCREEN_DISTANCE_OFFSET +
                           index * 4U),
                &record->distance[index]))
        {
            return false;
        }
    }

    if (!screen_read_int32(base_address + SCREEN_CRC_OFFSET, &value))
    {
        return false;
    }
    record->crc = (uint32_t)value;

    if (record->version != SCREEN_BACKUP_VERSION)
    {
        return false;
    }

    return record->crc == calculate_record_crc(record);
}

/**
 * @brief 将完整记录写入串口屏并回读校验
 */
static bool screen_write_record(uint16_t base_address,
                                const Screen_Backup_Record_t *record)
{
    Screen_Backup_Record_t verify_record;

    // 先使目标记录失效，避免写到一半掉电时被误认为有效
    Send_to_Screen_Eeprom_Write(
        base_address + SCREEN_MAGIC_OFFSET,
        0);
    osDelay(SCREEN_WRITE_DELAY_MS);

    Send_to_Screen_Eeprom_Write(
        base_address + SCREEN_VERSION_OFFSET,
        (int32_t)record->version);
    osDelay(SCREEN_WRITE_DELAY_MS);

    Send_to_Screen_Eeprom_Write(
        base_address + SCREEN_SEQUENCE_OFFSET,
        (int32_t)record->sequence);
    osDelay(SCREEN_WRITE_DELAY_MS);

    for (uint8_t index = 0U; index < 8U; index++)
    {
        Send_to_Screen_Eeprom_Write(
            (uint16_t)(base_address +
                       SCREEN_DISTANCE_OFFSET +
                       index * 4U),
            record->distance[index]);
        osDelay(SCREEN_WRITE_DELAY_MS);
    }

    Send_to_Screen_Eeprom_Write(
        base_address + SCREEN_CRC_OFFSET,
        (int32_t)record->crc);
    osDelay(SCREEN_WRITE_DELAY_MS);

    // 所有正文写完后，最后写入有效标志
    Send_to_Screen_Eeprom_Write(
        base_address + SCREEN_MAGIC_OFFSET,
        (int32_t)record->magic);
    osDelay(SCREEN_WRITE_DELAY_MS);

    // 回读验证，只有全部字段一致才认为备份成功
    if (!screen_read_record(base_address, &verify_record))
    {
        return false;
    }

    return screen_record_equal(record, &verify_record);
}

/**
 * @brief 比较两份串口屏备份记录
 */
static bool screen_record_equal(const Screen_Backup_Record_t *left,
                                const Screen_Backup_Record_t *right)
{
    if (left == NULL || right == NULL)
    {
        return false;
    }

    if (left->magic != right->magic ||
        left->version != right->version ||
        left->sequence != right->sequence ||
        left->crc != right->crc)
    {
        return false;
    }

    for (uint8_t index = 0U; index < 8U; index++)
    {
        if (left->distance[index] != right->distance[index])
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 将串口屏定点参数转换为浮点距离
 */
static bool screen_record_to_values(const Screen_Backup_Record_t *record,
                                    fp32 *values)
{
    if (record == NULL || values == NULL)
    {
        return false;
    }

    for (uint8_t index = 0U; index < 8U; index++)
    {
        values[index] =
            (fp32)((double)record->distance[index] /
                   SCREEN_DISTANCE_SCALE);
    }

    return parameters_are_valid(values);
}

/**
 * @brief 根据八个浮点距离构造一份串口屏备份记录
 */
static void build_screen_record(const fp32 *values,
                                uint32_t sequence,
                                Screen_Backup_Record_t *record)
{
    record->magic = SCREEN_BACKUP_MAGIC;
    record->version = SCREEN_BACKUP_VERSION;
    record->sequence = sequence;

    for (uint8_t index = 0U; index < 8U; index++)
    {
        (void)distance_to_fixed(values[index],
                                &record->distance[index]);
    }

    record->crc = calculate_record_crc(record);
}

/**
 * @brief 从串口屏单份备份记录恢复参数
 */
static bool restore_screen_backup(fp32 *values)
{
    Screen_Backup_Record_t record;

    if (!screen_read_record(SCREEN_BACKUP_ADDRESS, &record))
    {
        screen_sequence = 0U;
        return false;
    }

    screen_sequence = record.sequence;
    return screen_record_to_values(&record, values);
}

/**
 * @brief 将八个参数写入串口屏单份备份记录
 */
static bool save_screen_backup(const fp32 *values)
{
    Screen_Backup_Record_t record;
    uint32_t next_sequence;

    if (!parameters_are_valid(values))
    {
        return false;
    }

    // 保存序号保持在有符号 32 位正整数范围内
    if (screen_sequence >= 0x7FFFFFFEUL)
    {
        next_sequence = 1U;
    }
    else
    {
        next_sequence = screen_sequence + 1U;
    }

    build_screen_record(values, next_sequence, &record);

    if (!screen_write_record(SCREEN_BACKUP_ADDRESS, &record))
    {
        return false;
    }

    // 只有写入并回读验证成功后才更新保存序号
    screen_sequence = next_sequence;

    return true;
}
