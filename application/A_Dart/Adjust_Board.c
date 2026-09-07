//
// Created by 86134 on 2025/11/27.
//
#include "Adjust_Board.h"

#include "cmsis_os.h"
#include "dart.h"
#include "Send_to_Screen.h"

#define BUFF_SIZE  128U
#define ADJUST_ACK_SUCCESS_COMMAND "adjust_ack.val=1"
#define ADJUST_ACK_FAILURE_COMMAND "adjust_ack.val=0"

static uint8_t rx_buff[BUFF_SIZE];

typedef enum
{
    ADJUST_WAIT_SOF = 0,
    ADJUST_WAIT_CMD,
    ADJUST_WAIT_ITEM,
    ADJUST_READ_VALUE,
    ADJUST_WAIT_VALUE_END
} Adjust_Parser_State_t;

static Adjust_Parser_State_t parser_state = ADJUST_WAIT_SOF;
static uint8_t parser_cmd;
static uint8_t parser_item;
static float parser_values[4];
static float parser_value;
static float parser_sign;
static float parser_point_value;
static bool parser_has_integer_digit;
static bool parser_has_point;
static bool parser_has_sign;
float aaa = 0.0f;

static void Adjust_Reset_Parser(void);
static void Adjust_Start_Frame(void);
static void Adjust_Reset_Value(void);
static void Adjust_Report(bool success);
static void Adjust_Process_Byte(uint8_t byte);
static bool Adjust_Commit_Frame(uint8_t cmd, const float *values);
static void Adjust_Fail_And_Resync(uint8_t byte);

void adjust_task(void const *pvParameters)
{
    (void)pvParameters;

    /* UART7 is used for both the adjustment input and screen output. */
    HAL_UARTEx_ReceiveToIdle_IT(&huart7, rx_buff, BUFF_SIZE);

    while (1)
    {
        osDelay(1);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UART7)
    {
        if (size <= BUFF_SIZE)
        {
            for (uint16_t index = 0U; index < size; index++)
            {
                /* The parser state survives this callback and the next one. */
                Adjust_Process_Byte(rx_buff[index]);
            }
        }

        memset(rx_buff, 0, sizeof(rx_buff));
        HAL_UARTEx_ReceiveToIdle_IT(&huart7, rx_buff, BUFF_SIZE);
    }
}

void Adjust_Board_Restart_Receive(void)
{
    memset(rx_buff, 0, sizeof(rx_buff));
    HAL_UARTEx_ReceiveToIdle_IT(&huart7, rx_buff, BUFF_SIZE);
}

static void Adjust_Reset_Parser(void)
{
    parser_state = ADJUST_WAIT_SOF;
    parser_cmd = 0U;
    parser_item = 0U;
    parser_value = 0.0f;
    parser_sign = 1.0f;
    parser_point_value = 0.1f;
    parser_has_integer_digit = false;
    parser_has_point = false;
    parser_has_sign = false;
}

static void Adjust_Start_Frame(void)
{
    parser_state = ADJUST_WAIT_CMD;
    parser_cmd = 0U;
    parser_item = 0U;
    parser_values[0] = 0.0f;
    parser_values[1] = 0.0f;
    parser_values[2] = 0.0f;
    parser_values[3] = 0.0f;
}

static void Adjust_Reset_Value(void)
{
    parser_state = ADJUST_READ_VALUE;
    parser_value = 0.0f;
    parser_sign = 1.0f;
    parser_point_value = 0.1f;
    parser_has_integer_digit = false;
    parser_has_point = false;
    parser_has_sign = false;
}

static void Adjust_Report(bool success)
{
    /* UART7 DMA transmission is non-blocking. */
    Send_to_Screen(success ? ADJUST_ACK_SUCCESS_COMMAND :
                           ADJUST_ACK_FAILURE_COMMAND);
}

static void Adjust_Fail_And_Resync(uint8_t byte)
{
    Adjust_Report(false);
    Adjust_Reset_Parser();

    /* A frame header can also be the first byte after a malformed frame. */
    if (byte == 0x55U)
    {
        Adjust_Start_Frame();
    }
}

static void Adjust_Process_Byte(uint8_t byte)
{
    switch (parser_state)
    {
        case ADJUST_WAIT_SOF:
            if (byte == 0x55U)
            {
                Adjust_Start_Frame();
            }
            break;

        case ADJUST_WAIT_CMD:
            if (byte >= 0x01U && byte <= 0x04U)
            {
                parser_cmd = byte;
                parser_item = 0U;
                parser_state = ADJUST_WAIT_ITEM;
            }
            else
            {
                Adjust_Fail_And_Resync(byte);
            }
            break;

        case ADJUST_WAIT_ITEM:
            if (byte == (uint8_t)(parser_item + 1U))
            {
                Adjust_Reset_Value();
            }
            else
            {
                Adjust_Fail_And_Resync(byte);
            }
            break;

        case ADJUST_READ_VALUE:
            if (byte == 0xFFU)
            {
                if (!parser_has_integer_digit)
                {
                    Adjust_Fail_And_Resync(byte);
                }
                else
                {
                    parser_state = ADJUST_WAIT_VALUE_END;
                }
            }
            else if (byte >= (uint8_t)'0' && byte <= (uint8_t)'9')
            {
                if (!parser_has_point)
                {
                    parser_value = parser_value * 10.0f +
                                    (float)(byte - (uint8_t)'0');
                    parser_has_integer_digit = true;
                }
                else
                {
                    parser_value +=
                        (float)(byte - (uint8_t)'0') * parser_point_value;
                    parser_point_value *= 0.1f;
                }
            }
            else if (byte == (uint8_t)'-' && !parser_has_sign &&
                     !parser_has_integer_digit && !parser_has_point)
            {
                parser_sign = -1.0f;
                parser_has_sign = true;
            }
            else if (byte == (uint8_t)'.' && !parser_has_point &&
                     parser_has_integer_digit)
            {
                parser_has_point = true;
            }
            else
            {
                Adjust_Fail_And_Resync(byte);
            }
            break;

        case ADJUST_WAIT_VALUE_END:
            if (byte != 0xFFU)
            {
                Adjust_Fail_And_Resync(byte);
                break;
            }

            parser_values[parser_item] = parser_value * parser_sign;
            parser_item++;

            if (parser_item >= 4U)
            {
                Adjust_Report(Adjust_Commit_Frame(parser_cmd, parser_values));
                Adjust_Reset_Parser();
            }
            else
            {
                parser_state = ADJUST_WAIT_ITEM;
            }
            break;

        default:
            Adjust_Reset_Parser();
            break;
    }
}

static bool Adjust_Commit_Frame(uint8_t cmd, const float *values)
{
    float *target = NULL;
    float min_value;
    float max_value;

    switch (cmd)
    {
        case 0x01U:
            target = dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION];
            min_value = launcher_dart.min_trigger_pos;
            max_value = launcher_dart.max_trigger_pos;
            break;

        case 0x02U:
            target = dart_goal_set.trigger_distance_set[GOAL_BASE_STATION];
            min_value = launcher_dart.min_trigger_pos;
            max_value = launcher_dart.max_trigger_pos;
            break;

        case 0x03U:
            target = dart_goal_set.yaw_angle_offset[GOAL_FRONT_STATION];
            min_value = gimbal_dart.min_yaw_pos -
                        dart_goal_set.yaw_angle_set[GOAL_FRONT_STATION];
            max_value = gimbal_dart.max_yaw_pos -
                        dart_goal_set.yaw_angle_set[GOAL_FRONT_STATION];
            break;

        case 0x04U:
            target = dart_goal_set.yaw_angle_offset[GOAL_BASE_STATION];
            min_value = gimbal_dart.min_yaw_pos -
                        dart_goal_set.yaw_angle_set[GOAL_BASE_STATION];
            max_value = gimbal_dart.max_yaw_pos -
                        dart_goal_set.yaw_angle_set[GOAL_BASE_STATION];
            break;

        default:
            return false;
    }

    for (uint8_t index = 0U; index < 4U; index++)
    {
        if (values[index] < min_value || values[index] > max_value)
        {
            return false;
        }
    }

    /* Commit only after all four values have passed parsing and range checks. */
    for (uint8_t index = 0U; index < 4U; index++)
    {
        target[index] = values[index];
    }

    return true;
}
