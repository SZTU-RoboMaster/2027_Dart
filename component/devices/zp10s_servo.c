#include "zp10s_servo.h"

#include <stdio.h>
#include <string.h>

#define ZP10S_CMD_BUFFER_LEN   32U
#define ZP10S_RESP_BUFFER_LEN  32U

static uint8_t ZP10S_IsHalfDuplex(const ZP10S_HandleTypeDef *handle)
{
    if ((handle == NULL) || (handle->uart == NULL))
    {
        return 0U;
    }

    return (READ_BIT(handle->uart->Instance->CR3, USART_CR3_HDSEL) != 0U) ? 1U : 0U;
}

static int ZP10S_IsDigit(char ch)
{
    return (ch >= '0') && (ch <= '9');
}

static ZP10S_Status ZP10S_CheckHandle(const ZP10S_HandleTypeDef *handle)
{
    if ((handle == NULL) || (handle->uart == NULL))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    return ZP10S_STATUS_OK;
}

static ZP10S_Status ZP10S_SelectTransmitter(ZP10S_HandleTypeDef *handle)
{
    if (ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK)
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    if (ZP10S_IsHalfDuplex(handle) &&
        (HAL_HalfDuplex_EnableTransmitter(handle->uart) != HAL_OK))
    {
        return ZP10S_STATUS_ERROR;
    }

    return ZP10S_STATUS_OK;
}

static ZP10S_Status ZP10S_SelectReceiver(ZP10S_HandleTypeDef *handle)
{
    if (ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK)
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    if (ZP10S_IsHalfDuplex(handle) &&
        (HAL_HalfDuplex_EnableReceiver(handle->uart) != HAL_OK))
    {
        return ZP10S_STATUS_ERROR;
    }

    return ZP10S_STATUS_OK;
}

static void ZP10S_ClearRxBuffer(ZP10S_HandleTypeDef *handle)
{
    while (__HAL_UART_GET_FLAG(handle->uart, UART_FLAG_RXNE) != RESET)
    {
        volatile uint32_t dummy = handle->uart->Instance->DR;
        (void)dummy;
    }

    __HAL_UART_CLEAR_OREFLAG(handle->uart);
}

static ZP10S_Status ZP10S_SendCommand(ZP10S_HandleTypeDef *handle, const char *command)
{
    uint32_t start_tick;
    ZP10S_Status status;

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) || (command == NULL))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    status = ZP10S_SelectTransmitter(handle);
    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    if (HAL_UART_Transmit(handle->uart,
                          (uint8_t *)command,
                          (uint16_t)strlen(command),
                          handle->tx_timeout_ms) != HAL_OK)
    {
        return ZP10S_STATUS_ERROR;
    }

    start_tick = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(handle->uart, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - start_tick) > handle->tx_timeout_ms)
        {
            return ZP10S_STATUS_TIMEOUT;
        }
    }

    status = ZP10S_SelectReceiver(handle);
    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    return ZP10S_STATUS_OK;
}

static ZP10S_Status ZP10S_ReadFrame(ZP10S_HandleTypeDef *handle,
                                    char *response,
                                    size_t response_len,
                                    uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    size_t index = 0U;
    uint8_t ch = 0U;
    ZP10S_Status status;

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) || (response == NULL) || (response_len < 4U))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    status = ZP10S_SelectReceiver(handle);
    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        uint32_t elapsed = HAL_GetTick() - start_tick;
        uint32_t remaining = timeout_ms - elapsed;
        HAL_StatusTypeDef hal_status = HAL_UART_Receive(handle->uart,
                                                        &ch,
                                                        1U,
                                                        (remaining == 0U) ? 1U : remaining);

        if (hal_status == HAL_TIMEOUT)
        {
            return ZP10S_STATUS_TIMEOUT;
        }

        if (hal_status != HAL_OK)
        {
            return ZP10S_STATUS_ERROR;
        }

        if ((index == 0U) && (ch != '#'))
        {
            continue;
        }

        if (index >= (response_len - 1U))
        {
            return ZP10S_STATUS_PARSE_ERROR;
        }

        response[index++] = (char)ch;

        if (ch == '!')
        {
            response[index] = '\0';
            return ZP10S_STATUS_OK;
        }
    }

    return ZP10S_STATUS_TIMEOUT;
}

static ZP10S_Status ZP10S_Query(ZP10S_HandleTypeDef *handle,
                                const char *command,
                                char *response,
                                size_t response_len)
{
    uint32_t start_tick = HAL_GetTick();
    ZP10S_Status status = ZP10S_CheckHandle(handle);

    if ((status != ZP10S_STATUS_OK) || (command == NULL) || (response == NULL) || (response_len == 0U))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    status = ZP10S_SelectReceiver(handle);
    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    ZP10S_ClearRxBuffer(handle);

    status = ZP10S_SendCommand(handle, command);

    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    while ((HAL_GetTick() - start_tick) < handle->rx_timeout_ms)
    {
        uint32_t elapsed = HAL_GetTick() - start_tick;
        uint32_t remaining = handle->rx_timeout_ms - elapsed;

        status = ZP10S_ReadFrame(handle,
                                 response,
                                 response_len,
                                 (remaining == 0U) ? 1U : remaining);

        if (status != ZP10S_STATUS_OK)
        {
            return status;
        }

        /*
         * Some boards physically tie RX and TX together for the one-wire bus.
         * In that setup the MCU can receive its own command back before the
         * servo reply arrives, so we skip exact echoes here.
         */
        if (strcmp(response, command) == 0)
        {
            continue;
        }

        return ZP10S_STATUS_OK;
    }

    return ZP10S_STATUS_TIMEOUT;
}

static ZP10S_Status ZP10S_ParseUInt(const char *text,
                                    uint16_t max_digits,
                                    uint16_t *value,
                                    const char **end_ptr)
{
    uint32_t accumulator = 0U;
    uint16_t digits = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    while (ZP10S_IsDigit(*text) && (digits < max_digits))
    {
        accumulator = (accumulator * 10U) + (uint32_t)(*text - '0');
        ++text;
        ++digits;
    }

    if (digits == 0U)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    *value = (uint16_t)accumulator;

    if (end_ptr != NULL)
    {
        *end_ptr = text;
    }

    return ZP10S_STATUS_OK;
}

static ZP10S_Status ZP10S_ParseTenths(const char *text,
                                      uint16_t *value,
                                      const char **end_ptr)
{
    uint16_t integer_part = 0U;
    const char *cursor = NULL;
    ZP10S_Status status = ZP10S_ParseUInt(text, 4U, &integer_part, &cursor);

    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    if ((cursor == NULL) || (*cursor != '.') || !ZP10S_IsDigit(*(cursor + 1)))
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    *value = (uint16_t)(integer_part * 10U) + (uint16_t)(*(cursor + 1) - '0');

    if (end_ptr != NULL)
    {
        *end_ptr = cursor + 2;
    }

    return ZP10S_STATUS_OK;
}

static ZP10S_Status ZP10S_SendSimpleCommand(ZP10S_HandleTypeDef *handle,
                                            uint8_t id,
                                            const char *suffix)
{
    char command[ZP10S_CMD_BUFFER_LEN];

    if ((suffix == NULL) || (ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command, sizeof(command), "#%03u%s!", (unsigned int)id, suffix);
    return ZP10S_SendCommand(handle, command);
}

void ZP10S_Init(ZP10S_HandleTypeDef *handle, UART_HandleTypeDef *uart)
{
    if (handle == NULL)
    {
        return;
    }

    handle->uart = uart;
    handle->tx_timeout_ms = 20U;
    handle->rx_timeout_ms = 30U;
}

void ZP10S_SetTimeouts(ZP10S_HandleTypeDef *handle, uint32_t tx_timeout_ms, uint32_t rx_timeout_ms)
{
    if (handle == NULL)
    {
        return;
    }

    handle->tx_timeout_ms = tx_timeout_ms;
    handle->rx_timeout_ms = rx_timeout_ms;
}

ZP10S_Status ZP10S_Move(ZP10S_HandleTypeDef *handle, uint8_t id, uint16_t pulse, uint16_t time_ms)
{
    char command[ZP10S_CMD_BUFFER_LEN];

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) ||
        (pulse < ZP10S_MIN_PULSE_US) ||
        (pulse > ZP10S_MAX_PULSE_US) ||
        (time_ms > 9999U))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command,
                   sizeof(command),
                   "#%03uP%04uT%04u!",
                   (unsigned int)id,
                   (unsigned int)pulse,
                   (unsigned int)time_ms);

    return ZP10S_SendCommand(handle, command);
}

ZP10S_Status ZP10S_ReleaseTorque(ZP10S_HandleTypeDef *handle, uint8_t id)
{
    return ZP10S_SendSimpleCommand(handle, id, "PULK");
}

ZP10S_Status ZP10S_SetMode(ZP10S_HandleTypeDef *handle, uint8_t id, ZP10S_Mode mode)
{
    char command[ZP10S_CMD_BUFFER_LEN];

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) ||
        ((uint8_t)mode < (uint8_t)ZP10S_MODE_SERVO_270_CW) ||
        ((uint8_t)mode > (uint8_t)ZP10S_MODE_MOTOR_TIMED_CCW))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command, sizeof(command), "#%03uPMOD%u!", (unsigned int)id, (unsigned int)mode);
    return ZP10S_SendCommand(handle, command);
}

ZP10S_Status ZP10S_SetBaudRate(ZP10S_HandleTypeDef *handle, uint8_t id, ZP10S_BaudCode baud_code)
{
    char command[ZP10S_CMD_BUFFER_LEN];

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) ||
        ((uint8_t)baud_code < (uint8_t)ZP10S_BAUD_9600) ||
        ((uint8_t)baud_code > (uint8_t)ZP10S_BAUD_1000000))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command, sizeof(command), "#%03uPBD%u!", (unsigned int)id, (unsigned int)baud_code);
    return ZP10S_SendCommand(handle, command);
}

ZP10S_Status ZP10S_SetId(ZP10S_HandleTypeDef *handle, uint8_t current_id, uint8_t new_id)
{
    char command[ZP10S_CMD_BUFFER_LEN];

    if (ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK)
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command,
                   sizeof(command),
                   "#%03uPID%03u!",
                   (unsigned int)current_id,
                   (unsigned int)new_id);

    return ZP10S_SendCommand(handle, command);
}

ZP10S_Status ZP10S_Ping(ZP10S_HandleTypeDef *handle, uint8_t id)
{
    char command[ZP10S_CMD_BUFFER_LEN];
    char response[ZP10S_RESP_BUFFER_LEN];
    char expected[10];
    ZP10S_Status status;

    if (ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK)
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command, sizeof(command), "#%03uPID!", (unsigned int)id);
    (void)snprintf(expected, sizeof(expected), "#%03uP!", (unsigned int)id);

    status = ZP10S_Query(handle, command, response, sizeof(response));

    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    return (strcmp(response, expected) == 0) ? ZP10S_STATUS_OK : ZP10S_STATUS_PARSE_ERROR;
}

ZP10S_Status ZP10S_ReadPosition(ZP10S_HandleTypeDef *handle, uint8_t id, uint16_t *pulse)
{
    char command[ZP10S_CMD_BUFFER_LEN];
    char response[ZP10S_RESP_BUFFER_LEN];
    char expected_prefix[8];
    const char *cursor = NULL;
    ZP10S_Status status;

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) || (pulse == NULL))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command, sizeof(command), "#%03uPRAD!", (unsigned int)id);
    (void)snprintf(expected_prefix, sizeof(expected_prefix), "#%03uP", (unsigned int)id);

    status = ZP10S_Query(handle, command, response, sizeof(response));

    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    if (strncmp(response, expected_prefix, strlen(expected_prefix)) != 0)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    if (ZP10S_ParseUInt(response + strlen(expected_prefix), 4U, pulse, &cursor) != ZP10S_STATUS_OK)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    return ((cursor != NULL) && (*cursor == '!')) ? ZP10S_STATUS_OK : ZP10S_STATUS_PARSE_ERROR;
}

ZP10S_Status ZP10S_ReadMode(ZP10S_HandleTypeDef *handle, uint8_t id, ZP10S_Mode *mode)
{
    char command[ZP10S_CMD_BUFFER_LEN];
    char response[ZP10S_RESP_BUFFER_LEN];
    char expected_prefix[11];
    const char *cursor = NULL;
    uint16_t raw_mode = 0U;
    ZP10S_Status status;

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) || (mode == NULL))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command, sizeof(command), "#%03uPRTV!", (unsigned int)id);
    (void)snprintf(expected_prefix, sizeof(expected_prefix), "#%03uPMOD", (unsigned int)id);

    status = ZP10S_Query(handle, command, response, sizeof(response));

    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    if (strncmp(response, expected_prefix, strlen(expected_prefix)) != 0)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    if (ZP10S_ParseUInt(response + strlen(expected_prefix), 1U, &raw_mode, &cursor) != ZP10S_STATUS_OK)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    if ((cursor == NULL) ||
        (*cursor != '!') ||
        (raw_mode < (uint16_t)ZP10S_MODE_SERVO_270_CW) ||
        (raw_mode > (uint16_t)ZP10S_MODE_MOTOR_TIMED_CCW))
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    *mode = (ZP10S_Mode)raw_mode;
    return ZP10S_STATUS_OK;
}

ZP10S_Status ZP10S_ReadVersion(ZP10S_HandleTypeDef *handle, uint8_t id, char *version, size_t version_len)
{
    char command[ZP10S_CMD_BUFFER_LEN];
    char response[ZP10S_RESP_BUFFER_LEN];
    char expected_prefix[11];
    const char *payload = NULL;
    size_t payload_len = 0U;
    ZP10S_Status status;

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) || (version == NULL) || (version_len < 2U))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command, sizeof(command), "#%03uPVER!", (unsigned int)id);
    (void)snprintf(expected_prefix, sizeof(expected_prefix), "#%03uPV", (unsigned int)id);

    status = ZP10S_Query(handle, command, response, sizeof(response));

    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    if (strncmp(response, expected_prefix, strlen(expected_prefix)) != 0)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    payload = response + strlen(expected_prefix);
    payload_len = strlen(payload);

    if ((payload_len < 2U) || (payload[payload_len - 1U] != '!'))
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    payload_len -= 1U;

    if (payload_len >= version_len)
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)memcpy(version, payload, payload_len);
    version[payload_len] = '\0';

    return ZP10S_STATUS_OK;
}

ZP10S_Status ZP10S_ReadTelemetry(ZP10S_HandleTypeDef *handle, uint8_t id, ZP10S_Telemetry *telemetry)
{
    char command[ZP10S_CMD_BUFFER_LEN];
    char response[ZP10S_RESP_BUFFER_LEN];
    char expected_prefix[8];
    const char *cursor = NULL;
    ZP10S_Status status;

    if ((ZP10S_CheckHandle(handle) != ZP10S_STATUS_OK) || (telemetry == NULL))
    {
        return ZP10S_STATUS_INVALID_ARG;
    }

    (void)snprintf(command, sizeof(command), "#%03uPTEM!", (unsigned int)id);
    (void)snprintf(expected_prefix, sizeof(expected_prefix), "#%03uT", (unsigned int)id);

    status = ZP10S_Query(handle, command, response, sizeof(response));

    if (status != ZP10S_STATUS_OK)
    {
        return status;
    }

    if (strncmp(response, expected_prefix, strlen(expected_prefix)) != 0)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    cursor = response + strlen(expected_prefix);

    if (ZP10S_ParseTenths(cursor, &telemetry->temperature_deci_c, &cursor) != ZP10S_STATUS_OK)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    if ((cursor == NULL) || (*cursor != 'V'))
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    ++cursor;

    if (ZP10S_ParseTenths(cursor, &telemetry->voltage_deci_v, &cursor) != ZP10S_STATUS_OK)
    {
        return ZP10S_STATUS_PARSE_ERROR;
    }

    return ((cursor != NULL) && (*cursor == '!')) ? ZP10S_STATUS_OK : ZP10S_STATUS_PARSE_ERROR;
}
