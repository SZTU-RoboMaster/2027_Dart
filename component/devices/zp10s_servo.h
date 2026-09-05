#ifndef __ZP10S_SERVO_H__
#define __ZP10S_SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include <stddef.h>
#include <stdint.h>

/*
 * ZP10S driver notes
 *
 * This driver targets Zhongling single-bus servos that use the ASCII command
 * protocol shown by public ZX/ZP family documentation, for example:
 *   #000P1500T1000!
 *   #000PRAD!
 *   #000PVER!
 *
 * The bus is typically used at 115200 baud.
 *
 * This project now configures USART1 in STM32 half-duplex mode, so the driver
 * automatically switches between transmitter and receiver before command send
 * and feedback read. If the same driver is reused on a normal UART, it still
 * works with the old "TX/RX tied together externally" arrangement as well.
 */

#define ZP10S_DEFAULT_ID        0U
#define ZP10S_BROADCAST_ID      255U

#define ZP10S_MIN_PULSE_US      500U
#define ZP10S_NEUTRAL_PULSE_US  1500U
#define ZP10S_MAX_PULSE_US      2500U

typedef enum
{
    ZP10S_STATUS_OK = 0,
    ZP10S_STATUS_ERROR = -1,
    ZP10S_STATUS_TIMEOUT = -2,
    ZP10S_STATUS_INVALID_ARG = -3,
    ZP10S_STATUS_PARSE_ERROR = -4
} ZP10S_Status;

typedef enum
{
    ZP10S_MODE_SERVO_270_CW = 1U,
    ZP10S_MODE_SERVO_270_CCW = 2U,
    ZP10S_MODE_SERVO_180_CW = 3U,
    ZP10S_MODE_SERVO_180_CCW = 4U,
    ZP10S_MODE_MOTOR_TURNS_CW = 5U,
    ZP10S_MODE_MOTOR_TURNS_CCW = 6U,
    ZP10S_MODE_MOTOR_TIMED_CW = 7U,
    ZP10S_MODE_MOTOR_TIMED_CCW = 8U
} ZP10S_Mode;

typedef enum
{
    ZP10S_BAUD_9600 = 1U,
    ZP10S_BAUD_19200 = 2U,
    ZP10S_BAUD_38400 = 3U,
    ZP10S_BAUD_57600 = 4U,
    ZP10S_BAUD_115200 = 5U,
    ZP10S_BAUD_128000 = 6U,
    ZP10S_BAUD_256000 = 7U,
    ZP10S_BAUD_1000000 = 8U
} ZP10S_BaudCode;

typedef struct
{
    /*
     * Temperature and voltage are stored in tenths to avoid float parsing
     * overhead in the MCU runtime.
     *
     * Example: 28.1 C -> 281
     *          7.4  V -> 74
     */
    uint16_t temperature_deci_c;
    uint16_t voltage_deci_v;
} ZP10S_Telemetry;

typedef struct
{
    UART_HandleTypeDef *uart;
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
} ZP10S_HandleTypeDef;

/* Bind the driver instance to a UART and load conservative default timeouts. */
void ZP10S_Init(ZP10S_HandleTypeDef *handle, UART_HandleTypeDef *uart);

/* Adjust polling timeouts for slower buses or longer cable runs. */
void ZP10S_SetTimeouts(ZP10S_HandleTypeDef *handle, uint32_t tx_timeout_ms, uint32_t rx_timeout_ms);

/* Move the target servo to a pulse width in the 500-2500 us range. */
ZP10S_Status ZP10S_Move(ZP10S_HandleTypeDef *handle, uint8_t id, uint16_t pulse, uint16_t time_ms);

/* Disable holding torque so the output shaft can move freely. */
ZP10S_Status ZP10S_ReleaseTorque(ZP10S_HandleTypeDef *handle, uint8_t id);

/* Change the servo working mode reported by the public Zhongling docs. */
ZP10S_Status ZP10S_SetMode(ZP10S_HandleTypeDef *handle, uint8_t id, ZP10S_Mode mode);

/* Change the internal servo baud rate code. Update MCU UART after using this. */
ZP10S_Status ZP10S_SetBaudRate(ZP10S_HandleTypeDef *handle, uint8_t id, ZP10S_BaudCode baud_code);

/* Rewrite the servo ID. Use broadcast or the current ID as needed. */
ZP10S_Status ZP10S_SetId(ZP10S_HandleTypeDef *handle, uint8_t current_id, uint8_t new_id);

/* Probe whether a servo with the given ID is currently online. */
ZP10S_Status ZP10S_Ping(ZP10S_HandleTypeDef *handle, uint8_t id);

/* Read the current feedback pulse width from the servo. */
ZP10S_Status ZP10S_ReadPosition(ZP10S_HandleTypeDef *handle, uint8_t id, uint16_t *pulse);

/* Read the current operating mode. */
ZP10S_Status ZP10S_ReadMode(ZP10S_HandleTypeDef *handle, uint8_t id, ZP10S_Mode *mode);

/* Read the firmware version string, for example "0.8". */
ZP10S_Status ZP10S_ReadVersion(ZP10S_HandleTypeDef *handle, uint8_t id, char *version, size_t version_len);

/* Read telemetry in tenths: 281 means 28.1 C, 74 means 7.4 V. */
ZP10S_Status ZP10S_ReadTelemetry(ZP10S_HandleTypeDef *handle, uint8_t id, ZP10S_Telemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* __ZP10S_SERVO_H__ */
