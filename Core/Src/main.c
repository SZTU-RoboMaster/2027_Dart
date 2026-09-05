/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "bsp_can.h"
#include "remote.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  CLOCK_ERROR_NONE = 0U,
  CLOCK_ERROR_HSE_START_TIMEOUT = 1U,
  CLOCK_ERROR_PLL_START_TIMEOUT = 2U,
  CLOCK_ERROR_PLL_STOP_TIMEOUT = 3U,
  CLOCK_ERROR_PLL_CONFIG_MISMATCH = 4U,
  CLOCK_ERROR_OSC_CONFIG_UNKNOWN = 5U,
  CLOCK_ERROR_CLOCK_CONFIG = 6U,
  CLOCK_ERROR_RCC_DEINIT = 7U
} ClockErrorCode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile ClockErrorCode g_clock_error_code = CLOCK_ERROR_NONE;
volatile HAL_StatusTypeDef g_clock_hal_status = HAL_OK;
volatile uint32_t g_clock_rcc_cr = 0U;
volatile uint32_t g_clock_rcc_cfgr = 0U;
volatile uint32_t g_clock_rcc_pllcfgr = 0U;
const char * volatile g_clock_error_message = "none";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Clock_SaveOscConfigError(HAL_StatusTypeDef status)
{
  g_clock_hal_status = status;
  g_clock_rcc_cr = RCC->CR;
  g_clock_rcc_cfgr = RCC->CFGR;
  g_clock_rcc_pllcfgr = RCC->PLLCFGR;

  if ((status == HAL_TIMEOUT) && ((g_clock_rcc_cr & RCC_CR_HSERDY) == 0U))
  {
    g_clock_error_code = CLOCK_ERROR_HSE_START_TIMEOUT;
    g_clock_error_message = "HSE did not become ready";
  }
  else if ((status == HAL_TIMEOUT) &&
           ((g_clock_rcc_cr & RCC_CR_PLLON) != 0U) &&
           ((g_clock_rcc_cr & RCC_CR_PLLRDY) == 0U))
  {
    g_clock_error_code = CLOCK_ERROR_PLL_START_TIMEOUT;
    g_clock_error_message = "PLL did not lock";
  }
  else if ((status == HAL_TIMEOUT) &&
           ((g_clock_rcc_cr & RCC_CR_PLLON) == 0U) &&
           ((g_clock_rcc_cr & RCC_CR_PLLRDY) != 0U))
  {
    g_clock_error_code = CLOCK_ERROR_PLL_STOP_TIMEOUT;
    g_clock_error_message = "PLL did not stop";
  }
  else if ((status == HAL_ERROR) &&
           ((g_clock_rcc_cfgr & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL))
  {
    g_clock_error_code = CLOCK_ERROR_PLL_CONFIG_MISMATCH;
    g_clock_error_message = "running PLL configuration does not match";
  }
  else
  {
    g_clock_error_code = CLOCK_ERROR_OSC_CONFIG_UNKNOWN;
    g_clock_error_message = "unknown oscillator configuration error";
  }

}

static HAL_StatusTypeDef Clock_RCC_OscConfig(const RCC_OscInitTypeDef *config)
{
  HAL_StatusTypeDef status = HAL_RCC_OscConfig(config);

  if (status != HAL_OK)
  {
    Clock_SaveOscConfigError(status);
  }

  return status;
}

static HAL_StatusTypeDef Clock_RCC_DeInit(void)
{
  HAL_StatusTypeDef status = HAL_RCC_DeInit();

  if (status != HAL_OK)
  {
    g_clock_hal_status = status;
    g_clock_error_code = CLOCK_ERROR_RCC_DEINIT;
    g_clock_error_message = "RCC reset before clock configuration failed";
    g_clock_rcc_cr = RCC->CR;
    g_clock_rcc_cfgr = RCC->CFGR;
    g_clock_rcc_pllcfgr = RCC->PLLCFGR;
  }

  return status;
}

static HAL_StatusTypeDef Clock_RCC_ClockConfig(const RCC_ClkInitTypeDef *config,
                                                uint32_t flash_latency)
{
  HAL_StatusTypeDef status = HAL_RCC_ClockConfig(config, flash_latency);

  if (status != HAL_OK)
  {
    g_clock_hal_status = status;
    g_clock_error_code = CLOCK_ERROR_CLOCK_CONFIG;
    g_clock_error_message = "system clock switch failed";
    g_clock_rcc_cr = RCC->CR;
    g_clock_rcc_cfgr = RCC->CFGR;
    g_clock_rcc_pllcfgr = RCC->PLLCFGR;
  }

  return status;
}

#define HAL_RCC_OscConfig(config) Clock_RCC_OscConfig(config)
#define HAL_RCC_DeInit() Clock_RCC_DeInit()
#define HAL_RCC_ClockConfig(config, flash_latency) \
  Clock_RCC_ClockConfig((config), (flash_latency))


void Rest_Init()
{
  HAL_GPIO_WritePin(TURN_R_GPIO_Port,TURN_R_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(TURN_L_GPIO_Port,TURN_L_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(PUSH_L_GPIO_Port,PUSH_L_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(PUSH_R_GPIO_Port,PUSH_L_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(YWA_GPIO_Port,YWA_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(Triger_GPIO_Port,Triger_Pin,GPIO_PIN_SET);


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* A bootloader/debug session may leave the MCU running from an old PLL.
     Return to the HSI reset state before changing PLL parameters. */
  if (HAL_RCC_DeInit() != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI5_Init();
  MX_TIM12_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  can_filter_init();
  Rest_Init();
  remote_control_init();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
