/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Services/Motion/motion_service.h"
#include "app.h"
#include "board_config.h"
#include "Services/Safety/safety_service.h"
#include "Services/Log/log_service.h"
#include "Protocol/frame_defs.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_spi_error_count = 0;
volatile uint32_t g_spi_last_error = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//LOG_SVC_DEFINE(LOG_SVC_APP, "app");
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */
    board_config_apply_motion_gpio();
    board_config_force_encoder_quadrature();
    board_config_apply_interrupt_priorities();
    //board_config_apply_spi_dma_profile();
    app_init();
    // Inicia timers do laço de passos (TIM6) e controle/status (TIM7)
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    motion_demo_set_continuous(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //printf("oioioioioioi2\r\n");
    //HAL_Delay(1000);
    app_poll();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    app_spi_isr_txrx_done(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL) return;
    if (hspi->Instance != APP_SPI_INSTANCE) return;

    g_spi_last_error = hspi->ErrorCode;
    g_spi_error_count++;

    /* Indicação visual simples para diagnóstico */
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
}

/* Botões de segurança (EXTI):
 * - B1 (PC13): E-STOP imediato (pressionado = nível baixo)
 * - B2 (PC0): Release/recover + funções extras do demo (pressionado = baixo)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
    case GPIO_PIN_13: /* B1 - E-STOP */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            /* Pressionado: aciona E-STOP e para tudo agora */
            safety_estop_assert();
            motion_emergency_stop();
            /* Opcionalmente interrompe os timers para cessar qualquer atividade em ISR */
            HAL_TIM_Base_Stop_IT(&htim6);
            HAL_TIM_Base_Stop_IT(&htim7);
            /* Se houver PWM em TIM15 (LED/auxiliar), pare também */
            HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
        }
        break;
    case GPIO_PIN_0:  /* B2 - Release/Resume + demo speed step */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) {
            /* Libera segurança */
            safety_estop_release();
            /* Garante que os timers base voltem a rodar */
            HAL_TIM_Base_Start_IT(&htim6);
            HAL_TIM_Base_Start_IT(&htim7);
            /* Reativa movimentos conforme contexto */
            if (motion_demo_is_active()) {
                /* Cicla velocidade no modo demo contínuo */
                motion_demo_cycle_speed();
            } else {
                /* Se o demo estava desligado (ex.: após E-STOP), religa */
                motion_demo_set_continuous(1);
                /* Se usa PWM em TIM15 para indicação, retome */
                HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
            }
        }
        break;
    case GPIO_PIN_1:
    case GPIO_PIN_2:
    default:
        /* Reservado para sensores PROX/limites; sem ação específica aqui */
        break;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
#if LOG_ENABLE
	log_event_ids(LOG_SVC_APP, LOG_STATE_ERROR, -1);
	log_event_names("app", "error", "Error_Handler");
#endif
	__disable_irq();
	while (1) {
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
