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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "safety_manager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile char gps_buffer[GPS_BUFFER_SIZE];
volatile uint16_t gps_index = 0;
volatile uint8_t sentence_ready = 0;
volatile uint8_t gps_fix_valid = 0; 
volatile uint8_t Safety_switch_PinState = 0; 
volatile uint16_t gps_fix_quality = 0; 

volatile uint8_t safety_switch_pressed = 0; 
volatile uint8_t safety_enabled = 0; 

volatile uint8_t buzzer_PinState = 1;
volatile uint32_t last_GPS_Update = 0;
volatile uint32_t nmea_total_count = 0;
volatile uint32_t nmea_gga_count = 0;
volatile uint32_t nmea_rmc_count = 0;
volatile uint32_t nmea_gsv_count = 0;
volatile uint32_t nmea_gll_count = 0;
volatile uint32_t nmea_gsa_count = 0;
volatile uint32_t nmea_other_count = 0;
volatile uint32_t nmea_last_report_ms = 0;
char nmea_last_type[4] = "???";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t gps_byte;
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
  MX_ICACHE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HAL_UART_Receive(&huart2, &gps_byte, 1, 5) == HAL_OK)
    {
      if (gps_byte == '\r')
      {
      }
      else if (gps_byte == '\n')
      {
        gps_buffer[gps_index] = '\0';
        gps_index = 0;
        sentence_ready = 1;
      }
      else
      {
        if (gps_index < GPS_BUFFER_SIZE - 1)
        {
          gps_buffer[gps_index++] = gps_byte;
        }
        else
        {
          gps_index = 0;
        }
      }
    }

    if (sentence_ready)
    {
      if (gps_buffer[0] == '$')
      {
        char type0 = gps_buffer[3];
        char type1 = gps_buffer[4];
        char type2 = gps_buffer[5];

        nmea_last_type[0] = type0;
        nmea_last_type[1] = type1;
        nmea_last_type[2] = type2;
        nmea_last_type[3] = '\0';
        nmea_total_count++;

        if (type0 == 'G' && type1 == 'G' && type2 == 'A')
        {
          nmea_gga_count++;
          uint16_t index_variable = 0;
          uint8_t comma_count = 0;
          while(gps_buffer[index_variable] != '\0')
          {
            if(gps_buffer[index_variable] == ','){
              comma_count++;
            } if(comma_count == 6)
            {
              gps_fix_quality = gps_buffer[index_variable + 1] - '0';
              break;
            }
            index_variable++;
          }
        }
        else if (type0 == 'R' && type1 == 'M' && type2 == 'C')
        {
          nmea_rmc_count++;
          uint16_t index_variable = 0;
          uint8_t comma_count = 0;

          while (gps_buffer[index_variable] != '\0')
          {
            if (gps_buffer[index_variable] == ',')
            {
              comma_count++;
              if (comma_count == 2)
              {
                if (gps_buffer[index_variable + 1] == 'A')
                {
                  gps_fix_valid = 1;
                  uint32_t time_now = HAL_GetTick();
                  last_GPS_Update = time_now;
                }
                else if (gps_buffer[index_variable + 1] == 'V')
                {
                  gps_fix_valid = 0;
                }
                break;
              }
            }
            index_variable++;
          }
        }
        else if (type0 == 'G' && type1 == 'S' && type2 == 'V')
        {
          nmea_gsv_count++;
        }
        else if (type0 == 'G' && type1 == 'L' && type2 == 'L')
        {
          nmea_gll_count++;
        }
        else if (type0 == 'G' && type1 == 'S' && type2 == 'A')
        {
          nmea_gsa_count++;
        }
        else
        {
          nmea_other_count++;
        }
      }
      sentence_ready = 0;
    }
    SafetyManagerInput_t safety_input;
    SafetyManagerOutput_t safety_output;

    Safety_switch_PinState = HAL_GPIO_ReadPin(GPS_Safety_SW_GPIO_Port, GPS_Safety_SW_Pin);

    if (Safety_switch_PinState == GPIO_PIN_SET)
    {
        safety_switch_pressed = 1;
    }
    else
    {
        safety_switch_pressed = 0;
    }

    safety_input.switch_pressed = safety_switch_pressed;
    safety_input.gps_fix_valid = gps_fix_valid;
    safety_input.gps_fix_quality = gps_fix_quality;
    safety_input.last_gps_update_ms = last_GPS_Update;
    safety_input.now_ms = HAL_GetTick();

    SafetyManager_Update(&safety_input, &safety_output);

    safety_enabled = safety_output.safety_enabled;


    uint32_t time = HAL_GetTick();
    if(time - last_GPS_Update > GPS_TIMEOUT_MS){
      gps_fix_valid = 0;
      gps_fix_quality = 0;
    }

    if (time - nmea_last_report_ms >= 1000U)
    {
      nmea_last_report_ms = time;
      printf("NMEA total=%lu GGA=%lu RMC=%lu GSV=%lu GLL=%lu GSA=%lu OTHER=%lu last=%s fix_valid=%u fix_quality=%u\r\n",
             (unsigned long)nmea_total_count,
             (unsigned long)nmea_gga_count,
             (unsigned long)nmea_rmc_count,
             (unsigned long)nmea_gsv_count,
             (unsigned long)nmea_gll_count,
             (unsigned long)nmea_gsa_count,
             (unsigned long)nmea_other_count,
             nmea_last_type,
             gps_fix_valid,
             gps_fix_quality);
    }


    if(safety_enabled == 0)
    {
      HAL_GPIO_WritePin(GPS_BUZZER_N_GPIO_Port, GPS_BUZZER_N_Pin, GPIO_PIN_SET );
      buzzer_PinState = 0;
    }
    else
    {
      HAL_GPIO_WritePin(GPS_BUZZER_N_GPIO_Port, GPS_BUZZER_N_Pin, GPIO_PIN_RESET);
      buzzer_PinState = 1;
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_BUZZER_N_GPIO_Port, GPS_BUZZER_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPS_BUZZER_N_Pin */
  GPIO_InitStruct.Pin = GPS_BUZZER_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_BUZZER_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_Safety_SW_Pin */
  GPIO_InitStruct.Pin = GPS_Safety_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_Safety_SW_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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

#ifdef  USE_FULL_ASSERT
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
