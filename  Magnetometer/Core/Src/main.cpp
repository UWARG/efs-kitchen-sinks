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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mlx90393_i2c.hpp"
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
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float mag_x = -1;
float mag_y = -1;
float mag_z = -1;
float mag_norm = 0;  // calibrated |B| should be ~52-56 and stay there while rotating
float mag_raw_norm = 0;  // uncalibrated |B| for comparison

float hard_iron[3] = {0};
float axis_scale[3] = {0}; // soft-iron diagonal applied per axis

float cal_field_strength = 0;
float cal_radius[3] = {0}; // span each axis sees during rotation (~40-55)

uint32_t cal_samples = 0;
int cal_status = -1;  // -1 not run, 0 OK, 1 too few samples, 
                       // 2 poor rotation coverage, 3 singular fit, 4 bad fit
bool status = false;

volatile uint8_t recalibrate = 0; // set this to 1 in Live Expressions to re-run calibration

bool sensor_ok = false;

static void run_calibration(void)
{
	status = mlx90393.calibrate_4element(15000);

	hard_iron[0] = mlx90393.correction_factors.hard_iron[0];
	hard_iron[1] = mlx90393.correction_factors.hard_iron[1];
	hard_iron[2] = mlx90393.correction_factors.hard_iron[2];
	axis_scale[0] = mlx90393.correction_factors.soft_iron[0][0];
	axis_scale[1] = mlx90393.correction_factors.soft_iron[1][1];
	axis_scale[2] = mlx90393.correction_factors.soft_iron[2][2];
	cal_field_strength = mlx90393.correction_factors.field_strength;
	cal_radius[0] = mlx90393.cal_diag.radius[0];
	cal_radius[1] = mlx90393.cal_diag.radius[1];
	cal_radius[2] = mlx90393.cal_diag.radius[2];
	cal_samples = mlx90393.cal_diag.samples;
	cal_status = mlx90393.cal_diag.status;
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)&ch,1,HAL_MAX_DELAY);
	return ch;
}

//Magnetometer sender
//void send_MLX90393(void) {
//    uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_MAX_SIZE];
//
//    // put whatever you like in here for display in GUI
//    mlx90393_node.sensor_id = 1;
//    //1 uT = 0.01 Gauss, transfer in Gauss
//    mlx90393_node.magnetic_field_ga[0] = (float)mlx90393.get_x_data() / 100.0;
//    mlx90393_node.magnetic_field_ga[1] = (float)mlx90393.get_y_data() / 100.0;
//    mlx90393_node.magnetic_field_ga[2] = (float)mlx90393.get_z_data() / 100.0;
//    mlx90393_node.magnetic_field_covariance.len = 9;
//    //Covariance matrix with variance terms only, refer to 13.1 for noise standard deviation
//    //Settings: OSR = 3, digital filter = 5, conversion time = 52.92ms
//    //Standard deviation for XY-axis error = 5 mGauss, Z-axis = 7 mGauss
//    //Covariance is 0?
//    mlx90393_node.magnetic_field_covariance.data[0] = 2.5E-5;
//    mlx90393_node.magnetic_field_covariance.data[1] = 0.0;
//    mlx90393_node.magnetic_field_covariance.data[2] = 0.0;
//    mlx90393_node.magnetic_field_covariance.data[3] = 0.0;
//    mlx90393_node.magnetic_field_covariance.data[4] = 2.5E-5;
//    mlx90393_node.magnetic_field_covariance.data[5] = 0.0;
//    mlx90393_node.magnetic_field_covariance.data[6] = 0.0;
//    mlx90393_node.magnetic_field_covariance.data[7] = 0.0;
//    mlx90393_node.magnetic_field_covariance.data[8] = 4.9E-5;
//    uint32_t len = uavcan_equipment_ahrs_MagneticFieldStrength2_encode(&mlx90393_node, buffer);
//
//    // we need a static variable for the transfer ID. This is
//    // incremeneted on each transfer, allowing for detection of packet
//    // loss
//    static uint8_t transfer_id;
//
//    canardBroadcast(&canard,
//    				UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_SIGNATURE,
//					UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID,
//                    &transfer_id,
//                    CANARD_TRANSFER_PRIORITY_LOW,
//                    buffer,
//                    len);
//}




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
  MX_I2C3_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MLX90393 mlx90393(&hi2c3);
  sensor_ok = mlx90393.begin();

  if(sensor_ok)
  {
	  run_calibration();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(recalibrate)
	  {
		  recalibrate = 0;
		  run_calibration();
	  }

	  if(sensor_ok && mlx90393.i2c_read_data())
	  {
		  mag_x = mlx90393.get_x_calibrated();
		  mag_y = mlx90393.get_y_calibrated();
		  mag_z = mlx90393.get_z_calibrated();
		  mag_norm = sqrtf(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);

		  float rx = mlx90393.get_x_data();
		  float ry = mlx90393.get_y_data();
		  float rz = mlx90393.get_z_data();
		  mag_raw_norm = sqrtf(rx*rx + ry*ry + rz*rz);
	  }

	  HAL_Delay(100);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00100D14;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
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
