/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"
#include "stm32l5xx_ll_ucpd.h"
#include "stm32l5xx_ll_bus.h"
#include "stm32l5xx_ll_cortex.h"
#include "stm32l5xx_ll_rcc.h"
#include "stm32l5xx_ll_system.h"
#include "stm32l5xx_ll_utils.h"
#include "stm32l5xx_ll_pwr.h"
#include "stm32l5xx_ll_gpio.h"
#include "stm32l5xx_ll_dma.h"

#include "stm32l5xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#ifndef PPM_SIG
	static const float MIN_RESET_PULSE = 3000.0f;//the signal for sync
	static const float MIN_PULSE_WIDTH = 700.0f;//the min pulse for 1
	static const float MAX_PULSE_WIDTH = 1670.0f;//the max pulse for a whole period
	static const float BASE_FREQUENCY = 48000000.0f;//the l552 operating frequency
	static const float SEC_TO_MICROSEC = 1000000.0f;//the factor to convert time
	static const float DOWN_INTERVAL = MAX_PULSE_WIDTH - MIN_PULSE_WIDTH;//the variable time from high to low
	static const uint8_t MAX_CHANNEL = 16;//the max number of channel
#endif /*PPM_SIG*/

#ifdef UNIT_TEST
#define HIGH_PERC 1.0f
#define MID_PERC 0.5f
#define LOW_PERC 0.0f
#define test_PSC 14U//the prescalar will be used for the test

/*please configure this*/
#define channel_used 10//note that the max is 16
#define channel_reserved channel_used + 1//the last one is for reset sync position
#endif /*UNIT_TEST*/

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/*Microsecs_to_counter convert a time interval measured in microsecs to counter for ccr and arr
 * arg1: time_length: how long the interval needs to be
 * arg2: psc: the prescalar value used by the current system
 */
uint32_t microsecs_to_counter(uint32_t time_length, uint16_t psc);
/*percentage_arr calculate the frame based on the percentage
 * percentage: the input value
 */
uint32_t percentage_arr(float percentage);
/*calculate the reset pause in this case is a constant 3000*/
uint32_t reset_sync(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_2
#define VBUS_SENSE_GPIO_Port GPIOC
#define UCPD_FLT_Pin GPIO_PIN_14
#define UCPD_FLT_GPIO_Port GPIOB
#define ST_LINK_VCP_TX_Pin GPIO_PIN_7
#define ST_LINK_VCP_TX_GPIO_Port GPIOG
#define ST_LINK_VCP_RX_Pin GPIO_PIN_8
#define ST_LINK_VCP_RX_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOA
#define UCPD_DBN_Pin GPIO_PIN_5
#define UCPD_DBN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
