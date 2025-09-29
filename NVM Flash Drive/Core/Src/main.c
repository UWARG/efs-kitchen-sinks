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
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04
#define WRITE 0x02
#define READ 0x03
#define READ_STATUS 0x05
#define PAGE_PROGRAM   0x02

#define FLAG_STATUS 0x70
#define CLEAR_FLAG 0x50

#define SUBSECTOR_ERASE_4K 0x20

#define RX_TX
#define RX_RX
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Chip Select helpers
#define FLASH_CS_PORT GPIOA
#define FLASH_CS_PIN  GPIO_PIN_4
static inline void CS_LOW(void)  { HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET); }
static inline void CS_HIGH(void) { HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);   }


// low-level helper functions
static inline void spi_tx1(uint8_t v) { HAL_SPI_Transmit(&hspi1, &v, 1, HAL_MAX_DELAY);}
static inline void spi_tx(const uint8_t *buf, uint16_t n) { HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, n, HAL_MAX_DELAY);}
static void spi_rx(uint8_t *buf, uint16_t n) { HAL_SPI_Receive(&hspi1, buf, n, 1000);}

// function to read the ID of the NVM chip with command 9f
HAL_StatusTypeDef readID(uint8_t id[20]){
	// read the ID
	uint8_t id_cmd = 0x9f;
	HAL_StatusTypeDef stat;
	CS_LOW();
	stat = HAL_SPI_Transmit(&hspi1, &id_cmd, 1, 100);
	if (stat == HAL_OK){
<<<<<<< HEAD
		stat = HAL_SPI_Receive(&hspi1, id, 4, 100);
=======
		stat = HAL_SPI_Receive(&hspi1, id, 20, HAL_MAX_DELAY);
>>>>>>> e2c82894c66124032745c271513f866d48bd483c
	}
	CS_HIGH();
	if (stat == HAL_OK) {
		// we expoect ID to be 0x20, 0xBA, 0x19
		printf("JEDEC: %02X %02X %02X\r\n", id[0], id[1], id[2]);    // id[0] = MFR, id[1] = TYPE, id[2] = CAP
	}
	else printf("coudn't get ID, error \r\n");
	return stat;
}


/*
 * function to read status register
 * bit0 (LSB) = write in progress: 1 when busy (write, program, or erase in progress), 0 when ready
 * bit1 = write latch, 0 when cleared, 1 when set
 */
static uint8_t read_sr(void) {
    uint8_t sr = 0, cmd = READ_STATUS;
    CS_LOW();
    spi_tx1(cmd);
    spi_rx(&sr, 1);
    CS_HIGH();
    return sr;
}


/*
 * function to read the flag status register
 * bit 8 (MSB) = status bit, 1 means ready, 0 means busy
 * this is different from the read status register, it gives more error codes
 */
static uint8_t read_fsr(void) {
    uint8_t fsr = 0, cmd = FLAG_STATUS;
    CS_LOW();
    spi_tx1(cmd);
    spi_rx(&fsr, 1);
    CS_HIGH();
    return fsr;
}

/*
 * function to clear the flag status register
 * when run into an error and want to try again, good to use this
 */
static void clear_fsr(void) {
    uint8_t cmd = CLEAR_FLAG;
    CS_LOW(); spi_tx1(cmd); CS_HIGH();
}



/*
 * function to enable writes, should call this before every write
 *
 * returns true = write is enabled
 */
static bool write_enable(void) {
    uint8_t cmd = WRITE_ENABLE;
    CS_LOW(); spi_tx1(cmd); CS_HIGH();
    // make sure that the write enable latch is set (bit 1 of the status register)
    return (read_sr() & 0x02) != 0;
}


/*
 * function to wait a specific time and keep checking the SR register
 * use this to poll status reg after performing a read/write/erase
 */
static HAL_StatusTypeDef wait_ready(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while(1) {
        if ((read_sr() & 0x01) == 0) {	// finished when bit 0 of status reg is reset
            uint8_t fsr = read_fsr();
            if (fsr & 0x60) { // erase/program error
                clear_fsr();
                return HAL_ERROR;
            }
            return HAL_OK;
        }
        if ((HAL_GetTick() - t0) > timeout_ms) return HAL_TIMEOUT;
    }
}

/*
 * function to erase a 4K sector
 * make sure to enable write before calling this
 * the least significant 12 bits of add24 will be ignored (need multiples of 4096)
 */
static HAL_StatusTypeDef erase_4k(uint32_t addr24) {
    if (!write_enable()) return HAL_ERROR;
    uint8_t cmd[4] = { SUBSECTOR_ERASE_4K,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    CS_LOW(); spi_tx(cmd, sizeof(cmd)); CS_HIGH();
    return wait_ready(3000); // 3 s is plenty for 4KB erase
}


/*
 * function to program data into an array
 * the 24 bit address can be anywhere in the range, but there are page alignment rules
 * no automatic wrap into the next page
 */
static HAL_StatusTypeDef page_program(uint32_t addr24, const uint8_t *data, uint16_t len) {
    if (len == 0 || len > 256) return HAL_ERROR;
    if (!write_enable()) return HAL_ERROR;

    // check to make sure alignment is correct (write at the start of a page)
    if (((addr24 & 0xFF) + len) > 256 ) return HAL_ERROR;

    uint8_t hdr[4] = { PAGE_PROGRAM,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    CS_LOW();
    spi_tx(hdr, sizeof(hdr));
    spi_tx(data, len);
    CS_HIGH();
    return wait_ready(10); // page program typically <1ms; 10ms guard
}


/*
 * function to read data
 * addr24 is where the data will start from, and increment
 */
static void read_data(uint32_t addr24, uint8_t *out, uint16_t len) {
    uint8_t hdr[4] = { READ,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    CS_LOW();
    spi_tx(hdr, sizeof(hdr));   // send cmd+addr
    spi_rx(out, len);           // then clock out data
    CS_HIGH();
}



/*
 * function to test writing data, then reading it back.
 * should do this with a power cycle as well
 */
void ReceiveTransmitTest(void)
{
    const uint32_t addr = 0x00000008; // 3B address
    const uint8_t tx[4] = {0xAA, 0xAA, 0xAA, 0xAA};
    uint8_t rx[4] = {0};

    uint32_t base4k = addr & ~0x0FFFU;


    // ---------------- ERASE ---------------------
    if (!write_enable()) {
         printf("write enable failed before erase\r\n");
        return;
    }
    if (erase_4k(base4k) != HAL_OK) {         // send 0x20, 3B address
         printf("unable to erase subsector\r\n");
        return;
    }
    if (wait_ready(3000) != HAL_OK) {         // poll SR.WIP=0, timeout
         printf("didn't receive ready after erase\r\n");
        return;
    }


    // ------------------- WRITE  ---------------------

    if (!write_enable()) {
         printf("unable to enable write after erasing\r\n");
        return;
    }

    if (page_program(addr, tx, sizeof(tx)) != HAL_OK) { // send 0x02 + data
         printf("unable to write to page \r\n");
        return;
    }

    if (wait_ready(10) != HAL_OK) {           // poll until program finishes
         printf("timeout after %d ms \r\n", 10);
        return;
    }

    // ---- READ BACK ----
    read_data(addr, rx, sizeof(rx));

    // ---- VERIFY ----
    if (memcmp(tx, rx, sizeof(tx)) == 0) {
        printf("PASS: data verified at 0x%06lX\n", (unsigned long)addr);
    } else {
        printf("FAIL: mismatch at 0x%06lX\n", (unsigned long)addr);
    }

    printf("END OF TEST FUNCTION\r\n");
    return;
}


















void txTest() {
	uint8_t address[3] = {0x00, 0x00, 0x01};
	uint8_t* write_cmd = malloc(4 * sizeof(uint8_t));		// shouldn't use malloc (also need free), because size is known
	write_cmd[0] = WRITE;
	memcpy(write_cmd + 1, address, 3 * sizeof(uint8_t));
	uint8_t* read_cmd = malloc(4 * sizeof(uint8_t));
	read_cmd[0] = READ;
	memcpy(read_cmd + 1, address, 3 * sizeof(uint8_t));

	uint8_t tx[4] = {0xA6, 0x13, 0x15, 0x1B};
	uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);		// much easier to use CS_LOW to clean things up
	HAL_SPI_Transmit(&hspi1, (uint8_t*) WRITE_ENABLE, 1, HAL_MAX_DELAY);		// after transmitting this, should check status reg
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, write_cmd, 4, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) WRITE_DISABLE, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


	// in this while loop, the CS pin shound't be set and reset each time, that could interrupt any operations
	// CS only needs to go high between commands
	// all that needs to be done is polling.

	uint8_t status;
	do {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) READ_STATUS, &status, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_Delay(100);
	} while (status & 0x01);

	HAL_Delay(300);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, read_cmd, rx, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}








void rxTest() {
	uint8_t address[3] = {0x00, 0x00, 0x01};

#ifdef RX_TX
	uint8_t tx[4] = {0xA6, 0x13, 0x15, 0x1B};
	uint8_t* write_cmd = malloc(4 * sizeof(uint8_t));
	write_cmd[0] = WRITE;
	memcpy(write_cmd + 1, address, 3 * sizeof(uint8_t));

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) WRITE_ENABLE, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, write_cmd, 4, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) WRITE_DISABLE, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	uint8_t status;
	do {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) READ_STATUS, &status, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_Delay(100);
	} while (status & 0x01);
#endif

#ifdef RX_RX
	uint8_t* read_cmd = malloc(4 * sizeof(uint8_t));
	read_cmd[0] = READ;
	memcpy(read_cmd + 1, address, 3 * sizeof(uint8_t));
	uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, read_cmd, rx, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
#endif
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_SPI1_Init();
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

<<<<<<< HEAD
  for (int i = 0; i < 5; i++){
	  BSP_LED_Toggle(LED_RED);
	  BSP_LED_Toggle(LED_BLUE);
  }
  printf("starting...");
  uint8_t id[3];
  HAL_StatusTypeDef st = readID(id);

  if (st == HAL_OK) {
	  if (id[0] == 0x20 && id[1] == 0xBA && id[2] == 0x19) {
		  printf("Flash OK (Micron N25Q256A)\r\n");
	  } else {
		  printf("Unexpected JEDEC: %02X %02X %02X\r\n", id[0], id[1], id[2]);
	  }
  } else {
	  printf("readID failed: %d\r\n", (int)st);
  }

  uint8_t sr = read_fsr();
  if (sr & 0x01) {
      // WIP=1 → busy
	  printf("busy \r\n");
  } else {
      // WIP=0 → ready
	  printf("ready\r\n");
  }

  ReceiveTransmitTest();




=======
//  txTest();
//  rxTest();
  uint8_t id[20] = {0};
>>>>>>> e2c82894c66124032745c271513f866d48bd483c

  while (1)
  {
	  printf("hello world\r\n");
	  HAL_Delay(1000);
	  readID(id);

	  // currently any rx is failing

	  // make sure to wait 1 ms for power up time befoe using the chip
<<<<<<< HEAD
	  printf("in while loop, done test \r\n");
	  BSP_LED_Toggle(LED_GREEN);
	  HAL_Delay(3000);

=======
>>>>>>> e2c82894c66124032745c271513f866d48bd483c
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
