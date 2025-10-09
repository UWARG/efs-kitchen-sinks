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
//#include <stdio.h>
//#include <string.h>
//#include <stdbool.h>
//#include <math.h>
#include "MahonyAHRS.hpp"
#include "IMU.hpp"


#define CS_PIN GPIO_PIN_4
#define CS_GPIO_PORT GPIOA
//#define UB0_REG_WHO_AM_I 0x75
//#define REG_BANK_SEL  0x76
//#define UB0_REG_DEVICE_CONFIG  0x11
//#define UB0_REG_PWR_MGMT0 0x4E
//#define UB0_REG_TEMP_DATA1 0x1D
//
//
//
//int begin();
//int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
//void writeRegister(uint8_t subAddress, uint8_t data);
//int setBank(uint8_t bank);
//void setLowNoiseMode();
//void reset();
//uint8_t whoAmI();
//void AGT(uint8_t *dataBuffer);
//void setAccelFS(uint8_t fssel);

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

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
//int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
//
//	uint8_t tx = subAddress | 0x80; //
//	uint8_t dummy_tx[count]; //
//	memset(dummy_tx, 0, count*sizeof(dummy_tx[0]));
//	uint8_t dummy_rx;
//	HAL_StatusTypeDef ret;
//
//	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
//
//	ret = HAL_SPI_TransmitReceive(&hspi1, &tx, &dummy_rx, 1, HAL_MAX_DELAY);
//
//	ret = HAL_SPI_TransmitReceive(&hspi1, dummy_tx, dest, count, HAL_MAX_DELAY);
//
//	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET);
//
//	return 1;
//}
//
//void writeRegister(uint8_t subAddress, uint8_t data){
//	uint8_t tx_buf[2] = {subAddress, data};
//	HAL_StatusTypeDef ret;
//
//	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
//	ret = HAL_SPI_Transmit(&hspi1, tx_buf, 2, HAL_MAX_DELAY);
//	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET);
//
////	readRegisters(subAddress, 1, &dummy_rx);
////
////	if(dummy_rx == data) {
////		return 1;
////	  }
////	  else{
////		return -1;
////	  }
////
////	return 1;
//}
//
//
//
//int setBank(uint8_t bank){
//	writeRegister(REG_BANK_SEL , bank);
//	return 1;
//}
//
//void setLowNoiseMode(){
//	writeRegister(UB0_REG_PWR_MGMT0, 0x0F);
//}
//
//void reset(){
//	setBank(0);
//
//	writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);
//
//	HAL_Delay(1);
//}
//
//uint8_t whoAmI(){
//	uint8_t buffer;
//	readRegisters(UB0_REG_WHO_AM_I, 1, &buffer);
//	return buffer;
//}
//
//void AGT(uint8_t *dataBuffer){
//	readRegisters(UB0_REG_TEMP_DATA1, 14, dataBuffer);
//}
//
//
//float _gyroScale = 0;
//uint8_t current_fssel = 0;
//uint8_t _gyroFS = 0;
//float _gyroBD[3] = {0, 0, 0};
//float _gyrB[3] = {0, 0, 0};
//float _gyr[3] = {0, 0, 0};
//uint8_t gyroBuffer[14];
//int16_t rawMeasGyro[7];
//
//void setGyroFS(uint8_t fssel){
//
//	setBank(0);
//	uint8_t reg;
//	readRegisters(0x4F, 1, &reg);
//	reg = (fssel << 5) | (reg & 0x1F);
//	writeRegister(0x4F, reg);
//	_gyroScale = (2000.0f / (float)(1 << fssel)) / 32768.0f;
//	_gyroFS = fssel;
//}
//
//void calibrateGyro(){
//	const uint8_t current_fssel = _gyroFS;
//	setGyroFS(0x03);
//	_gyroBD[0] = 0;
//	_gyroBD[1] = 0;
//	_gyroBD[2] = 0;
//	for (size_t i=0; i < 1000; i++) {
//		AGT(gyroBuffer);
//		for (size_t i=0; i<7; i++) {
//			rawMeasGyro[i] = ((int16_t)gyroBuffer[i*2] << 8) | gyroBuffer[i*2+1];
//		}
//		for (size_t i=0; i<3; i++) {
//			_gyr[i] = (float)rawMeasGyro[i+4] / 16.4;
//		}
//		_gyroBD[0] += (_gyr[0] + _gyrB[0]) / 1000;
//		_gyroBD[1] += (_gyr[1] + _gyrB[1]) / 1000;
//		_gyroBD[2] += (_gyr[2] + _gyrB[2]) / 1000;
//		HAL_Delay(1);
//	}
//	_gyrB[0] = _gyroBD[0];
//	_gyrB[1] = _gyroBD[1];
//	_gyrB[2] = _gyroBD[2];
//	setGyroFS(current_fssel);
//}
//uint8_t _accelFS = 0;
//float _accelScale = 0.0;
//float _accBD[3] = {};
//uint8_t accelBuffer[14];
//float _acc[3] = {};
//float _accS[3] = {1.0f, 1.0f, 1.0f};
//float _accB[3] = {};
//int16_t rawMeasAccel[7];
//float _accMax[3] = {};
//float _accMin[3] = {};
//
//
//void setAccelFS(uint8_t fssel){
//	setBank(0);
//	uint8_t reg;
//	readRegisters(0x50, 1, &reg);
//	reg = (fssel << 5) | (reg & 0x1F);
//	writeRegister(0x50, reg);
//	_accelScale = (float)(1 << (4 - fssel)) / 32768.0f;
//	_accelFS = fssel;
//}
//
//void configureNotchFilter(){
//	uint8_t BW_SEL = 7;
//	uint32_t f_des = 1300;
//	double pi = 3.14159265;
//	double COSWZ = cos(2 * pi * f_des / 32);
//	int NF_COSWZ = 0;
//	bool NF_COSWZ_SEL = 0;
//	if(abs(COSWZ) <= 0.875){
//		NF_COSWZ_SEL = 0;
//		NF_COSWZ = round(COSWZ * 256);
//	}else{
//		NF_COSWZ_SEL = 1;
//		if(COSWZ > 0.875){
//			NF_COSWZ = round(8 * (1 - COSWZ) * 256);
//		}else{
//			NF_COSWZ = round(-8 * (1 + COSWZ) * 256);
//		}
//	}
//	setBank(1);
//	writeRegister(0x0F, (uint8_t)(NF_COSWZ & 0xFF));  // Lower byte for X-axis
//	writeRegister(0x10, (uint8_t)(NF_COSWZ & 0xFF));  // Lower byte for Y-axis
//	writeRegister(0x11, (uint8_t)(NF_COSWZ & 0xFF));  // Lower byte for Z-axis
//	writeRegister(0x12, (uint8_t)((NF_COSWZ >> 8) & 0x01));  // Upper bit for all axes
//
//	uint8_t reg_0x12;
//	readRegisters(0x12, 1, &reg_0x12);
//	// Modify only necessary bits (Bit 3 = X, Bit 4 = Y, Bit 5 = Z)
//	reg_0x12 &= ~(0b00111000);  // Clear bits 3, 4, 5
//	reg_0x12 |= (NF_COSWZ_SEL << 3) | (NF_COSWZ_SEL << 4) | (NF_COSWZ_SEL << 5);
//	writeRegister(0x12, reg_0x12);
//
//	// Set Notch Filter Bandwidth
//	writeRegister(0x13, BW_SEL << 4);
////	writeRegister(0x0f, (uint8_t)(NF_COSWZ & 0xff));
////	writeRegister(0x12, NF_COSWZ_SEL << 3);
////
////	writeRegister(0x10, (uint8_t)(NF_COSWZ & 0xff));
////	writeRegister(0x12, NF_COSWZ_SEL << 4);
////
////	writeRegister(0x11, (uint8_t)(NF_COSWZ & 0xff));
////	writeRegister(0x12, NF_COSWZ_SEL << 5);
////
////	writeRegister(0x13, BW_SEL << 4);
////	writeRegister(0x0b, 0x00);
//
//	setBank(0);
//}
//
//typedef struct {
//    uint16_t bandwidth;   // 3dB Bandwidth (Hz)
//    uint8_t delt;         // ACCEL_AAF_DELT / GYRO_AAF_DELT
//    uint16_t deltsqr;     // ACCEL_AAF_DELTSQR / GYRO_AAF_DELTSQR
//    uint8_t bitshift;     // ACCEL_AAF_BITSHIFT / GYRO_AAF_BITSHIFT
//} AAF_Config;
//
//static const AAF_Config aaf_table[] = {
//    {42,   1,    1,   15}, {84,   2,    4,   13}, {126,  3,    9,   12},
//    {170,  4,   16,   11}, {213,  5,   25,   10}, {258,  6,   36,   10},
//    {303,  7,   49,    9}, {348,  8,   64,    9}, {394,  9,   81,    9},
//    {441, 10,  100,    8}, {488, 11,  122,    8}, {536, 12,  144,    8},
//    {585, 13,  170,    8}, {634, 14,  196,    7}, {684, 15,  224,    7},
//    {734, 16,  256,    7}, {785, 17,  288,    7}, {837, 18,  324,    7},
//    {890, 19,  360,    6}, {943, 20,  400,    6}, {997, 21,  440,    6},
//    {1051,22,  488,    6}, {1107,23,  528,    6}, {1163,24,  576,    6},
//    {1220,25,  624,    6}, {1277,26,  680,    6}, {1336,27,  736,    5},
//    {1395,28,  784,    5}, {1454,29,  848,    5}, {1515,30,  896,    5},
//    {1577,31,  960,    5}, {1639,32, 1024,    5}, {1702,33, 1088,    5},
//    {1766,34, 1152,    5}, {1830,35, 1232,    5}, {1896,36, 1296,    5},
//    {1962,37, 1376,    4}, {2029,38, 1440,    4}, {2097,39, 1536,    4},
//    {2166,40, 1600,    4}, {2235,41, 1696,    4}, {2306,42, 1760,    4},
//    {2377,43, 1856,    4}, {2449,44, 1952,    4}, {2522,45, 2016,    4},
//    {2596,46, 2112,    4}, {2671,47, 2208,    4}, {2746,48, 2304,    4},
//    {2823,49, 2400,    4}, {2900,50, 2496,    4}, {2978,51, 2592,    4},
//    {3057,52, 2720,    4}, {3137,53, 2816,    3}, {3217,54, 2944,    3},
//    {3299,55, 3008,    3}, {3381,56, 3136,    3}, {3464,57, 3264,    3},
//    {3548,58, 3392,    3}, {3633,59, 3456,    3}, {3718,60, 3584,    3},
//    {3805,61, 3712,    3}, {3892,62, 3840,    3}, {3979,63, 3968,    3}
//};
//
//static const AAF_Config *getAAFConfig(uint16_t bandwidth) {
//    const AAF_Config *best = &aaf_table[0];
//    for (size_t i = 0; i < sizeof(aaf_table)/sizeof(aaf_table[0]); i++) {
//        if (aaf_table[i].bandwidth >= bandwidth) {
//            best = &aaf_table[i];
//            break;
//        }
//    }
//    return best;
//}
//
//void setAntiAliasFilter(uint16_t bandwidth_hz, bool accel_enable, bool gyro_enable) {
//    const AAF_Config *cfg = getAAFConfig(bandwidth_hz);
//
//    // accel
//    setBank(2);
//
//    uint8_t reg03;
//    readRegisters(0x03, 1, &reg03);
//    reg03 &= ~0x7E;                         // Clear bits 6:1
//    reg03 |= (cfg->delt & 0x3F) << 1;       // ACCEL_AAF_DELT
//    if (!accel_enable)
//        reg03 |= 1 << 0;                    // ACCEL_AAF_DIS = 1
//    else
//        reg03 &= ~(1 << 0);                 // Enable AAF
//    writeRegister(0x03, reg03);
//
//    writeRegister(0x04, (uint8_t)(cfg->deltsqr & 0xFF));  // Lower 8 bits of DELTSQR
//    uint8_t reg05;
//    readRegisters(0x05, 1, &reg05);
//    reg05 &= 0x00;                          // Clear bits 7:0
//    reg05 |= ((cfg->deltsqr >> 8) & 0x0F);  // Upper 4 bits of DELTSQR
//    reg05 |= (cfg->bitshift << 4) & 0xF0;   // ACCEL_AAF_BITSHIFT
//    writeRegister(0x05, reg05);
//
//    // gyro
//    setBank(1);
//
//    uint8_t reg0C;
//    readRegisters(0x0C, 1, &reg0C);
//    reg0C &= ~0x3F;                        // Clear bits 5:0
//    reg0C |= (cfg->delt & 0x3F);           // GYRO_AAF_DELT
//    writeRegister(0x0C, reg0C);
//
//    writeRegister(0x0D, (uint8_t)(cfg->deltsqr & 0xFF));  // Lower 8 bits
//    uint8_t reg0E;
//    readRegisters(0x0E, 1, &reg0E);
//    reg0E &= 0x00;                         // Clear bits
//    reg0E |= ((cfg->deltsqr >> 8) & 0x0F); // Upper 4 bits
//    reg0E |= (cfg->bitshift << 4) & 0xF0;  // GYRO_AAF_BITSHIFT
//    writeRegister(0x0E, reg0E);
//
//    uint8_t reg0B;
//    readRegisters(0x0B, 1, &reg0B);
//    if (!gyro_enable)
//        reg0B |= (1 << 1);                 // Disable Gyro AAF
//    else
//        reg0B &= ~(1 << 1);                // Enable Gyro AAF
//    writeRegister(0x0B, reg0B);
//
//    setBank(0);
//}
//
//int begin(){
//	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET);
//	reset();
//	uint8_t address = whoAmI();
//	setLowNoiseMode();
//	setAccelFS(0b01101001);
//	configureNotchFilter();
//	setAntiAliasFilter(213, true, true);
//	calibrateGyro();
//	return address;
//}
//
//float alpha = 0.1;  // Adjust filtering strength
//float filtered_gyro_x = 0;  // Store previous value for X-axis
//float filtered_gyro_y = 0;  // Store previous value for Y-axis
//float filtered_gyro_z = 0;  // Store previous value for Z-axis
//
//// Low-pass filter function
//float lowPassFilter(float raw_value, int select) {
//	if(select == 0){
//		filtered_gyro_x = alpha * raw_value + (1 - alpha) * filtered_gyro_x;
//		    return filtered_gyro_x;
//	}
//	if(select == 1){
//		filtered_gyro_y = alpha * raw_value + (1 - alpha) * filtered_gyro_y;
//		return filtered_gyro_y;
//	}
//	filtered_gyro_z = alpha * raw_value + (1 - alpha) * filtered_gyro_z;
//	return filtered_gyro_z;
//}
//
//
//uint8_t myBuffer[14];
//int16_t rawMeas[7];
//double accel[4]; // x,y,z,magnitude
//double gyro[3];

float test_roll = 0.0f;
float test_pitch = 0.0f;
float test_yaw = 0.0f;

float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_UCPD1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_UCPD1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
//  uint8_t address = begin();
////	printf("Gyro X: %.2f, Y: %.2f, Z: %.2f\r\n", gyro[0], gyro[1], gyro[2]);
//
//	uint8_t new_buffer[1];
//	uint8_t new_buffer_accel[1];
//
//
//	int sample = readRegisters(0x4F, 1, new_buffer);
//	int sample_accel = readRegisters(0x50, 1, new_buffer_accel);
  	IMU imu(&hspi1, GPIOA, GPIO_PIN_4);
	Mahony ahrs;
	uint8_t addr = imu.begin();

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
//	  AGT(myBuffer);
//	  	  for (size_t i=0; i<7; i++) {
//	  	      rawMeas[i] = ((int16_t)myBuffer[i*2] << 8) | myBuffer[i*2+1];
//	  	   }
//	  	  int16_t temperature = rawMeas[0] / 132.48f + 25;
//	  	  for (size_t i=0; i<3; i++) {
//	  		  accel[i] = (float)rawMeas[i+1] / 2048.0 * 9.81 / 2.0;
//	  	  }
//
//	  	  accel[3] = pow(((accel[0]*accel[0]) + (accel[1]*accel[1]) + (accel[2]*accel[2])), 0.5);
//
//	  	  for (size_t i=0; i<3; i++) {
//	  		  gyro[i] = lowPassFilter((float)rawMeas[i+4] / 16.4, i);
//	  	  }
//
//	  	      // Convert to physical units
//	  	  	  // ENU
//	  //	      ax = (float)accel[0];                    // g
//	  //	      ay = (float)accel[1];
//	  //	      az = (float)-accel[2];
//	  //	      gx = ((float)-gyro[0]);        // rad/s
//	  //	      gy = ((float)-gyro[1]);
//	  //	      gz = ((float)gyro[2]);
//
//	  	      // NED
//	  	      ax = (float)accel[1];                    // g
//	  		  ay = (float)accel[0];
//	  		  az = ((float)accel[2]);
//	  		  gx = ((float)-gyro[1]);        // rad/s
//	  		  gy = ((float)-gyro[0]);
//	  		  gz = ((float)-gyro[2]);
//
//	  		// Update Mahony filter
//	  		ahrs.updateIMU(gx, gy, gz, ax, ay, az);
//
//	  		// Get Euler angles
//	  		test_roll  = ahrs.getRoll();
//	  		test_pitch = ahrs.getPitch();
//	  		test_yaw   = ahrs.getYaw();
//	  	      HAL_Delay(50);
	  imu.getAccelGyro(ax, ay, az, gx, gy, gz);
	  ahrs.updateIMU(gx, gy, gz, ax, ay, az);
	  test_roll = ahrs.getRoll();
	  test_pitch = ahrs.getPitch();
	  test_yaw = ahrs.getYaw();
	  HAL_Delay(50);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  BSP_LED_Toggle(LED_GREEN);  // Toggle LED state
//	  HAL_Delay(500);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**UCPD1 GPIO Configuration
  PB15   ------> UCPD1_CC2
  PA15 (JTDI)   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
