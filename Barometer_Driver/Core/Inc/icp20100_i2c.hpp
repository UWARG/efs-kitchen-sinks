#ifndef INC_ICP20100_I2C_HPP_
#define INC_ICP20100_I2C_HPP_

#include <stdio.h>
#include <stdint.h>
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_i2c.h"



#define ICP20100_I2C_ADDR (0x64 << 1) // i2c is 7 bit address, left shit by 1 to make 1 byte.

#define ICP20100_REG_MODE_SELECT 	0xC0
#define ICP20100_DEVICE_ID 			0x0C
#define ICP20100_MASTER_LOCK 		0xBE
#define ICP20100_OTP_CONFIG_1 		0xAC
#define ICP20100_OTP_STATUS			0xB9
#define ICP20100_OTP_STATUS2 		0xBF
#define ICP20100_VERSION_REG 		0xD3 // Version register
#define ICP20100_OTP_DBG2 			0xBC
#define ICP20100_OTP_MRA_LSB 		0xAF
#define ICP20100_OTP_MRA_MSB		0xB0
#define ICP20100_OTP_MRB_LSB 		0xB1
#define ICP20100_OTP_MRB_MSB		0xB2
#define ICP20100_OTP_MR_LSB 		0xAD
#define ICP20100_OTP_MR_MSB			0xAE
#define ICP20100_OTP_ADDRESS		0xB5
#define ICP20100_OTP_COMMAND		0xB6
#define ICP20100_OTP_RDATA 			0xB8
#define ICP20100_TRIM1_MSB 			0x05
#define ICP20100_TRIM2_LSB			0x06
#define ICP20100_TRIM2_MSB 			0x07


#define ICP20100_REG_MODE_SELECT_KEY 0x04
#define ICP20100_MASTER_UNLOCK_KEY 0x1F
#define ICP20100_OTP_ENABLE_BOTH 0x03
#define ICP20100_OTP_STATUS2_BOOTUP 0x01

#define ICP20100_POWER_MODE (1 << 2)
#define ICP20100_FORCED_MES_TRIGGER (1 << 4)

#define ICP20100_TRIGGER_COMMAND_MEAS (ICP20100_POWER_MODE | ICP20100_FORCED_MES_TRIGGER)

#define ICP20100_PRESS_DATA_0 0xFA

#define SCALE 131072.0f
#define Alt_min 70.0f
#define Alt_max 40.0f

class ICP20100{
	public:
		ICP20100(I2C_HandleTypeDef *hi2c);
		void initiateBarometer();
		void I2C_MemRxCallback();
		float readPressureDMA();
		float readPressureSequential();
		float readTemperatureDMA();
	private:
		I2C_HandleTypeDef *hi2c;
		uint8_t FIFO_REGISTER;
		uint8_t Press_Temp_Data[6];
		volatile bool dataFilled = 0;
		volatile uint8_t callbackCount; // Used to keep track for state machine
		volatile float latestPressurekPa = 0.0f;
		volatile float latestTemperatureC = 0.0f;
		bool initiatedRead = false;
		bool readRegister(uint16_t memAddress, uint8_t * pData, uint16_t size, I2C_HandleTypeDef *hi2c);
		bool writeRegister(uint16_t memAddress, uint8_t * pData, uint16_t size, I2C_HandleTypeDef *hi2c);

};
#endif /* INC_ICP20100_I2C_HPP_ */
