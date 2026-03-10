#include "icp20100_i2c.hpp"
#include "main.h"

#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_i2c.h"

ICP20100::ICP20100(I2C_HandleTypeDef *hi2c){
	//empty constructor
	this->hi2c = hi2c;
}

void ICP20100::transmit()
{
    uint8_t unlock = ICP20100_MASTER_UNLOCK_KEY;
    uint8_t status = 0;

    HAL_StatusTypeDef ret;

    HAL_StatusTypeDef ret2;

    HAL_StatusTypeDef ret3;

    ret = HAL_I2C_Mem_Write(
            hi2c,
            ICP20100_I2C_ADDR,
            ICP20100_MASTER_LOCK,
            I2C_MEMADD_SIZE_8BIT,
            &unlock,
            1,
            HAL_MAX_DELAY);

    ret2 = HAL_I2C_Mem_Write(
                hi2c,
                ICP20100_I2C_ADDR,
                ICP20100_MASTER_LOCK,
                I2C_MEMADD_SIZE_8BIT,
                &unlock,
                1,
                HAL_MAX_DELAY);

    ret3 = HAL_I2C_Mem_Read(
            hi2c,
            ICP20100_I2C_ADDR,
            ICP20100_VERSION_REG,
            I2C_MEMADD_SIZE_8BIT,
            &status,
            1,
            HAL_MAX_DELAY);
}

void ICP20100::initiateBarometer()
{
	uint8_t version = 0;
	uint8_t boot_status = 0;
	uint8_t unlock = ICP20100_MASTER_UNLOCK_KEY;
	uint8_t mode_select = 0;
	uint8_t otp_config = 0;
	//Write to lock register twice to get access to main registers and initiate communication w/ I2C
	HAL_I2C_Mem_Write(hi2c,
					  ICP20100_I2C_ADDR,
					  ICP20100_MASTER_LOCK,
					  I2C_MEMADD_SIZE_8BIT,
					  &unlock,
					  1,
					  HAL_MAX_DELAY);

	HAL_I2C_Mem_Write(hi2c,
						  ICP20100_I2C_ADDR,
						  ICP20100_MASTER_LOCK,
						  I2C_MEMADD_SIZE_8BIT,
						  &unlock,
						  1,
						  HAL_MAX_DELAY);

	// Check version
	HAL_I2C_Mem_Read(hi2c,
					 ICP20100_I2C_ADDR,
					 ICP20100_VERSION_REG,
					 I2C_MEMADD_SIZE_8BIT,
					 &version,
					 1,
					 HAL_MAX_DELAY);

	if(version == 0xB2){ // Initialization done if version B
		return;
	}

	// Check version
	HAL_I2C_Mem_Read(hi2c,
					 ICP20100_I2C_ADDR,
					 ICP20100_OTP_STATUS2_BOOTUP,
					 I2C_MEMADD_SIZE_8BIT,
					 &boot_status,
					 1,
					 HAL_MAX_DELAY);

	// Mask Boot Status
	boot_status &= (0x01);

	if(boot_status == 1){ // Initialization done, barometer did not go through power cycle.
		return;
	}

	// Bring ASIC into power mode, preserve other bits

	HAL_I2C_Mem_Read(hi2c,
					 ICP20100_I2C_ADDR,
					 ICP20100_REG_MODE_SELECT,
					 I2C_MEMADD_SIZE_8BIT,
					 &mode_select,
					 1,
					 HAL_MAX_DELAY);

	mode_select |= (0x04);

	HAL_I2C_Mem_Write(hi2c,
	                  ICP20100_I2C_ADDR,
	                  ICP20100_REG_MODE_SELECT,
	                  I2C_MEMADD_SIZE_8BIT,
	                  &mode_select,
	                  1,
	                  HAL_MAX_DELAY);

	HAL_Delay(4); // blocking delay 4, as required by data sheet

	//Enable OTP and write switch <-- step 7 NOT DONE


	// RESET = 0
	uint8_t reset = 0x00;

	//Toggle DBG2 register <-- step 8 NOT DONE

	HAL_I2C_Mem_Write(hi2c,
	                  ICP20100_I2C_ADDR,
	                  ICP20100_OTP_DBG2,
	                  I2C_MEMADD_SIZE_8BIT,
	                  &reset,
	                  1,
	                  HAL_MAX_DELAY);

	HAL_Delay(1);

}

float ICP20100::readPressure()
{
    uint8_t data[3];
    uint8_t unlock = ICP20100_MASTER_UNLOCK_KEY;
    uint8_t otp_enable = ICP20100_OTP_ENABLE_BOTH;
    uint8_t trigger = ICP20100_TRIGGER_COMMAND_MEAS;

    // unlock device
    HAL_I2C_Mem_Write(hi2c,
                      ICP20100_I2C_ADDR,
                      ICP20100_MASTER_LOCK,
                      I2C_MEMADD_SIZE_8BIT,
                      &unlock,
                      1,
                      HAL_MAX_DELAY);

    // enable OTP
    HAL_I2C_Mem_Write(hi2c,
                      ICP20100_I2C_ADDR,
                      ICP20100_OTP_CONFIG_1,
                      I2C_MEMADD_SIZE_8BIT,
                      &otp_enable,
                      1,
                      HAL_MAX_DELAY);

    // trigger measurement
    HAL_I2C_Mem_Write(hi2c,
                      ICP20100_I2C_ADDR,
                      ICP20100_REG_MODE_SELECT,
                      I2C_MEMADD_SIZE_8BIT,
                      &trigger,
                      1,
                      HAL_MAX_DELAY);

    HAL_Delay(10);   // conversion time

    // read pressure (3 bytes)
    HAL_I2C_Mem_Read(hi2c,
                     ICP20100_I2C_ADDR,
                     ICP20100_PRESS_DATA_0,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     3,
                     HAL_MAX_DELAY);

    uint32_t raw = ((uint32_t)data[0] << 16) |
                   ((uint32_t)data[1] << 8)  |
                   data[2];

    raw &= 0x000FFFFF;

    float pressure = raw / 4.0f;

    printf("%02X %02X %02X\r\n", data[0], data[1], data[2]);

    return pressure;
}

