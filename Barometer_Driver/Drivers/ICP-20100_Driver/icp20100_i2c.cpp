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
/*
void ICP20100::initiateBarometer()
{
	uint8_t version = 0x00;
	uint8_t boot_status = 0x00;
	uint8_t unlock = ICP20100_MASTER_UNLOCK_KEY;
	uint8_t lock = 0x00;
	uint8_t mode_select = 0x00;
	uint8_t otp_config = 0x00;
	uint8_t reset = 0x00;
	uint8_t redundant_read = 0x00;
	uint8_t command_address = 0x00;
	uint8_t status = 0x01;
	uint16_t offset = 0x0000;
	uint8_t gain = 0x0000;
	uint8_t HFosc = 0x0000;
	uint8_t Rdata = 0x0000;
	uint32_t err;

	//Write to lock register twice to get access to main registers and initiate communication w/ I2C
	if(HAL_I2C_Mem_Write(hi2c,
					  ICP20100_I2C_ADDR,
					  ICP20100_MASTER_LOCK,
					  I2C_MEMADD_SIZE_8BIT,
					  &unlock,
					  1,
					  HAL_MAX_DELAY)!= HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	if(HAL_I2C_Mem_Write(hi2c,
						  ICP20100_I2C_ADDR,
						  ICP20100_MASTER_LOCK,
						  I2C_MEMADD_SIZE_8BIT,
						  &unlock,
						  1,
						  HAL_MAX_DELAY)!= HAL_OK){
			err = HAL_I2C_GetError(hi2c);
			return;
	}

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

	// Check boot up status
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

	// Bring ASIC into power mode, preserve other bits <-- Step 5

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

	//Enable OTP and write switch <-- step 7
	HAL_I2C_Mem_Read(hi2c,
					ICP20100_I2C_ADDR,
					ICP20100_OTP_CONFIG_1,
					I2C_MEMADD_SIZE_8BIT,
					&otp_config,
					1,
					HAL_MAX_DELAY);

	otp_config |= (0x03);

	HAL_I2C_Mem_Write(hi2c,
					  ICP20100_I2C_ADDR, 	// Address to device
					  ICP20100_OTP_CONFIG_1, // Address to config peripheral in barometer
					  I2C_MEMADD_SIZE_8BIT, // 8 bit data packet
					  &otp_config,
					  1,					// Amount of data to be sent
					  HAL_MAX_DELAY);
	HAL_Delay(1); // should be wait 10 microseconds

	//Toggle DBG2 register STEP 8

	HAL_I2C_Mem_Read(hi2c,
					ICP20100_I2C_ADDR,
					ICP20100_OTP_DBG2,
					I2C_MEMADD_SIZE_8BIT,
					&reset,
					1,
					HAL_MAX_DELAY);

	reset |= (0x80);

	HAL_I2C_Mem_Write(hi2c,
	                  ICP20100_I2C_ADDR,
	                  ICP20100_OTP_DBG2,
	                  I2C_MEMADD_SIZE_8BIT,
	                  &reset,
	                  1,
	                  HAL_MAX_DELAY);
	HAL_Delay(1);

	reset &= ~(0x80);

	HAL_I2C_Mem_Write(hi2c,
		              ICP20100_I2C_ADDR,
		              ICP20100_OTP_DBG2,
		              I2C_MEMADD_SIZE_8BIT,
		              &reset,
		              1,
		              HAL_MAX_DELAY);

	HAL_Delay(1);

	// STEP 9 PROGRAM REDUNDANT READ

	redundant_read = 0x04;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRA_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRA_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY);

	redundant_read = 0x21;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRB_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY);

	redundant_read = 0x20;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRB_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY);

	redundant_read = 0x10;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MR_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY);

	redundant_read = 0x80;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MR_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY);

	// STEP 10 WRITE ADDRESS CONTENT AND READ COMMAND

	command_address = 0xF8;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);

	command_address = 0x00; //X0010000
	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);
	command_address &= ~(0x7F);
	command_address |= (0x10);
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);

	// STEP 11: Wait for OTP read to finish

	while(status != 0){
		HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
		HAL_Delay(1000);
	}

	// STEP 12: Read offset

	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &offset, 1, HAL_MAX_DELAY);

	// STEP 13: Write next address

	command_address = 0xF9;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);

	command_address = 0x00; //X0010000
	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);
	command_address &= ~(0x7F);
	command_address |= (0x10);
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);

	// STEP 14: Wait for OTP read to finish

	status = 1;

	while(status & 0x01){
	    HAL_I2C_Mem_Read(hi2c,
	                     ICP20100_I2C_ADDR,
	                     ICP20100_OTP_STATUS,
	                     I2C_MEMADD_SIZE_8BIT,
	                     &status,
	                     1,
	                     HAL_MAX_DELAY);
	}

	// STEP 15: Read gain

	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &gain, 1, HAL_MAX_DELAY);

	// Step 16: Write next address

	command_address = 0xFA;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);

	command_address = 0x00; //X0010000
	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);
	command_address &= ~(0x7F);
	command_address |= (0x10);
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY);

	// STEP 17: Wait for OTP read to finish

	while(status != 0){
		HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
		HAL_Delay(1000);
	}

	// STEP 18: Read HFosc

	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &HFosc, 1, HAL_MAX_DELAY);

	// STEP 19: Disable OTP

	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY);

	otp_config &= ~(0x03);

	HAL_I2C_Mem_Write(hi2c,ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY);
	HAL_Delay(1); // should be wait 10 microseconds

	// STEP 20: Write offset to main registers
	uint8_t trim_reg;
	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM1_MSB,
	                 I2C_MEMADD_SIZE_8BIT, &trim_reg, 1, HAL_MAX_DELAY);

	// Clear the 6-bit PEFE_OFFSET_TRIM field (bits 5:0)
	trim_reg &= ~0x3F;

	uint8_t offset_low = offset & 0x3F;   // extract lower 6 bits
	trim_reg |= offset_low;

	// Write back
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM1_MSB,
	                  I2C_MEMADD_SIZE_8BIT, &trim_reg, 1, HAL_MAX_DELAY);

	// STEP 21: Write gain to main registers

	HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_MSB,
					 I2C_MEMADD_SIZE_8BIT, &Rdata, 1, HAL_MAX_DELAY);

	Rdata &= ~(0b00111100);
	gain &= ~(0x07);
	Rdata |= (gain << 4);

	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_MSB,
		                  I2C_MEMADD_SIZE_8BIT, &Rdata, 1, HAL_MAX_DELAY);

	// STEP 22: Write HFosc trim value to main registers
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_LSB,
			                  I2C_MEMADD_SIZE_8BIT, &HFosc, 1, HAL_MAX_DELAY);

	// STEP 23: Lock main registers
	//Write to lock register twice to get access to main registers and initiate communication w/ I2C
	HAL_I2C_Mem_Write(hi2c,
						  ICP20100_I2C_ADDR,
						  ICP20100_MASTER_LOCK,
						  I2C_MEMADD_SIZE_8BIT,
						  &lock,
						  1,
						  HAL_MAX_DELAY);


	 // STEP 24: Move to standby
	uint8_t power_mode = 0;
	HAL_I2C_Mem_Read(hi2c,
		                  ICP20100_I2C_ADDR,
		                  ICP20100_REG_MODE_SELECT,
		                  I2C_MEMADD_SIZE_8BIT,
		                  &power_mode,
		                  1,
		                  HAL_MAX_DELAY);
	power_mode |= 0xF4;

	HAL_I2C_Mem_Write(hi2c,
	                  ICP20100_I2C_ADDR,
	                  ICP20100_REG_MODE_SELECT,
	                  I2C_MEMADD_SIZE_8BIT,
	                  &power_mode,
	                  1,
	                  HAL_MAX_DELAY);
	// STEP 25: Check boot up status to 1, avoid reintialization

	uint8_t boot_config = 1;
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS2_BOOTUP, I2C_MEMADD_SIZE_8BIT, &boot_config, 1, HAL_MAX_DELAY);

}*/

void ICP20100::initiateBarometer()
{
	uint32_t err;

	//Step 1: Power on ASIC

	//Step 2: Write to lock register twice to get access to main registers and initiate communication w/ I2C
	uint8_t unlock = ICP20100_MASTER_UNLOCK_KEY;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_MASTER_LOCK, I2C_MEMADD_SIZE_8BIT, &unlock, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_MASTER_LOCK, I2C_MEMADD_SIZE_8BIT, &unlock,1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	// Step 3: Read from the version register,
	uint8_t version = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_VERSION_REG, I2C_MEMADD_SIZE_8BIT, &version, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	if(version == 0xB2){ // Initialization done if version B
		return;
	}

	// Step 4: Check boot up status from OTP_Status2 register. Check specifically bit 0.
	uint8_t boot_status = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS2_BOOTUP, I2C_MEMADD_SIZE_8BIT, &boot_status, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	// Mask boot status register to only read the 0th bit
	boot_status &= (0x01);

	if(boot_status == 1){ // Initialization done, barometer did not go through power cycle.
		return;
	}

	// Step 5: Bring ASIC into power mode to get access to main registers
	// Set the 3rd bit of the mode_select register to 1.
	uint8_t mode_select = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &mode_select, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	mode_select |= (0x04); // Read previous register and toggle the 3rd bit to preserve previous bits

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &mode_select, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	HAL_Delay(4); // blocking delay 4, as required by data sheet

	// Step 6: Unlock main registers by setting the Master_Lock register to 0x1f

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_MASTER_LOCK, I2C_MEMADD_SIZE_8BIT, &unlock, 1, HAL_MAX_DELAY) != HAL_OK){
			err = HAL_I2C_GetError(hi2c);
			return;
	}


	//Step 7: Enable OTP and write switch by setting the config1 register's bits 0 and 1 to 1.
	uint8_t otp_config = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c); // Read register to preserve bits before setting
		return;
	}

	otp_config |= (0x03); // Sets bits 011

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	HAL_Delay(1); // should be wait 10 microseconds

	//Step 8: Toggle the OTP_DBG2 register bit 8 (reset bit)
	uint8_t reset = 0x00;

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_DBG2, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	reset |= (0x80);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_DBG2, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	HAL_Delay(1);

	reset &= ~(0x80);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_DBG2, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return;
	}

	HAL_Delay(1);

	// STEP 9: Program redundant read
	uint8_t redundant_read = 0x00;

	redundant_read = 0x04;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRA_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRA_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	redundant_read = 0x21;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRB_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	redundant_read = 0x20;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRB_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	redundant_read = 0x10;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MR_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	redundant_read = 0x80;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MR_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 10: Write address content and read command
	uint8_t command_address = 0x00;
	command_address = 0xF8;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	command_address = 0x00; //X0010000
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	command_address &= ~(0x7F);
	command_address |= (0x10);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 11: Wait for OTP read to finish
	uint8_t status = 1;
	int timeout = 1000;
	while((status & 0x01) && timeout--)
	{
	    if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR,
	        ICP20100_OTP_STATUS,
	        I2C_MEMADD_SIZE_8BIT,
	        &status,
	        1,
	        HAL_MAX_DELAY) != HAL_OK)
	    {
	        err = HAL_I2C_GetError(hi2c);
	        return;
	    }
	}

	if(timeout <= 0)
	{
	    err = OTP_TIMEOUT;
	    return;
	}

	// STEP 12: Read offset from the OTP_RDATA register
	uint8_t offset = 0x0000;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &offset, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 13: Write next address

	command_address = 0xF9;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	command_address = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	command_address &= ~(0x7F);
	command_address |= (0x10);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 14: Wait for OTP read to finish

	status = 1;
	int timeout = 1000;
	while((status & 0x01) && timeout--)
		{
		    if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR,
		        ICP20100_OTP_STATUS,
		        I2C_MEMADD_SIZE_8BIT,
		        &status,
		        1,
		        HAL_MAX_DELAY) != HAL_OK)
		    {
		        err = HAL_I2C_GetError(hi2c);
		        return;
		    }
		}

	if(timeout <= 0)
	{
	    err = OTP_TIMEOUT;
	    return;
	}

	// STEP 15: Read gain from OTP_RDATA register
	uint8_t gain = 0x0000;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &gain, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// Step 16: Write next address content

	command_address = 0xFA;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	command_address = 0x00; //X0010000
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	command_address &= ~(0x7F);
	command_address |= (0x10);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 17: Wait for OTP read to finish
	status = 1;
	int timeout = 1000;
	while((status & 0x01) && timeout--)
			{
			    if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR,
			        ICP20100_OTP_STATUS,
			        I2C_MEMADD_SIZE_8BIT,
			        &status,
			        1,
			        HAL_MAX_DELAY) != HAL_OK)
			    {
			        err = HAL_I2C_GetError(hi2c);
			        return;
			    }
			}

	if(timeout <= 0)
	{
		err = OTP_TIMEOUT;
		return;
	}

	// STEP 18: Read HFosc
	uint8_t HFosc = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &HFosc, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 19: Disable OTP

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	otp_config &= ~(0x03);

	if(HAL_I2C_Mem_Write(hi2c,ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	HAL_Delay(1); // should be wait 10 microseconds

	// STEP 20: Write offset to main registers
	uint8_t trim_reg;

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM1_MSB, I2C_MEMADD_SIZE_8BIT, &trim_reg, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// Clear the 6-bit PEFE_OFFSET_TRIM field (bits 5:0)
	trim_reg &= ~0x3F;

	uint8_t offset_low = offset & 0x3F;
	trim_reg |= offset_low;

	// Write back
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM1_MSB,
	                  I2C_MEMADD_SIZE_8BIT, &trim_reg, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 21: Write gain to main registers
	uint8_t Rdata = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_MSB,
					 I2C_MEMADD_SIZE_8BIT, &Rdata, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	Rdata &= ~(0b01110000);
	gain &= (0x07);
	Rdata |= (gain << 4);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_MSB,
		                  I2C_MEMADD_SIZE_8BIT, &Rdata, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 22: Write HFosc trim value to main registers
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_LSB,
			                  I2C_MEMADD_SIZE_8BIT, &HFosc, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 23: Lock main registers
	uint8_t lock = 0x00;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_MASTER_LOCK, I2C_MEMADD_SIZE_8BIT, &lock, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	 // STEP 24: Move to standby
	uint8_t power_mode = 0;

	if(HAL_I2C_Mem_Read(hi2c,
		                  ICP20100_I2C_ADDR,
		                  ICP20100_REG_MODE_SELECT,
		                  I2C_MEMADD_SIZE_8BIT,
		                  &power_mode,
		                  1,
		                  HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	power_mode &= ~(0x04);

	if(HAL_I2C_Mem_Write(hi2c,
	                  ICP20100_I2C_ADDR,
	                  ICP20100_REG_MODE_SELECT,
	                  I2C_MEMADD_SIZE_8BIT,
	                  &power_mode,
	                  1,
	                  HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

	// STEP 25: Check boot up status to 1, avoid reintialization

	uint8_t boot_config = 1;

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS2_BOOTUP, I2C_MEMADD_SIZE_8BIT, &boot_config, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }

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


bool ICP20100::selfTest(float &pressure_out)
{
    uint8_t boot_status = 0;
    uint8_t version = 0;
    uint8_t mode_select = 0;
    uint8_t data[3];

    /* ---------- Check BOOT STATUS ---------- */

    if(HAL_I2C_Mem_Read(hi2c,
                        ICP20100_I2C_ADDR,
                        ICP20100_OTP_STATUS2,
                        I2C_MEMADD_SIZE_8BIT,
                        &boot_status,
                        1,
                        HAL_MAX_DELAY) != HAL_OK)
        return false;

    if((boot_status & 0x01) == 0)
        return false;

    /* ---------- Check VERSION ---------- */

    if(HAL_I2C_Mem_Read(hi2c,
                        ICP20100_I2C_ADDR,
                        ICP20100_VERSION_REG,
                        I2C_MEMADD_SIZE_8BIT,
                        &version,
                        1,
                        HAL_MAX_DELAY) != HAL_OK)
        return false;

    if(version != 0xB2)
        return false;

    /* ---------- Trigger measurement ---------- */

    if(HAL_I2C_Mem_Read(hi2c,
                        ICP20100_I2C_ADDR,
                        ICP20100_REG_MODE_SELECT,
                        I2C_MEMADD_SIZE_8BIT,
                        &mode_select,
                        1,
                        HAL_MAX_DELAY) != HAL_OK)
        return false;

    mode_select |= (1 << 2);  // power mode
    mode_select |= (1 << 4);  // trigger measurement

    if(HAL_I2C_Mem_Write(hi2c,
                         ICP20100_I2C_ADDR,
                         ICP20100_REG_MODE_SELECT,
                         I2C_MEMADD_SIZE_8BIT,
                         &mode_select,
                         1,
                         HAL_MAX_DELAY) != HAL_OK)
        return false;

    HAL_Delay(10);  // wait conversion

    /* ---------- Read pressure ---------- */

    if(HAL_I2C_Mem_Read(hi2c,
                        ICP20100_I2C_ADDR,
                        ICP20100_PRESS_DATA_0,
                        I2C_MEMADD_SIZE_8BIT,
                        data,
                        3,
                        HAL_MAX_DELAY) != HAL_OK)
        return false;

    uint32_t raw =
        ((uint32_t)data[0] << 16) |
        ((uint32_t)data[1] << 8)  |
        data[2];

    raw &= 0x000FFFFF;   // 20-bit pressure field

    float pressure = raw / 4.0f;

    pressure_out = pressure;

    /* ---------- sanity check ---------- */

    if(pressure < 30000 || pressure > 120000)
        return false;

    return true;
}
