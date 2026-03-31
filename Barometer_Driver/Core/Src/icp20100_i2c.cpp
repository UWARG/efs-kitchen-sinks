#include "icp20100_i2c.hpp"
#include "main.h"

#include <stdint.h>
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_i2c.h"

#define ICP20100_FIFO_FILL 0xC4
#define ICP20100_DEVICE_STATUS 0xCD
#define ICP20100_MODE_SYNC_STATUS_BIT 0x01

ICP20100::ICP20100(I2C_HandleTypeDef *hi2c){
	//empty constructor
	this->hi2c = hi2c;
	this->callbackCount = 0;
	this->FIFO_REGISTER = 0;
}

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
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS2, I2C_MEMADD_SIZE_8BIT, &boot_status, 1, HAL_MAX_DELAY) != HAL_OK){
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
	    err = HAL_TIMEOUT;
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
	timeout = 1000;
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
	    err = HAL_TIMEOUT;
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
	timeout = 1000;
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
		err = HAL_TIMEOUT;
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

	uint8_t boot_config = ICP20100_OTP_STATUS2_BOOTUP;

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS2, I2C_MEMADD_SIZE_8BIT, &boot_config, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return; }
}

bool ICP20100::readRegister(
                                uint16_t memAddress,
                                uint8_t * pData,
                                uint16_t size,
                                I2C_HandleTypeDef *hi2c) {

    return HAL_I2C_Mem_Read_DMA(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) == HAL_OK;
}

bool ICP20100::writeRegister(
                                uint16_t memAddress,
                                uint8_t * pData,
                                uint16_t size,
                                I2C_HandleTypeDef *hi2c) {

    return HAL_I2C_Mem_Write_DMA(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) == HAL_OK;
}

void ICP20100::I2C_MemRxCallback() {
	switch(callbackCount) {
		case 0: // Step 1: Start FIFO fill register read via DMA
			dataFilled = 0;
			if (readRegister(ICP20100_FIFO_FILL, &FIFO_REGISTER, 1, hi2c)) {
				callbackCount = 1;
			} else {
				callbackCount = 0;
			}
			break;

		case 1: // Step 2: FIFO read complete. If data ready, read pressure/temp burst.
			FIFO_REGISTER &= 0x1F;
			if (FIFO_REGISTER > 0) {
				if (readRegister(ICP20100_PRESS_DATA_0, Press_Temp_Data, 6, hi2c)) {
					callbackCount = 2;
				} else {
					callbackCount = 0;
				}
			} else {
				// Keep polling FIFO until at least one sample is ready.
				if (!readRegister(ICP20100_FIFO_FILL, &FIFO_REGISTER, 1, hi2c)) {
					callbackCount = 0;
				}
			}
			break;

		case 2: { // Step 3: Burst read complete. Convert and publish latest pressure.
			uint32_t press_raw = ((Press_Temp_Data[2] & 0x0F) << 16) | (Press_Temp_Data[1] << 8) | Press_Temp_Data[0];
			uint32_t temp_raw  = ((Press_Temp_Data[5] & 0x0F) << 16) | (Press_Temp_Data[4] << 8) | Press_Temp_Data[3];

			int32_t press_signed = (int32_t)(press_raw & 0xFFFFF);
			if (press_signed & 0x80000) {
				press_signed |= 0xFFF00000;
			}

			int32_t temp_signed = (int32_t)(temp_raw & 0xFFFFF);
			if (temp_signed & 0x80000) {
				temp_signed |= 0xFFF00000;
			}

			(void)temp_signed;
			latestPressurekPa = (float)(((double)press_signed * 40.0) / 131072.0 + 70.0);
			dataFilled = 1;
			callbackCount = 0;
			break;
		}

		default:
			callbackCount = 0;
			break;
	}
}

float ICP20100::readPressureDMA()
{
	if (callbackCount != 0) {
		return latestPressurekPa;
	}

	if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
		return latestPressurekPa;
	}

	// Datasheet: wait until DEVICE_STATUS.MODE_SYNC_STATUS == 1 before writing MODE_SELECT.
	uint8_t device_status = 0;
	uint32_t sync_timeout_ms = 10;
	while (sync_timeout_ms--) {
		if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_DEVICE_STATUS, I2C_MEMADD_SIZE_8BIT, &device_status, 1, 10) != HAL_OK) {
			return latestPressurekPa;
		}

		if ((device_status & ICP20100_MODE_SYNC_STATUS_BIT) != 0U) {
			break;
		}

		HAL_Delay(1);
	}

	if ((device_status & ICP20100_MODE_SYNC_STATUS_BIT) == 0U) {
		return latestPressurekPa;
	}

	// Trigger one forced conversion.
	uint8_t mode_cfg = 0x90; // 0b10010000: MEAS_CONFIG=4, FORCED_TRIGGER=1, MEAS_MODE=0, POWER_MODE=0
	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &mode_cfg, 1, 10) != HAL_OK) {
		return latestPressurekPa;
	}

	// Kick off DMA state machine. FIFO polling starts in callback step 1.
	I2C_MemRxCallback();

	// Non-blocking: returns last completed DMA-converted pressure.
	return latestPressurekPa;
}


float ICP20100::readPressureSequential()
{
    uint8_t press_data_1;
	uint8_t press_data_2;
	uint8_t press_data_3;
	uint8_t temp_data_1;
	uint8_t temp_data_2;
	uint8_t temp_data_3;
	// STEP 1: Poll FIFO register in FIFO field
		// 000000 in register field == empty.

	uint8_t mode_cfg = 0x0C; // 0b00001100: MEAS_CONFIG=4, FORCED_TRIGGER=1, MEAS_MODE=1, POWER_MODE=0
	HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, 0xC0, I2C_MEMADD_SIZE_8BIT, &mode_cfg, 1, HAL_MAX_DELAY);
	HAL_Delay(50); // wait for conversion (~50 ms for MODE4)

	uint8_t FIFO_REGISTER = 0;
	uint8_t err = 0;
	while(FIFO_REGISTER <= 0){
		if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_FIFO_FILL, I2C_MEMADD_SIZE_8BIT, &FIFO_REGISTER, 1, HAL_MAX_DELAY) != HAL_OK){
			err = HAL_I2C_GetError(hi2c); return 0;
		}

		// Mask first 3 bits
		FIFO_REGISTER &= (0x1F);
	}

	// STEP 2: Read out press data individually
	/*
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_PRESS_DATA_0, I2C_MEMADD_SIZE_8BIT, &press_data_1, 1, HAL_MAX_DELAY)!= HAL_OK){
		err = HAL_I2C_GetError(hi2c); return;
	}

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_PRESS_DATA_1, I2C_MEMADD_SIZE_8BIT, &press_data_2, 1, HAL_MAX_DELAY)!= HAL_OK){
		err = HAL_I2C_GetError(hi2c); return;
	}

	//LAST 4 BITS ARE GARBAGE
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_PRESS_DATA_2, I2C_MEMADD_SIZE_8BIT, &press_data_3, 1, HAL_MAX_DELAY)!= HAL_OK){
		err = HAL_I2C_GetError(hi2c); return;
	} 

	press_data_3 &= (0x0F); // Only care about the first 4 bits. 
	
	__uint32_t raw_pressure = ((press_data_3 & 0x0F) << 16) | (press_data_2 << 8) | press_data_1;
	__uint32_t pressure = (raw/2^17) * 40 + 70;

	// xxxx xxxx xxxx AAAAAAAA BBBBBBBB CCCC 

	// STEP 3: Read out temp data individually
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TEMP_DATA_0, I2C_MEMADD_SIZE_8BIT, &temp_data_1, 1, HAL_MAX_DELAY)!= HAL_OK){
		err = HAL_I2C_GetError(hi2c); return;
	}

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TEMP_DATA_1, I2C_MEMADD_SIZE_8BIT, &temp_data_2, 1, HAL_MAX_DELAY)!= HAL_OK){
		err = HAL_I2C_GetError(hi2c); return;
	}

	//LAST 4 BITS ARE GARBAGE
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TEMP_DATA_2, I2C_MEMADD_SIZE_8BIT, &temp_data_3, 1, HAL_MAX_DELAY)!= HAL_OK){
		err = HAL_I2C_GetError(hi2c); return;
	} 
	
	temp_data_3 &= (0x0F);
	__uint32_t raw_temp = ((temp_data_3 & 0x0F) << 16) | (temp_data_2 << 8) | temp_data_1;
	*/

	uint8_t buffer[6];
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_PRESS_DATA_0,
						I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY) != HAL_OK) {
		// error
	}
	uint32_t press_raw = ((buffer[2] & 0x0F) << 16) | (buffer[1] << 8) | buffer[0];
	uint32_t temp_raw  = ((buffer[5] & 0x0F) << 16) | (buffer[4] << 8) | buffer[3];

	// Step 4: Sign extend to
	int32_t press_signed = (int32_t)(press_raw & 0xFFFFF);          // Keep lower 20 bits
	if (press_signed & 0x80000) {          // If bit 19 is set (negative)
		press_signed |= 0xFFF00000;        // Sign extend to 32 bits
	}
	int32_t temp_signed = (int32_t)(temp_raw & 0xFFFFF);
	if (temp_signed & 0x80000) {
		temp_signed |= 0xFFF00000;
	}

	// Step 5: Convert to physical units
	// Pressure in kPa (or multiply by 10 for hPa, by 1000 for Pa)
	double press_kPa_int = ((double)press_signed * 40) / 131072 + 70;
	int32_t temp_C_int = ((int64_t)temp_signed * 65) / 262144 + 25;

	// For fractional results, you can scale by 100 to get 0.01°C resolution
	int32_t temp_C_100 = ((int64_t)temp_signed * 65 * 100) / 262144 + 2500;
	int32_t hi;

	return press_kPa_int
}