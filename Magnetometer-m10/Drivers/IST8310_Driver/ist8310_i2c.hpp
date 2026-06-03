/*
 * ist8310_i2c.hpp
 *
 *  Created on: Mar 1, 2026
 *      Author: jeong
 */

#ifndef IST8310_DRIVER_IST8310_I2C_HPP_
#define IST8310_DRIVER_IST8310_I2C_HPP_

#include "main.h"
#include <stdint.h>
#include <stddef.h>
#include "stm32l5xx_hal.h"

#define IST8310_I2C_ADDR_7BIT 0x0E
#define IST8310_I2C_ADDR (IST8310_I2C_ADDR_7BIT << 1)
//identification
#define IST8310_REG_WAI 0x00
#define IST8310_WAI_VAL 0x10
//status 1
#define IST8310_REG_STAT1 0x02
#define IST8310_STAT1_DRDY (1 << 0)
#define IST8310_STAT1_DOR (1 << 1)
//output data
#define IST8310_REG_DATA 0x03
//status 2
#define IST8310_REG_STAT2 0x09
#define IST8310_STAT2_INT (1 << 3)
//control
#define IST8310_REG_CNTL1 0x0A
#define IST8310_REG_CNTL2 0x0B
#define IST8310_CNTL1_STANDBY 0x00
#define IST8310_CNTL1_SINGLE 0x01
#define IST8310_CNTL2_SRST (1 << 0)
#define IST8310_CNTL2_DRP (1 << 2)
#define IST8310_CNTL2_DREN (1 << 3)
//self test
#define IST8310_REG_STR 0x0C
#define IST8310_STR_SELF_TEST 0x40
#define IST8310_STR_NORMAL 0x00
//config
#define IST8310_REG_TCCNTL 0x40
#define IST8310_REG_AVGCNTL 0x41
#define IST8310_REG_PDCNTL 0x42
#define IST8310_AVGCNTL_16X 0x24
#define IST8310_PDCNTL_VAL 0xC0
//sensor characteristics
#define IST8310_RESOLUTION_UT_LSB 0.3f

class ist8310_i2c
{
private: 
	I2C_HandleTypeDef *_hi2c;
	uint16_t _addr;

	//3 axis raw data
	struct ist8310_raw_data {
		int16_t x;
		int16_t y;
		int16_t z;
	};

	//converted data
	struct ist8310_converted_data {
		float heading;
		float x;
		float y;
		float z;
	};

	volatile struct ist8310_raw_data raw;
	volatile struct ist8310_converted_data converted;

	HAL_StatusTypeDef i2c_transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size);

public:
	ist8310_i2c(I2C_HandleTypeDef *hi2c);
	~ist8310_i2c();

	bool init();
	bool read();
 
	//getters
	float get_x_data();
	float get_y_data();
	float get_z_data();
	float get_heading();

	int16_t get_raw_x();
	int16_t get_raw_y();
	int16_t get_raw_z();
};



#endif
