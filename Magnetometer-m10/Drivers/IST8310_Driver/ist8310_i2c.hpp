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
#include "stm32l5xx_it.h"

#define IST8310_I2C_ADDR_7BIT 0x0E
#define IST8310_I2C_ADDR (IST8310_I2C_ADDR_7Bit <<1) 

//identification
#define IST8310_REG_WAI 0x00 //reading connectivity
#define IST8310_WAI_VAL 0x10 //should output 0x10 when 0x00 is called
//status 1
#define IST8310_REG_STAT1 0x02 //register 1
#define IST8310_STAT1_DATA (1 << 0)
#define IST8310_STAT2_DOR (1 << 1)
//output data
#define IST8310_REG_DATA 0x03 //output data, X,Y,Z high and lows
//status 2
#define IST8310_REG_STAT2 0x09
#define IST8310_STAT2_INT (1 << 3)
//control
#define IST8310_REG_CNTL1 0x0A //register 1
#define IST8310_REG_CNTL2 0x0B //register 2
#define IST8310_CNTL1_STANDBY 0x00
#define IST8310_CNTL1_SINGLE 0x01
#define IST8310_CNTL2_SRST (1 << 0)
#define IST8310_CNTL2_DRP (1 << 2)
#define IST8310_CNTL2_DREN (1 << 3)
//self test
#define IST8310_REG_STR 0x0C
#define IST8310_STR_SELF_TEST 0x40
#define IST8310_STR_NORMAL 0x00
//config val
#define IST8310_AVGCNTL_16X 0x24
#define IST8310_PDCNTL_VAL 0xC0
//sensor characteristics
#define IST8310_RESOLUTION_UT_LSB 0.3f

class ist8310_i2c
{
private: 
	I2C_HandleTypeDef *_hi2c;
	uint16_t _addr;
	bool _initialized;

	//3 axis raw data
	struct ist8310_raw_data {
		init16_t x;
		init16_t y;
		init16_t z;
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
	HAL_StatusTypeDef i2c_transceive_IT(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size);

	//helper functions
	int writeReg(uint8_t reg, uint8_t value);
	int readReg(uint8_t reg, uint8_t *out);
	int readBytes(uint8_t reg, uint8_t *buf, uint16_t len);

public:
	ist8310_i2c(I2C_HandleTypeDef *hi2c);
	~ist8310_i2c();

	bool i2c_SM(); //start single measurement
	bool i2c_RM(); //read raw measurement
	bool i2c_RT(); //reset
	bool i2c_WR(uint8_t reg, uint8_t  val); //write register
	bool i2c_RR(uint8_t reg, uint8_t *out); //read register
	bool i2c_ST(); //self test

	bool i2c_init();
	bool i2c_set_averaging(uint8_t avg);
	bool i2c_set_drdy(bool enable);
	bool i2c_has_error();

	//status
	bool i2c_is_data_ready();
	bool i2c_is_over_field();
	bool i2c_read_wai();

	//data
	bool i2c_read_data();
	void decode();
	void convert();

	//flags
	bool get_drdy_flag();
	void set_update_flag(bool update);
	bool read_update_flag();
 
	//getters
	float get_x_data();
	float get_y_data();
	float get_z_data();
	float get_heading();
	int16_t get_raw_x();
	int16_t get_raw_y();
	int16_t get_raw_z();
	bool is_initialized();
};

ist8310_i2c::ist8310_i2c(I2C_HandleTypeDef *hi2c)
{
	_hi2c=hi2c;
	_addr=IST8310_I2C_ADDR;
	_initialized=false;
	_data_ready_flag=false;
	_update_flag=false;
}

ist8310_i2c::~ist8310_i2c()
{
}

