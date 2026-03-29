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
	//3 axis raw data
	struct ist8310_raw_data {
		init16_t x;
		init16_t y;
		init16_t z;

		
	};


//
public:
	ist8310_i2c(/* args */);
	~ist8310_i2c();
};

ist8310_i2c::ist8310_i2c(/* args */)
{
}

ist8310_i2c::~ist8310_i2c()
{
}

