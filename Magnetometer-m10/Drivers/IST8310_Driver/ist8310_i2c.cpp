/*
 * ist8310_i2c.cpp
 *
 *  Created on: Mar 1, 2026
 *      Author: jeong
 */
#include "ist8310_i2c.hpp"
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_i2c.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


ist8310_i2c::ist8310_i2c(I2C_HandleTypeDef *hi2c)
{
	this -> _hi2c=hi2c;
	this -> _addr=IST8310_I2C_ADDR;
	this -> raw.x = 0;
	this -> raw.y = 0;
	this -> raw.z = 0;
	this -> converted.x = 0;
	this -> converted.y = 0;
	this -> converted.z = 0;
	this -> converted.heading = 0;
}

//I have no idea what this is for
ist8310_i2c::~ist8310_i2c()
{
}

HAL_StatusTypeDef ist8310_i2c::i2c_transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(this->_hi2c, this->	_addr, tx_data, tx_size, HAL_MAX_DELAY);
    if(status != HAL_OK){
        return status;
    }
    status = HAL_I2C_Master_Receive(this->_hi2c, this->_addr, rx_data, rx_size, HAL_MAX_DELAY);
    return status;
}

bool ist8310_i2c::init(){
    //check connectivity via who am I function
    uint8_t reg = IST8310_REG_WAI;
    uint8_t wai = 0;

    if(i2c_transceive(&reg, &wai, 1, 1) != HAL_OK)
    {
        return false;
    }

    if(wai != IST8310_WAI_VAL)
    {
        return false;
    }

    //reset
    uint8_t tx_rst[2] = {IST8310_REG_CNTL2, IST8310_CNTL2_SRST};
    if (i2c_transceive(tx_rst, nullptr, 2, 0) != HAL_OK){
        return false;
    }
    HAL_Delay(50);

    //16x averaging
    uint8_t tx_avg[2] = {IST8310_REG_AVGCNTL, IST8310_AVGCNTL_16X};
    if(i2c_transceive(tx_avg, nullptr, 2, 0) != HAL_OK)
    {
        return false;
    }

    //pulse duration
    uint8_t tx_pd[2] = {IST8310_REG_PDCNTL, IST8310_PDCNTL_VAL};
    if(i2c_transceive(tx_pd, nullptr, 2, 0) != HAL_OK){
        return false;
    }

    return true;
}

bool ist8310_i2c::read(){
    //single measurement
    uint8_t tx_sm[2] = {IST8310_REG_CNTL1, IST8310_CNTL1_SINGLE};
    if(i2c_transceive(tx_sm, nullptr, 2, 0) != HAL_OK){
		return false;
    }

    //pause for conversion
    HAL_Delay(7);

    //check the data
    uint8_t reg_stat = IST8310_REG_STAT1;
    uint8_t stat = 0;
    if(i2c_transceive(&reg_stat, &stat, 1, 1) != HAL_OK)
    {
        return false;
    }

    if(!(stat & IST8310_STAT1_DRDY)){
        return false;
    }

    if(!(stat & IST8310_STAT1_DRDY)){
        HAL_Delay(3);
        if(i2c_transceive(&reg_stat, &stat, 1, 1) != HAL_OK)
        {
            return false;
        }

        if(!(stat & IST8310_STAT1_DRDY))
        {
            return false;
        }
    }

    //read 6 bytes raw data
    uint8_t reg_data = IST8310_REG_DATA;
    uint8_t buf[6];
    if(i2c_transceive(&reg_data, buf, 1, 6) != HAL_OK)
    {
        return false;
    }

    //decode
    uint8_t *p = buf;
    this -> raw.x = (int16_t)((uint16_t)*(p + 1) << 8 | *(p + 0));
	this -> raw.y = (int16_t)((uint16_t)*(p + 3) << 8 | *(p + 2));
	this -> raw.z = (int16_t)((uint16_t)*(p + 5) << 8 | *(p + 4));

    //convert
    this -> converted.x = (float)this -> raw.x * IST8310_RESOLUTION_UT_LSB;
    this -> converted.y = (float)this -> raw.y * IST8310_RESOLUTION_UT_LSB;
    this -> converted.z = (float)this -> raw.z * IST8310_RESOLUTION_UT_LSB;

    //heading
    float heading_rad = atan2f(- this -> converted.y, this -> converted.x);
    float heading_deg = heading_rad * (180.0f / (float) M_PI);
    if(heading_deg < 0.0f)
    {
        heading_deg += 360.0f;
    }
    this -> converted.heading = heading_deg;
    return true;
}

float ist8310_i2c::get_x_data() {return this -> converted.x;}
float ist8310_i2c::get_y_data() {return this -> converted.y;}
float ist8310_i2c::get_z_data() {return this -> converted.z;}
float ist8310_i2c::get_heading() {return this -> converted.heading;}
int16_t ist8310_i2c::get_raw_x() {return this -> raw.x;}
int16_t ist8310_i2c::get_raw_y() {return this -> raw.y;}
int16_t ist8310_i2c::get_raw_z() {return this -> raw.z;}
 
