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
#include "stm32l5xx_it.h"

typedef struct {
	I2C_HandleTypeDef *hi2c; //pointer to i2c
	int16_t x;
	int16_t y;
	int16_t z;
} IST8310_Handle_t;

void IST_8310_init(IST8310_Handle_t *dev, IST8310_Handle_t *hi2c ){

};

uint8_t IST8310_ReadData(IST8310_Handle_t *dev) {

};

#endif /* IST8310_DRIVER_IST8310_I2C_HPP_ */
