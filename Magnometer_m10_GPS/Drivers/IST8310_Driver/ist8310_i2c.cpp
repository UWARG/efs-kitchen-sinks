/*
 * ist8310_i2c.cpp
 *
 *  Created on: Mar 1, 2026
 *      Author: jeong
 */
#include "ist8310_i2c.hpp"

void IST8310_Init(IST8310_Handle_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;

    uint8_t chip_id = 0;
    // 0x1C is the address (0x0E << 1)
    if (HAL_I2C_Mem_Read(dev->hi2c, 0x1C, 0x00, 1, &chip_id, 1, 100) == HAL_OK) {
        if (chip_id == 0x10) {
            // Connected to i2c
        }
    }
}

