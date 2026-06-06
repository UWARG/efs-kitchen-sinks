/*
 * BQ76925.h
 *
 *  Created on: May 30, 2026
 *      Author: danie
 */

#ifndef INC_BQ76925_H_
#define INC_BQ76925_H_

typedef struct {
	uint8_t offset_corr;
	uint8_t gain_corr;
} ADC_calibration_values;

uint8_t test_read_reg(I2C_HandleTypeDef *hi2c1);


#endif /* INC_BQ76925_H_ */
