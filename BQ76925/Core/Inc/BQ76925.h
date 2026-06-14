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

typedef enum {
	VSS,
	CELL,
	VREF05,
	VREF085
} vcout_sel;

typedef enum {
	C1,
	C2,
	C3,
	C4,
	C5,
	C6,
	TEMP
} cells;

uint8_t test_read_reg(I2C_HandleTypeDef *hi2c1);
int init_BQ76925(I2C_HandleTypeDef *hi2c1, ADC_calibration_values cal_vals[]);
void vcout(I2C_HandleTypeDef *hi2c1, vcout_sel vcout_sel, cells cell_sel);


#endif /* INC_BQ76925_H_ */
