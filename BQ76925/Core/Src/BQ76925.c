/*
 * BQ76925.c
 *
 *  Created on: May 30, 2026
 *      Author: danie
 */
#include "main.h"


#define I2C_ADDR		0b0100		// Address from datasheet but weird since only 2 MSBs sent

#define STATUS	 	  	0x00
#define CELL_CTL		0x01		// Select which cell for Vout
#define BAL_CTL			0x02		// Start balancing between two adjacent cells
#define CONFIG_1		0x03		// Current comparator and amplifier configuration
#define CONFIG_2		0x04		// CRC enable and Vref voltage level selection
#define POWER_CTL		0x05		// Sleep control and current, cell, thermistor, Vref enable
#define CHIP_ID			0x07		// Chip ID, RO
#define VREF_CAL		0x10		// Offset and gain correction for Vref
#define VC1_CAL			0x11		// Offset and gain correction for cell 1 reading
#define VC2_CAL			0x12		// Offset and gain correction for cell 2 reading
#define VC3_CAL			0x13		// Offset and gain correction for cell 3 reading
#define VC4_CAL			0x14		// Offset and gain correction for cell 4 reading
#define VC5_CAL			0x15		// Offset and gain correction for cell 5 reading
#define VC6_CAL		 	0x16		// Offset and gain correction for cell 6 reading
#define VC_CAL_EXT_1	0x17		// Offset and gain correction factor for cell 1 and 2
#define VC_CAL_EXT_2	0x18		// Offset and gain correction factor for cells 3, 4, 5, 6
#define VREF_CAL_EXT	0x1B		// Offset and gain correction factor for Vref



static void read_byte(I2C_HandleTypeDef *hi2c1, uint8_t reg, uint8_t *data) {
	uint8_t full_addr = (I2C_ADDR << 3 | reg) << 1;
	HAL_I2C_Mem_Read(hi2c1, full_addr, 0, 0, data, 1, 100);
}

static void write_byte(I2C_HandleTypeDef *hi2c1, uint8_t reg, uint8_t *data) {
	uint8_t full_addr = (I2C_ADDR << 3 | reg) << 1;
	HAL_I2C_Mem_Write(hi2c1, full_addr, 0, 0, data, 1, 100);
}

void get_cal_vals(I2C_HandleTypeDef *hi2c1, ADC_calibration_values cal_vals[]) {
	int i;
	uint8_t cal_MSB = 128;
	uint8_t cal_MSB_reg = VC_CAL_EXT_1;
	for(i = 1; i < 7; i++) {
		uint8_t cal_reg = 0;
		read_byte(hi2c1, VC1_CAL, &cal_reg);
		uint8_t cal_sign = 0;
		read_byte(hi2c1, cal_MSB_reg, &cal_sign);

	}
}

void init_BQ76925(I2C_HandleTypeDef *hi2c1, ADC_calibration_values cal_vals[]) {
	// Init

	// 2ms wait for I2C bootup
	HAL_Delay(2);

	// 1. Verify I2C is working, read status register and chip id
	uint8_t sensor_status = 0x00;
	read_byte(hi2c1, STATUS, &sensor_status);

	uint8_t chip_id = 0x00;
	read_byte(hi2c1, CHIP_ID, &chip_id);

	// NOTE: We should not use a CRC byte, for now, don't know if we need to disable this in the CONFIG_2 register
	// Additional note: Read automatically uses CRC byte, but can ignore

	// Get ADC Calibration values
	get_cal_vals(hi2c1, val_vals);
	uint8_t vref_cal = 0;
	uint8_t *ptr_data = &vref_cal;
	read_byte(hi2c1, VREF_CAL, ptr_data);


	// Set Vref voltage for 0.6 cell voltage gain

}

/* Test function for I2C, get rid of later
uint8_t test_read_reg(I2C_HandleTypeDef *hi2c1) {
	uint8_t data = 0;
	uint8_t *data_ptr = &data;
	read_byte(hi2c1, STATUS, data_ptr);
	return *data_ptr;
}
*/


// List of driver functions
// PRIVATE:
// read_byte()
// write_byte()

// PUBLIC:
// init()  --> Gets calibration values
// vref_to_vcout()
// measure_cell()
// enable_NTC()
// balance_cells()
// enter_sleep()



