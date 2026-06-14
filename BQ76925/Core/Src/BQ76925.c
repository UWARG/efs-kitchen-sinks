/*
 * BQ76925.c
 *
 *  Created on: May 30, 2026
 *      Author: danie
 */
#include "main.h"
#include "BQ76925.h"

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
//	HAL_I2C_Mem_Read(hi2c1, full_addr, 0, 0, data, 1, 100);
	HAL_I2C_Master_Receive(hi2c1, full_addr, data, 1, 100);
}

static int write_byte(I2C_HandleTypeDef *hi2c1, uint8_t reg, uint8_t *data) {
	uint8_t full_addr = (I2C_ADDR << 3 | reg) << 1;
	HAL_I2C_Master_Transmit(hi2c1, full_addr, data, 1, 100);
	uint8_t verify = 0;
	read_byte(hi2c1, reg, &verify);
	if(verify == *data) {
		return 1;
	} else {
		return 0;
	}
}

void get_cal_vals(I2C_HandleTypeDef *hi2c1, ADC_calibration_values cal_vals[]) {
	uint8_t vref_cal_reg = 0;
	uint8_t vref_cal_ext_reg = 0;

	read_byte(hi2c1, VREF_CAL, &vref_cal_reg);
	read_byte(hi2c1, VREF_CAL_EXT, &vref_cal_ext_reg);
	uint8_t extra = vref_cal_ext_reg & 0b10;
	int8_t vref_offset = vref_cal_reg >> 4 | extra << 3;
	vref_offset = (vref_cal_ext_reg & 0x100) ? vref_offset*-1 : vref_offset;
	int8_t gain_offset = vref_cal_reg & 0xF;
	gain_offset = (vref_cal_ext_reg & 1) ? gain_offset*-1 : gain_offset;

	cal_vals[0].offset_corr = vref_offset;
	cal_vals[0].gain_corr = gain_offset;

	uint8_t calibration_register = VC1_CAL;
	int cal_iteration = 0;
	uint8_t cal_MSB =  1 << 7;

	int8_t offset = 0;
	int8_t gain = 0;

	uint8_t sign_reg_1 = 0;
	uint8_t sign_reg_2 = 0;
	read_byte(hi2c1, VC_CAL_EXT_1, &sign_reg_1);
	read_byte(hi2c1, VC_CAL_EXT_2, &sign_reg_2);
	uint8_t sign_reg = sign_reg_1;
	for(int i = 1; i < 7; i++) {
		uint8_t cal_reg = 0;
		read_byte(hi2c1, calibration_register + i - 1, &cal_reg);
		offset = cal_reg >> 4;
		offset = (sign_reg & (cal_MSB >> cal_iteration)) ? offset*-1 : offset;
		gain = cal_reg & 0xF;
		gain = (sign_reg & (cal_MSB >> (cal_iteration + 1))) ? gain*-1 : gain;
		cal_vals[i].offset_corr = offset;
		cal_vals[i].gain_corr = gain;
		cal_iteration += 2;
		calibration_register++;
		if(i == 2) {
			sign_reg = sign_reg_2;
			cal_iteration = 0;
		}
	}
}

static void enable_power(I2C_HandleTypeDef *hi2c1) {
	uint8_t power_reg = 0b111;	// Change back to 0b110, enable VREF for testing
	write_byte(hi2c1, POWER_CTL, &power_reg);
}

int init_BQ76925(I2C_HandleTypeDef *hi2c1, ADC_calibration_values cal_vals[]) {

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
	get_cal_vals(hi2c1, cal_vals);


	// Set Vref voltage for 0.6 cell voltage gain in CONFIG_2 register
	uint8_t vref_selection = 0;
	if(!write_byte(hi2c1, CONFIG_2, &vref_selection)) {
		return 0;
	}

	// Enable power to cell amplifier and thermistor
	enable_power(hi2c1);
}

void vcout(I2C_HandleTypeDef *hi2c1, vcout_sel vcout_sel, cells cell_sel) {
	uint8_t byte = vcout_sel << 4 | cell_sel;
	if(!write_byte(hi2c1, CELL_CTL, &byte)) {
		return;
	}

}

void balance_cell(I2C_HandleTypeDef *hi2c1, cells cell_sel, int enable) {
	uint8_t byte = 0;
	read_byte(hi2c1, BAL_CTL, &byte);
	if(enable) {
		byte |= 1 << cell_sel;
	} else {
		byte &= 0 << cell_sel;
	}
	write_byte(hi2c1, BAL_CTL, &byte);
}

void stop_balance(I2C_HandleTypeDef *hi2c1, cells cell_sel, int enable) {
	uint8_t byte = 0;
	write_byte(hi2c1, BAL_CTL, &byte);
}



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



