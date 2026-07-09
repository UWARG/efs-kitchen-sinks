/*
 * mlx90393_i2c.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Henry
 */
#include "mlx90393_i2c.hpp"

#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_i2c.h"
#include <math.h>
#include "arm_math.h"

MLX90393::MLX90393(I2C_HandleTypeDef *hi2c){
	this->hi2c = hi2c;
	this->reg.gain = MLX90393_GAIN_1X;
	this->reg.x_res = MLX90393_RES_16;
	this->reg.y_res = MLX90393_RES_16;
	this->reg.z_res = MLX90393_RES_15;
	this->reg.hallconf = 0x0C;
	this->reg.tcmp_en = 0x00;
	this->reg.filter = 0x05;
	this->reg.osr = 0x03;
	this->raw.t = 0;
	this->raw.x = 0;
	this->raw.y = 0;
	this->raw.z = 0;
	this->converted.t = 0;
	this->converted.x = 0;
	this->converted.y = 0;
	this->converted.z = 0;
	this->zyxt = 0x0E;
	this->rm_flag = false;
	this->mes_updated = true;
	this->wait_flag = false;
	this->correction_factors.soft_iron[0][0] = 1;
	this->correction_factors.soft_iron[1][1] = 1;
	this->correction_factors.soft_iron[2][2] = 1;
	this->correction_factors.hard_iron[0] = 0;
	this->correction_factors.hard_iron[1] = 0;
	this->correction_factors.hard_iron[2] = 0;
}

bool MLX90393::begin(){
	HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_SET);
	if(!i2c_EX()) return false;
	if(!i2c_RT()) return false;
	HAL_Delay(2); // settle after reset
	if(!i2c_set_resolution(MLX90393_RES_16, MLX90393_RES_16, MLX90393_RES_15)) return false;
	if(!i2c_set_oversampling(0x03)) return false;
	if(!i2c_set_filter(0x05)) return false;
	return true;
}

HAL_StatusTypeDef MLX90393::i2c_transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(hi2c, DEFAULT_I2C_ADDRESS, tx_data, tx_size, HAL_MAX_DELAY);
	if(status != HAL_OK){
		return status;
	}
	status = HAL_I2C_Master_Receive(hi2c, DEFAULT_I2C_ADDRESS, rx_data, rx_size, HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef MLX90393::i2c_transceive_IT(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(hi2c, DEFAULT_I2C_ADDRESS, tx_data, tx_size, HAL_MAX_DELAY);
	if(status != HAL_OK){
		return status;
	}
	status = HAL_I2C_Master_Receive_DMA(hi2c, DEFAULT_I2C_ADDRESS, rx_data, rx_size);
	return status;
}

bool MLX90393::i2c_SM()
{
	uint8_t tx_data = (uint8_t)CMD_START_MEASUREMENT | this->zyxt;
	uint8_t buf = 0x00;
	if(i2c_transceive(&tx_data, &buf, 1, 1) != HAL_OK){
		return false;
	}
	this->reg.stat = buf;
	return true;
}

bool MLX90393::i2c_RM(){
	this->rm_flag = true;
	this->mes_updated = false;
	this->wait_flag = true;
	uint8_t tx_data = (uint8_t)CMD_READ_MEASUREMENT | this->zyxt;
	if(i2c_transceive_IT(&tx_data, this->rx_data, 1, 7) != HAL_OK){
		return false;
	}
	return true;
}

bool MLX90393::i2c_EX(){
	uint8_t tx_data = CMD_EXIT;
	uint8_t buf = 0x00;
	if(i2c_transceive(&tx_data, &buf, 1, 1) != HAL_OK){
		return false;
	}
	this->reg.stat = buf;
	return true;
}

bool MLX90393::i2c_RT(){
	uint8_t tx_data = CMD_RESET;
	uint8_t buf = 0x00;
	if(i2c_transceive(&tx_data, &buf, 1, 1) != HAL_OK){
		return false;
	}
	this->reg.stat = buf;
	return true;
}

bool MLX90393::i2c_WR(uint8_t regNum, uint16_t tx_data){
	uint8_t tx[4] = {CMD_WRITE_REGISTER, (uint8_t)(tx_data >> 8), (uint8_t)(tx_data & 0xFF), (uint8_t)regNum << 2};
	uint8_t buf = 0x00;
	if(i2c_transceive(tx, &buf, 4, 1) != HAL_OK){
		return false;
	}
	this->reg.stat = buf;
	return true;
}
bool MLX90393::i2c_RR(uint8_t regNum){
	uint8_t tx_data[2] = {CMD_READ_REGISTER, (uint8_t)regNum << 2};
	uint8_t rx_data[3];
	if(i2c_transceive(tx_data, rx_data, 2, 3) != HAL_OK){
		return false;
	}
	this->reg.stat = rx_data[0];
	this->reg.val = rx_data[1] << 8 | rx_data[2];
	return true;
}

bool MLX90393::i2c_set_gain(uint8_t gain){
	if(!i2c_RR(MLX90393_CONF1)){
		return false;
	}
	uint16_t data = this->reg.val & ~MLX90393_GAIN_MASK;
	data |= gain << MLX90393_GAIN_SHIFT;
	this->reg.gain = gain;
	return(i2c_WR(MLX90393_CONF1, data));
}

bool MLX90393::i2c_get_gain(){
	if(!i2c_RR(MLX90393_CONF1)){
		return false;
	}
	uint16_t data = reg.val & MLX90393_GAIN_MASK;
	this->reg.gain = data >> MLX90393_GAIN_SHIFT;
	return true;
}

bool MLX90393::i2c_set_resolution(uint8_t x_res, uint8_t y_res, uint8_t z_res){
	if(!i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Res 2 and 3 not allowed if temperature compensation enabled. See 16.2.10
	if(this->reg.tcmp_en == 1){
		if(x_res == 2 || x_res == 3 || y_res == 2 || y_res == 3 || z_res == 2 || z_res == 3){
			return false;
		}
	}
	this->reg.x_res = x_res;
	this->reg.y_res = y_res;
	this->reg.z_res = z_res;
	uint16_t data = this->reg.val;
	if(x_res != 0){
		data = (data & ~MLX90393_X_RES_MASK) | (x_res << MLX90393_X_RES_SHIFT);
	}
	if(y_res != 0){
		data = (data & ~MLX90393_Y_RES_MASK) | (y_res << MLX90393_Y_RES_SHIFT);
	}
	if(z_res != 0){
		data = (data & ~MLX90393_Z_RES_MASK) | (z_res << MLX90393_Z_RES_SHIFT);
	}
	return(i2c_WR(MLX90393_CONF3, data));
}


bool MLX90393::i2c_get_resolution(){
	if(i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = this->reg.val;
	this->reg.x_res = (data & MLX90393_X_RES_MASK) >> MLX90393_X_RES_SHIFT;
	this->reg.y_res = (data & MLX90393_Y_RES_MASK) >> MLX90393_Y_RES_SHIFT;
	this->reg.z_res = (data & MLX90393_Z_RES_MASK) >> MLX90393_Z_RES_SHIFT;
	return true;
}

bool MLX90393::i2c_set_filter(uint8_t filter){
	if(!i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Not permitted settings see 16.2.5
	if(this->reg.hallconf == 0x0C){
		if(this->reg.osr == 0x00){
			if(filter == 0 || filter == 1){
				return false;
			}
		}
		if(this->reg.osr == 0x01){
			if(filter == 1){
				return false;
			}
		}
	}
	this->reg.filter = filter;
	uint16_t data = this->reg.val;
	data = (data & ~MLX90393_FILTER_MASK) | (filter << MLX90393_FILTER_SHIFT);
	return(i2c_WR(MLX90393_CONF3, data));

}
bool MLX90393::i2c_get_filter(){
	if(!i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = this->reg.val;
	this->reg.filter = (data & MLX90393_FILTER_MASK) >> MLX90393_FILTER_SHIFT;
	return true;
}
bool MLX90393::i2c_set_oversampling(uint8_t osr){
	if(!i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Not permitted settings see 16.2.5
	if(this->reg.hallconf == 0x0C){
		if(this->reg.filter == 0x00){
			if(osr == 0 || osr == 1){
				return false;
			}
		}
		if(this->reg.hallconf == 0x01){
			if(osr == 1){
				return false;
			}
		}
	}
	this->reg.osr = osr;
	uint16_t data = this->reg.val;
  data = (data & ~MLX90393_OSR_MASK) | (this->reg.osr << MLX90393_OSR_SHIFT);
	return(i2c_WR(MLX90393_CONF3, data));
}

bool MLX90393::i2c_get_oversampling(){
	if(!i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = this->reg.val;
	this->reg.osr = (data & MLX90393_OSR_MASK) >> MLX90393_OSR_SHIFT;
	return true;
}

bool MLX90393::i2c_has_error(){
	return (this->reg.stat & ERROR_BIT) != 0;
}

bool MLX90393::i2c_read_data(){
	if(!i2c_SM()){
		return false;
	}
	HAL_Delay(mlx90393_tconv[this->reg.filter][this->reg.osr] + 10);
	if(!i2c_RM()){
		return false;
	}
	return true;
}

bool MLX90393::calibrate_4element(uint32_t duration_ms){
	uint32_t current_time = HAL_GetTick();
	const uint8_t matrix_size = 4;

	float32_t matrix_x[matrix_size][matrix_size] = {0};
	float32_t matrix_y[matrix_size] = {0};                   // FIX: size 4 (was indexed [4])
	float32_t matrix_result[matrix_size] = {0};
	float32_t matrix_x_inv[matrix_size][matrix_size] = {0};  // FIX: inverse needs a separate dst

	arm_matrix_instance_f32 arm_matrix_x, arm_matrix_x_inv, arm_matrix_y, arm_matrix_result;

	float min_v[3] = { 1e9f,  1e9f,  1e9f};
	float max_v[3] = {-1e9f, -1e9f, -1e9f};
	uint32_t samples = 0;

	while(HAL_GetTick() < current_time + duration_ms) {      
		this->i2c_SM();
		HAL_Delay(3);
		this->i2c_RM();
		this->decode();
		this->convert();

		float x = this->converted.x, y = this->converted.y, z = this->converted.z;
		float s = x*x + y*y + z*z;

		matrix_x[0][0] += x*x; matrix_x[0][1] += x*y; matrix_x[0][2] += x*z; matrix_x[0][3] += x;
		matrix_x[1][0] += x*y; matrix_x[1][1] += y*y; matrix_x[1][2] += y*z; matrix_x[1][3] += y;
		matrix_x[2][0] += x*z; matrix_x[2][1] += y*z; matrix_x[2][2] += z*z; matrix_x[2][3] += z;
		matrix_x[3][0] += x;   matrix_x[3][1] += y;   matrix_x[3][2] += z;   matrix_x[3][3] += 1;

		matrix_y[0] += x*s;
		matrix_y[1] += y*s;
		matrix_y[2] += z*s;
		matrix_y[3] += s;                            

		if(x < min_v[0]) min_v[0] = x; if(x > max_v[0]) max_v[0] = x;
		if(y < min_v[1]) min_v[1] = y; if(y > max_v[1]) max_v[1] = y;
		if(z < min_v[2]) min_v[2] = z; if(z > max_v[2]) max_v[2] = z;
		samples++;
	}

	this->cal_diag.samples = samples;

	arm_mat_init_f32(&arm_matrix_x,      matrix_size, matrix_size, (float32_t *)matrix_x);
	arm_mat_init_f32(&arm_matrix_x_inv,  matrix_size, matrix_size, (float32_t *)matrix_x_inv);
	arm_mat_init_f32(&arm_matrix_y,      matrix_size, 1,           (float32_t *)matrix_y);       // FIX: 4x1
	arm_mat_init_f32(&arm_matrix_result, matrix_size, 1,           (float32_t *)matrix_result);  // FIX: 4x1

	if(samples < 100){ this->cal_diag.status = 1; return false; }

	if(arm_mat_inverse_f32(&arm_matrix_x, &arm_matrix_x_inv) != ARM_MATH_SUCCESS){
		this->cal_diag.status = 3; return false;             // singular
	}
	if(arm_mat_mult_f32(&arm_matrix_x_inv, &arm_matrix_y, &arm_matrix_result) != ARM_MATH_SUCCESS){
		this->cal_diag.status = 3; return false;
	}

	float cx = 0.5f * matrix_result[0];
	float cy = 0.5f * matrix_result[1];
	float cz = 0.5f * matrix_result[2];
	this->correction_factors.hard_iron[0] = cx;
	this->correction_factors.hard_iron[1] = cy;
	this->correction_factors.hard_iron[2] = cz;

	float field = sqrtf(matrix_result[3] + cx*cx + cy*cy + cz*cz);
	this->correction_factors.field_strength = field;

	this->cal_diag.radius[0] = 0.5f * (max_v[0] - min_v[0]);
	this->cal_diag.radius[1] = 0.5f * (max_v[1] - min_v[1]);
	this->cal_diag.radius[2] = 0.5f * (max_v[2] - min_v[2]);

	if(this->cal_diag.radius[0] < 10.0f || this->cal_diag.radius[1] < 10.0f || this->cal_diag.radius[2] < 10.0f){
		this->cal_diag.status = 2; return false;             // poor rotation coverage
	}
	if(!(field > 20.0f && field < 90.0f)){
		this->cal_diag.status = 4; return false;             // non-physical field
	}

	this->correction_factors.soft_iron[0][0] = field / this->cal_diag.radius[0];
	this->correction_factors.soft_iron[1][1] = field / this->cal_diag.radius[1];
	this->correction_factors.soft_iron[2][2] = field / this->cal_diag.radius[2];

	this->cal_diag.status = 0;
	return true;
}

int MLX90393::zyxt_set_bits(){
	int count = 0;
	uint8_t temp = this->zyxt;
	while(temp){
		if(temp & 0x1){
			count++;
		}
		temp >>= 1;
	}
	return count;
}

void MLX90393::decode(){
	uint8_t *cursor = this->rx_data;
	this->reg.stat = this->rx_data[0];
	cursor += 1; //Skip status byte
	if(this->zyxt & MLX90393_T){
		this->raw.t = decode_helper(cursor);
		cursor += 2;
	}
	if(this->zyxt & MLX90393_X){
		this->raw.x = decode_helper(cursor);
		cursor += 2;
	}
	if(this->zyxt & MLX90393_Y){
		this->raw.y = decode_helper(cursor);
		cursor += 2;
	}
	if(this->zyxt & MLX90393_Z){
		this->raw.z = decode_helper(cursor);
		cursor += 2;
	}
}

int16_t MLX90393::decode_helper(uint8_t *data){
	return (data[0] << 8 | data[1]);
}

void MLX90393::convert(){
	// RES 2/3 are unsigned -> reinterpret the raw word as unsigned and subtract 
	// 							the zero-field offset to recover a signed value
	// RES 0/1 are already signed
    int32_t x = (this->reg.x_res == MLX90393_RES_17) ? (int32_t)(uint16_t)this->raw.x - MLX90393_RES17_ZERO_OFFSET :
                (this->reg.x_res == MLX90393_RES_18) ? (int32_t)(uint16_t)this->raw.x - MLX90393_RES18_ZERO_OFFSET :
                                                       (int32_t)this->raw.x;
    int32_t y = (this->reg.y_res == MLX90393_RES_17) ? (int32_t)(uint16_t)this->raw.y - MLX90393_RES17_ZERO_OFFSET :
                (this->reg.y_res == MLX90393_RES_18) ? (int32_t)(uint16_t)this->raw.y - MLX90393_RES18_ZERO_OFFSET :
                                                       (int32_t)this->raw.y;
    int32_t z = (this->reg.z_res == MLX90393_RES_17) ? (int32_t)(uint16_t)this->raw.z - MLX90393_RES17_ZERO_OFFSET :
                (this->reg.z_res == MLX90393_RES_18) ? (int32_t)(uint16_t)this->raw.z - MLX90393_RES18_ZERO_OFFSET :
                                                       (int32_t)this->raw.z;

    // Z uses column [0] to match hallconf 0x0C (was [1])
    this->converted.x = (float)x * sens_lookup_0xC[this->reg.gain][this->reg.x_res][0];
    this->converted.y = (float)y * sens_lookup_0xC[this->reg.gain][this->reg.y_res][0];
    this->converted.z = (float)z * sens_lookup_0xC[this->reg.gain][this->reg.z_res][0];

}

void MLX90393::set_zyxt(uint8_t set_zyxt){
	this->zyxt = set_zyxt;
}

bool MLX90393::get_rm_flag(){
	return this->rm_flag;
}

void MLX90393::set_update_flag(bool update){
	this->mes_updated = update;
}

bool MLX90393::read_update_flag(){
	return this->mes_updated;
}

void MLX90393::set_wait_flag(bool update){
	this->wait_flag = update;
}

bool MLX90393::read_wait_flag(){
	return this->wait_flag;
}

float MLX90393::get_x_data(){
	return this->converted.x;
}

float MLX90393::get_y_data(){
	return this->converted.y;
}

float MLX90393::get_z_data(){
	return this->converted.z;
}

