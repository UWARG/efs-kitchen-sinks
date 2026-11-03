#include "is31fl3195_i2c.hpp"

const uint8_t IS31FL3195_MAGIC_VALUE = 0xC5;

IS31FL3195_Channel IS31FL3195_ALL_CHANNELS[4] = { { // CH1
		.REG_CURRENT_LEVEL = 0x10, }, { // CH2
		.REG_CURRENT_LEVEL = 0x21, }, { // CH3
		.REG_CURRENT_LEVEL = 0x32, }, { // CH4
		.REG_CURRENT_LEVEL = 0x40, }, };

IS31FL3195::IS31FL3195(I2C_HandleTypeDef *hi2c, uint8_t address) :
		hi2c(hi2c), address(address) {
}

HAL_StatusTypeDef IS31FL3195::registerWrite(IS31FL3195_Register reg,
		uint8_t val) {
	// as per datasheet
	uint8_t pData[2] = { static_cast<uint8_t>(reg), val };
	return HAL_I2C_Master_Transmit(hi2c, address, pData, sizeof(pData),
	HAL_MAX_DELAY);
}

HAL_StatusTypeDef IS31FL3195::reset() {
	return this->registerWrite(IS31FL3195_REG_RESET, IS31FL3195_MAGIC_VALUE);
}

HAL_StatusTypeDef IS31FL3195::configure(IS31FL3195_ConfReg conf) {
	return this->registerWrite(IS31FL3195_REG_RESET, *(uint8_t*) &conf);
}

HAL_StatusTypeDef IS31FL3195::writeCurrentLevel(IS31FL3195_Channel &channel,
		IS31FL3195_CurrentLevel level) {
	return this->registerWrite(channel.REG_CURRENT_LEVEL, level);
}

HAL_StatusTypeDef IS31FL3195::setSolidColor(IS31FL3195_CurrentLevel rgbw[4]) {
	HAL_StatusTypeDef status;
	IS31FL3195_ConfReg
	confreg { .mode_1 = IS31FL3195_MODE_CURRENT_LEVEL };

	status = this->configure(confreg);
	if (status != HAL_OK)
		return status;

	for (size_t i = 0; i < 4; i++) {
		status = this->writeCurrentLevel(IS31FL3195_ALL_CHANNELS[i], rgbw[i]);
		if (status != HAL_OK)
			return status;
	}

	return HAL_OK;
}

HAL_StatusTypeDef IS31FL3195::setPattern(void *pArr, size_t pLen) {
	// TODO: implement setPattern

	return HAL_ERROR;
}

IS31FL3195::~IS31FL3195() {
	// TODO Auto-generated destructor stub
}

