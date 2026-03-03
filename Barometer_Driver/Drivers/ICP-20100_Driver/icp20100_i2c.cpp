#include "icp20100_i2c.hpp"
#include "main.h"

#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_i2c.h"

ICP20100::ICP20100(I2C_HandleTypeDef *hi2c){
	//empty constructor
	this->hi2c = hi2c;
}

void ICP20100::transmit()
{
	uint8_t dummy = 0x00;
	if(HAL_I2C_Master_Transmit(hi2c, ICP20100_I2C_ADDR, &dummy, I2C_MEMADD_SIZE_8BIT, 10) == HAL_OK){
		BSP_LED_Toggle(LED_RED);
		BSP_LED_Toggle(LED_GREEN);
		BSP_LED_Toggle(LED_BLUE);
	}
}

