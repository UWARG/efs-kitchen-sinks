#ifndef INC_ICP20100_I2C_HPP_
#define INC_ICP20100_I2C_HPP_

#include <stdio.h>
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_i2c.h"



#define ICP20100_I2C_ADDR (0x64 << 1) // i2c is 7 bit address, left shit by 1 to make 1 byte.

#define ICP20100_REG_MODE_SELECT 0xC0
#define ICP20100_DEVICE_ID 0x0C
#define ICP20100_MASTER_LOCK 0xBE
#define ICP20100_OTP_CONFIG_1 0xAC
#define ICP20100_OTP_STATUS2 0xBF

#define ICP20100_REG_MODE_SELECT_KEY 0x04
#define ICP20100_MASTER_UNLOCK_KEY 0x1F
#define ICP20100_OTP_ENABLE_BOTH 0x03
#define ICP20100_OTP_STATUS2_BOOTUP 0x01

#define ICP20100_POWER_MODE (1 << 2)
#define ICP20100_FORCED_MES_TRIGGER (1 << 4)

#define ICP20100_TRIGGER_COMMAND_MEAS (ICP20100_POWER_MODE | ICP20100_FORCED_MES_TRIGGER)

#define ICP20100_PRESS_DATA_0 0xFA

#define SCALE 131072.0f
#define Alt_min 70.0f
#define Alt_max 40.0f

class ICP20100{
	public:
		ICP20100(I2C_HandleTypeDef *hi2c);
		void toggleBlueLed();
		void transmit();

	private:
		I2C_HandleTypeDef *hi2c;

};
#endif /* INC_ICP20100_I2C_HPP_ */
