#include "board_safety_switch.hpp"

bool BoardSafetySwitch::isPressed() const
{
    return HAL_GPIO_ReadPin(GPS_Safety_SW_GPIO_Port, GPS_Safety_SW_Pin) == GPIO_PIN_SET;
}
