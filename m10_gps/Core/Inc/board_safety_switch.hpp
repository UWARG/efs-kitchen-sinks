#pragma once

#include "main.h"
#include "safety_switch_iface.hpp"

// implementation that reads the stm32 gpio wired to the safety switch.
class BoardSafetySwitch : public ISafetySwitch
{
    public:
    bool isPressed() const override; 
};
