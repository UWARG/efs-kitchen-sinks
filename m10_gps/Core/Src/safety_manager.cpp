#include "safety_manager.hpp"

SafetyManager::SafetyManager(const ISafetySwitch& safetySwitch)
    : safetySwitch(safetySwitch)
{
}

bool SafetyManager::isGpsFresh(const SafetyManagerInput& input) const
{
    return (input.nowMs - input.lastGpsUpdateMs) <= kGpsTimeoutMs;
}

SafetyManagerOutput SafetyManager::update(const SafetyManagerInput& input) const
{
    // Safety is only enabled when switch is pressed and latest GPS data is valid.
    const bool safetyEnabled =
        safetySwitch.isPressed() &&
        input.gpsFixValid &&
        (input.gpsFixQuality > 0U) &&
        isGpsFresh(input);

    // alarm when system is unsafe.
    const bool buzzerOn = !safetyEnabled;

    return {safetyEnabled, buzzerOn};
}
