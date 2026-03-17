#pragma once

#include <cstdint>
#include "safety_switch_iface.hpp"

// data gathered outside manager and is used to evaluate the safety state.
struct SafetyManagerInput
{
    bool gpsFixValid;
    uint16_t gpsFixQuality;
    uint32_t lastGpsUpdateMs;
    uint32_t nowMs;
};

struct SafetyManagerOutput
{
    bool safetyEnabled;
};

// own safety policy while relying on injected interfaces for hardware state.
class SafetyManager
{
public:
    explicit SafetyManager(const ISafetySwitch& safetySwitch);

    static constexpr uint32_t kGpsTimeoutMs = 1000U;
    SafetyManagerOutput update(const SafetyManagerInput& input) const;

private:
    bool isGpsFresh(const SafetyManagerInput& input) const;

    const ISafetySwitch& safetySwitch;
};
