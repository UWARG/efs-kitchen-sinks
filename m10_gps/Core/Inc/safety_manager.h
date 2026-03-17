#pragma once

#include <stdint.h>
#define GPS_TIMEOUT_MS 1000

typedef struct
{
    uint8_t switch_pressed;
    uint8_t gps_fix_valid;
    uint16_t gps_fix_quality;
    uint32_t last_gps_update_ms;
    uint32_t now_ms;
} SafetyManagerInput_t;

typedef struct
{
    uint8_t safety_enabled;
}SafetyManagerOutput_t;

void SafetyManager_Update(const SafetyManagerInput_t *input, SafetyManagerOutput_t *output);