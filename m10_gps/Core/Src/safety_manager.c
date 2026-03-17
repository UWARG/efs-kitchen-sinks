#include "safety_manager.h"


void SafetyManager_Update(const SafetyManagerInput_t *input, SafetyManagerOutput_t *output)
{
    uint8_t gps_fresh = 0;

    if ((input->now_ms - input->last_gps_update_ms) <= GPS_TIMEOUT_MS)
    {
        gps_fresh = 1;
    }

    if (input->switch_pressed && input->gps_fix_valid && (input->gps_fix_quality > 0) && gps_fresh)
    {
        output->safety_enabled = 1;
    }
    else
    {
        output->safety_enabled = 0;
    }
}
