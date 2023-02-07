#include "main.h"

#ifndef PPM_SIG
#define correction 1.0f /*this correction is used in psc and arr calculation*/
#endif /*PPM_SIG*/

uint32_t microsecs_to_counter(uint32_t time_length, uint16_t psc) {
	float tick_length, tick_count;
	tick_length = BASE_FREQUENCY / (psc + correction);
	tick_length = tick_length / SEC_TO_MICROSEC;

	tick_count = time_length * tick_length;

	return (uint32_t) tick_count;
}


uint32_t percentage_arr(float percentage) {
	float time_added;
	time_added = percentage * DOWN_INTERVAL;

	return MIN_PULSE_WIDTH + (uint16_t)time_added;
}

uint32_t reset_sync(void) {
	return MIN_PULSE_WIDTH + MIN_RESET_PULSE;
}
