#pragma once

#ifdef __cplusplus
#include <cstdint>
#endif

typedef struct {
	uint8_t nodeId;
	int health;
	uint32_t lastHeartbeatTime;
	uint32_t uptime;
	uint32_t softwareVersion;
	uint32_t hardwareVersion;
} Node_t;
