#include <cstdint>
#include "dsdlc_generated/include/dronecan_msgs.h"

struct canNode {
    uint64_t lastSeenTick;
    uavcan_protocol_NodeStatus status;
};
