#include <cstdint>
#include <uavcan.protocol.NodeStatus.h>

struct canNode {
    uint64_t lastSeenTick;
    uavcan_protocol_NodeStatus status;
};
