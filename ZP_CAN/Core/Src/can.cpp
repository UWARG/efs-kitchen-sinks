
#include "can.hpp"

static void StaticOnTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    CAN* self = static_cast<CAN*>(ins->user_reference);
    self->CanardOnTransferReception(ins, transfer);
}

static bool StaticShouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_sig, uint16_t id, CanardTransferType type, uint8_t src) {

	return static_cast<CAN*>(ins->user_reference)->CanardShouldAcceptTransfer(ins, out_sig, id, type, src);
}

CAN::CAN(FDCAN_HandleTypeDef *hfdcan) : hfdcan(hfdcan) {
	static uint8_t canardMemoryPool[1024];

	canardInit(&canard,
			canardMemoryPool,
			sizeof(canardMemoryPool),
			&StaticOnTransferReception,
			&StaticShouldAcceptTransfer,
			this
	);

	nodeStatus = {0};

	canard.node_id = NODE_ID;
}

CAN::~CAN() {}



bool CAN::CanardShouldAcceptTransfer(
	const CanardInstance *ins,
	uint64_t *out_data_type_signature,
	uint16_t data_type_id,
	CanardTransferType transfer_type,
	uint8_t source_node_id)
{
	if (transfer_type == CanardTransferTypeRequest) {
		// check if we want to handle a specific service request
		switch (data_type_id) {
			case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
			/* case UAVCAN_PROTOCOL_GETNODEINFO_ID: */ {
				if (transfer_type == CanardTransferTypeResponse || transfer_type == CanardTransferTypeRequest) {
					*out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
					return true;
				}
				else if (transfer_type == CanardTransferTypeBroadcast) {
					*out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
					return true;
				}
				else {
					return false;
				}
			}
			case UAVCAN_PROTOCOL_NODESTATUS_ID: {
				return true;
			}
		}
	}
	return false;
}

void CAN::CanardOnTransferReception(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id) {
			case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
			/* case UAVCAN_PROTOCOL_GETNODEINFO_ID: */ {
				if (transfer->transfer_type == CanardTransferTypeResponse) {
					// handleGetNodeInfo here
				}
				else if (transfer->transfer_type == CanardTransferTypeBroadcast) {
					handleNodeAllocation(transfer);
				}
				break;
			}
			case UAVCAN_PROTOCOL_NODESTATUS_ID: {
				handleNodeStatus(transfer);
				break;
			}
        }
    }
}


void CAN::handleNodeStatus(CanardRxTransfer *transfer) {
	uint32_t tick = HAL_GetTick();

	canNode node {0};

	node.lastSeenTick = tick;

	bool success = uavcan_protocol_NodeStatus_decode(transfer, &node.status);

	if (!success) return;

	// Node ID out of bounds or is anonymous
	if (transfer->source_node_id > CANARD_MAX_NODE_ID) return;
	if (transfer->source_node_id == 0) return;

	canNodes[transfer->source_node_id] = node;
}

void CAN::handleNodeAllocation(CanardRxTransfer *transfer){

	uint8_t first_half_unique_id[8];

 	if (transfer->source_node_id != 0) // the source node is not 0, it is not anonymous
	{
		return;
	}

    struct uavcan_protocol_dynamic_node_id_Allocation msg = {};
	uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

	if (msg.node_id != 0) // the node id is not 0, it is not anonymous
	{
		return;
	}

	memcpy(first_half_unique_id, msg.unique_id.data, 8);

	// Generate the node id and allocate it
	msg.node_id = allocateNode();

	if (msg.node_id == -1){
		return;
	}

	msg = {};

	// Send message back
	uint8_t decode_buffer[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
	uavcan_protocol_dynamic_node_id_Allocation_encode(&msg, decode_buffer);



	broadcast(
			CanardTransferTypeResponse,
		UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
		UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
		0,
		CANARD_TRANSFER_PRIORITY_LOW,
		decode_buffer,
		sizeof(decode_buffer)
	);

}

int8_t CAN::allocateNode() {
	// check if the node id is already allocated
	int currId = nextAvailableID;

	if (currId > CANARD_MAX_NODE_ID) {
		return -1; // no more node ids available
	}

	nextAvailableID++;

	return currId;
}

/*
Function to convert all canard CAN frames and send them through HAL

Consider removing for loop
*/
void CAN::sendCANTx() {
	bool success = true;
	for (const CanardCANFrame *frame; frame != nullptr; frame = canardPeekTxQueue(&canard)) {
		if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
			FDCAN_TxHeaderTypeDef txHeader;

			txHeader.Identifier = frame->id;
			txHeader.IdType = FDCAN_EXTENDED_ID;
			txHeader.TxFrameType = FDCAN_DATA_FRAME;
			txHeader.DataLength = FDCAN_DLC_BYTES_8;
			txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			txHeader.BitRateSwitch = FDCAN_BRS_OFF;
			txHeader.FDFormat = FDCAN_CLASSIC_CAN;

			const uint8_t *txData = frame->data;

			bool success = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) == HAL_OK;

			if (success) {
				canardPopTxQueue(&canard);
			}
		}
	}
}

bool CAN::routineTasks() {
	sendCANTx();

	uint32_t tick = HAL_GetTick();

	if (tick > last1HzTick + UAVCAN_PROTOCOL_NODESTATUS_MAX_BROADCASTING_PERIOD_MS/2) {
		last1HzTick = tick;
		process1HzTasks();
	}
}

void CAN::sendNodeStatus() {
	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    nodeStatus.uptime_sec = HAL_GetTick()/1000LL;
    nodeStatus.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    nodeStatus.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    nodeStatus.sub_mode = 0;
    // put whatever you like in here for display in GUI
    nodeStatus.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&nodeStatus, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;


    broadcast(CanardTransferTypeBroadcast,
			UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
			UAVCAN_PROTOCOL_NODESTATUS_ID,
			&transfer_id,
			CANARD_TRANSFER_PRIORITY_LOW,
			buffer,
			len
	);
}

void CAN::process1HzTasks() {

	uint32_t timestamp_msec = HAL_GetTick();

	// Check if nodes invalid
	for (int i = CANARD_MIN_NODE_ID; i <= CANARD_MAX_NODE_ID; i++) {
		// Make copy of status in case it changes
		canNode node = canNodes[i];

		if (timestamp_msec-node.lastSeenTick > UAVCAN_PROTOCOL_NODESTATUS_OFFLINE_TIMEOUT_MS) {
			canNodes[i].status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE;
		}
	}

	// Transmit NodeStatus
	sendNodeStatus();
}


/*
Wrapper function with mutex
*/
int16_t CAN::broadcastObj(CanardTxTransfer* transfer) {
	osStatus_t status = osMutexAcquire(canBroadcastMutex, CAN_BROADCAST_MUTEX_TIMEOUT);

	if (status != osOK){
		return -1; // handle failure
	}

	int16_t res = canardBroadcastObj(&canard, transfer);
	osMutexRelease(canBroadcastMutex);

	return res;
}

int16_t CAN::broadcast(
	CanardTransferType transfer_type,
	uint64_t data_type_signature,
	uint16_t data_type_id,
	uint8_t* inout_transfer_id,
	uint8_t priority,
	const uint8_t* payload,
	uint16_t payload_len
	#if CANARD_ENABLE_CANFD
		, bool canfd; ///< True if CAN FD is enabled
	#endif
	#if CANARD_ENABLE_DEADLINE
		, uint64_t deadline_usec; ///< Deadline in microseconds
	#endif
	#if CANARD_MULTI_IFACE
		, uint8_t iface_mask; ///< Bitmask of interfaces to send the transfer on
	#endif
	#if CANARD_ENABLE_TAO_OPTION
		, bool tao; ///< True if tail array optimization is enabled
	#endif
)
{
	CanardTxTransfer transfer_object = {
		transfer_type = transfer_type,
		data_type_signature = data_type_signature,
		data_type_id = data_type_id,
		inout_transfer_id = inout_transfer_id,
		priority = priority,
		payload = payload,
		payload_len = payload_len,
		#if CANARD_ENABLE_CANFD
			canfd = canfd, ///< True if CAN FD is enabled
		#endif
		#if CANARD_ENABLE_DEADLINE
			deadline_usec = deadline_usec, ///< Deadline in microseconds
		#endif
		#if CANARD_MULTI_IFACE
			iface_mask = iface_mask, ///< Bitmask of interfaces to send the transfer on
		#endif
		#if CANARD_ENABLE_TAO_OPTION
			tao = tao, ///< True if tail array optimization is enabled
		#endif
	};

	return broadcastObj(&transfer_object);
}
