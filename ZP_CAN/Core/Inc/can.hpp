
#pragma once

#include <can_iface.hpp>
#include <canard.h>
#include "stm32l5xx_hal.h"
#include <dronecan_msgs.h>
#include "can_defines.hpp"
#include "cmsis_os2.h"
#include "museq.hpp"
#include <cstdint>
#include "can_datatypes.hpp"


class CAN : ICAN {

private:
	canNode canNodes[CANARD_MAX_NODE_ID + 1];
	uint8_t nextAvailableID = CANARD_MIN_NODE_ID + 1;
	FDCAN_HandleTypeDef *hfdcan;

	CanardInstance canard;

	void sendNodeStatus();

	void sendCANTx();

	void handleNodeAllocation(CanardRxTransfer* transfer);
	void handleNodeStatus(CanardRxTransfer* transfer);

	int8_t allocateNode();

	uavcan_protocol_NodeStatus nodeStatus;

	uint32_t last1HzTick = 0;


public:
	CAN(FDCAN_HandleTypeDef *hfdcan);

	bool CanardShouldAcceptTransfer(const CanardInstance* ins,          ///< Library instance
		uint64_t* out_data_type_signature,  ///< Must be set by the application!
		uint16_t data_type_id,              ///< Refer to the specification
		CanardTransferType transfer_type,   ///< Refer to CanardTransferType
		uint8_t source_node_id);

	void CanardOnTransferReception(CanardInstance* ins,                 ///< Library instance
		CanardRxTransfer* transfer);

	virtual ~CAN();

	// Called as much as possible
	bool routineTasks();

	// Called once every second
	void process1HzTasks();

	int16_t canardSTM32Receive(FDCAN_HandleTypeDef *hfdcan, uint32_t RxLocation, CanardCANFrame *const rx_frame);


	int16_t broadcastObj(
		CanardTxTransfer* transfer
	);

	int16_t broadcast(
		uint64_t data_type_signature,
		uint16_t data_type_id,
		uint8_t* inout_transfer_id,
		uint8_t priority,
		const void* payload,
		uint16_t payload_len
	);

};
