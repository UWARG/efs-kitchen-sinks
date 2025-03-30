
#pragma once

#include <can_iface.hpp>
#include "can_datatypes.hpp"
#include <canard.h>
#include "stm32l5xx_hal.h"
#include <can_defines.hpp>
#include "../../dsdlc_generated/include/dronecan_msgs.h"

//#include <map>

class CAN : ICAN {

private:
//	std::map<uint8_t, Node_t> canNodes;
	uint8_t nextAvailableID = 2;


	FDCAN_HandleTypeDef *hfdcan;

	void handle_NodeAllocation(CanardRxTransfer *transfer);

	// Returns the id of the allocated node
	void handle_ReceiveNodeInfo(CanardRxTransfer *transfer);
	bool removeNode();

	CanardInstance canInst;

	static bool CanardShouldAcceptTransfer(const CanardInstance* ins,          ///< Library instance
	                                            uint64_t* out_data_type_signature,  ///< Must be set by the application!
	                                            uint16_t data_type_id,              ///< Refer to the specification
	                                            CanardTransferType transfer_type,   ///< Refer to CanardTransferType
	                                            uint8_t source_node_id);

	static void CanardOnTransferReception(CanardInstance* ins,                 ///< Library instance
	                                           CanardRxTransfer* transfer);

	void sendCANTx();

public:
	CAN(FDCAN_HandleTypeDef* hfdcan);
	CAN();

	virtual ~CAN();

	int16_t canardSTM32Receive(FDCAN_HandleTypeDef *hfdcan, uint32_t RxLocation, CanardCANFrame *const rx_frame);

	bool routineTasks();


	/**
	 * Wrapper function with mutex
	 *
	* Sends a broadcast transfer.
	* If the node is in passive mode, only single frame transfers will be allowed (they will be transmitted as anonymous).
	*
	* For anonymous transfers, maximum data type ID (CanardTxTransfer::data_type_id) is limited to 3 (see specification for details).
	*
	* Please refer to the specification for more details about data type signatures (CanardTxTransfer::data_type_signature). Signature for
	* any data type can be obtained in many ways; for example, using the generated code generated using dronecan_dsdlc (see the repository).
	*
	* Use CanardTxTransfer structure to pass the transfer parameters. The structure is initialized by the
	* canardInitTxTransfer() function.
	*
	* Pointer to the Transfer ID (CanardTxTransfer::inout_transfer_id) should point to a persistent variable
	* (e.g. static or heap allocated, not on the stack); it will be updated by the library after every transmission.
	* The Transfer ID value cannot be shared between transfers that have different descriptors!
	* More on this in the transport layer specification.
	*
	* Returns the number of frames enqueued, or negative error code.
	*/
	int16_t canardBroadcastObj1(
		CanardTxTransfer* transfer      ///< Transfer object
	);

	int16_t canardBroadcast(            ///< Library instance
		uint64_t data_type_signature,   ///< See above
		uint16_t data_type_id,          ///< Refer to the specification
		uint8_t* inout_transfer_id,     ///< Pointer to a persistent variable containing the transfer ID
		uint8_t priority,               ///< Refer to definitions CANARD_TRANSFER_PRIORITY_*
		const void* payload,            ///< Transfer payload
		uint16_t payload_len
	);

};
