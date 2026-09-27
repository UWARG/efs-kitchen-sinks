/*
 * Comms.hpp
 *
 *  Created on: Aug 20, 2022
 *      Author: blueb
 */

#ifndef INC_COMMS_HPP_
#define INC_COMMS_HPP_

#include <stdint.h>
//#include "cmsis_armcc.h"

#define GROUND_ADDRESS 0x0013A20041B16D1C

/*
 *  Contains competition/drone agnostic data that will be needed regardless
 *  of drone type and competition requirements. GPS and IMU data as well as the
 *  current motor outputs.
 *
 *  25 bytes
 */

/*
class __attribute__ ((packed)) DroneBasicPayload {
	private: // BIG ENDIAN
		float latitude;
		float longitude;
		float altitude; //12

		float yaw;
		float pitch;
		float roll; // 12

		uint8_t motorOutputs[12];

	public: // LITTLE ENDIAN
		float getLatitude();
		void setLatitude(float lat);
		float getLongitude();
		void setLongitude(float longitude);
		float getAltitude();
		void setAltitude(float altitude);

		float getYaw();
		void setYaw(float yaw);
		float getPitch();
		void setPitch(float pitch);
		float getRoll();
		void setRoll(float roll);

		uint8_t getMotorOutput(int index);
		void setMotorOutput(int index, uint8_t val);

		uint8_t* getArr();
};

*/

struct __attribute__ ((packed)) DroneBasicPayload {

		float latitude;
		float longitude;
		float altitude; //12

		float yaw;
		float pitch;
		float roll; // 12

		uint8_t motorOutputs[12];
};

/*
 * Holds all the different types of payloads and includes generic things like our CRC (eventually) and
 * the time the packet was sent.
 *
 * 30 bytes
*/
struct __attribute__ ((packed)) Packet {
	uint32_t time;
	uint8_t payloadType;
	//uint8_t payloadLength;

	// add a CRC eventually
};
/*
class __attribute__ ((packed)) Packet {
	private: // BIG ENDIAN
		uint32_t time;
		uint8_t payloadType;
		uint8_t payload[sizeof(DroneBasicPayload)];
		// add a CRC eventually
	public: // LITTLE ENDIAN
		uint8_t getPayloadType() const;
		void setPayloadType(uint8_t payloadType);
		uint32_t getTime() const;
		void setTime(uint32_t time);
		void setPayload(uint8_t* payload, int size);

		uint8_t* getArr();
};

*/

/*
 * Zigbee 3.0 transmit frame. Used to send data from one xbee to another.
 *
 *
 */

struct __attribute__ ((packed)) TransmitFrameHeader {
	uint8_t startDelim = 0x7E;
	uint16_t messageLength; // length of a transmit frame WITHOUT the message and WITHOUT the length and checksum
	uint8_t frameType = 0x10;
	uint8_t frameID = 0x01;
	uint64_t destAddress = GROUND_ADDRESS;
	uint16_t shortAddress = 0xFFFE; // FFFE in little endian
	uint8_t broadcastRadius = 0x00;
	uint8_t options = 0x00;

};

void TransmitMessage(DroneBasicPayload* payload);


#endif /* INC_COMMS_HPP_ */
