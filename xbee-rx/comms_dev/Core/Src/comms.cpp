/*
 * comms.cpp
 *
 *  Created on: Aug 20, 2022
 *      Author: blueb
 */

#include "comms.hpp"
#include "main.h"
#include "usart.h"
//#include "cmsis_armcc.h"

#define UART_TIMEOUT 100

void rev(uint8_t* val, int size) {
	uint8_t temp;
	for (int x = 0; x < size/2; x++) {
		temp = val[x];
		val[x] = val[size - 1 - x];
		val[size - 1 - x] = temp;
	}
}
/*
float DroneBasicPayload::getLatitude() {
	float tmp = this->latitude;
	rev((uint8_t*)&tmp, sizeof(float));
	return tmp;
}
void DroneBasicPayload::setLatitude(float lat) {
	rev((uint8_t*)&lat, sizeof(float));
	this->latitude = lat;
}

float DroneBasicPayload::getLongitude() {
	float tmp = this->longitude;
	rev((uint8_t*)&tmp, sizeof(float));
	return tmp;
}
void DroneBasicPayload::setLongitude(float lon) {
	rev((uint8_t*)&lon, sizeof(float));
	this->longitude = lon;
}

float DroneBasicPayload::getAltitude() {
	float tmp = this->altitude;
	rev((uint8_t*)&tmp, sizeof(float));
	return tmp;
}
void DroneBasicPayload::setAltitude(float alt) {
	rev((uint8_t*)&alt, sizeof(float));
	this->altitude = alt;
}

float DroneBasicPayload::getYaw() {
	float tmp = this->yaw;
	rev((uint8_t*)&tmp, sizeof(float));
	return tmp;
}
void DroneBasicPayload::setYaw(float yaw) {
	rev((uint8_t*)&yaw, sizeof(float));
	this->yaw = yaw;
}

float DroneBasicPayload::getPitch() {
	float tmp = this->pitch;
	rev((uint8_t*)&tmp, sizeof(float));
	return tmp;
}
void DroneBasicPayload::setPitch(float pitch) {
	rev((uint8_t*)&pitch, sizeof(float));
	this->pitch = pitch;
}

float DroneBasicPayload::getRoll() {
	float tmp = this->roll;
	rev((uint8_t*)&tmp, sizeof(float));
	return tmp;
}
void DroneBasicPayload::setRoll(float roll) {
	rev((uint8_t*)&roll, sizeof(float));
	this->roll = roll;
}

uint8_t DroneBasicPayload::getMotorOutput(int index) {
	return this->motorOutputs[index];
}
void DroneBasicPayload::setMotorOutput(int index, uint8_t val) {
	this->motorOutputs[index] = val;
}

uint8_t* DroneBasicPayload::getArr() {
	return (uint8_t*)this;
}

uint8_t Packet::getPayloadType() const {
	return payloadType;
}

void Packet::setPayloadType(uint8_t payloadType) {
	this->payloadType = payloadType;
}

uint32_t Packet::getTime() const {
	float tmp = this->time;
	rev((uint8_t*)&tmp, sizeof(uint32_t));
	return tmp;
}
void Packet::setTime(uint32_t time) {
	rev((uint8_t*)&time, sizeof(uint32_t));
	this->time = time;
}

void Packet::setPayload(uint8_t* payload, int size) {
	for(int x = 0; x < size && x < sizeof(this->payload);x++) {
			this->payload[x] = payload[x];
	}

}

*/
bool calculateChecksum(uint8_t* msg, uint8_t size) {
	if(size < 4) {
		return false;
	}

	uint8_t checksum = 0;
	for (int x = 3; x < size-1;x++) {
		checksum += msg[x];
	}
	msg[size-1] = checksum;
	return true;
}


/*
void TransmitMessage(DroneBasicPayload payload) {

	Packet packet = {};
	packet.payloadType = 1;

	packet.time = HAL_GetTick();
	uint8_t* payloadArr = (uint8_t*)&payload;

	for(int x = 0; x < sizeof(DroneBasicPayload);x++) {
		packet.payload[x] = payloadArr[x];
	}

	TransmitFrame tf = {};
	tf.messageLength += sizeof(Packet);


	uint8_t* packetArr = (uint8_t*)&packet;

	for(int x = 0; x < sizeof(Packet);x++) {
		tf.message[x] = packetArr[x];
	}

	tf.messageLength += sizeof(Packet);

	uint8_t rawMessageDbg[50];
	uint8_t* rawMessage = (uint8_t*)&tf;
	uint16_t sizeMessage = sizeof(TransmitFrame);
	bool res = calculateChecksum(rawMessage, sizeMessage);
	if(!res) {
		// message is too small
		return;
	}

	for (int x = sizeMessage - 1; x > 0; x++) {
		rawMessageDbg[sizeMessage - 1 - x] = rawMessage[x];
	}

	rawMessageDbg;
	//actually send the message
	HAL_UART_Transmit(&huart2,rawMessage, sizeMessage,UART_TIMEOUT);
}


*/

void TransmitMessage(DroneBasicPayload* payload) {
	Packet packet = {};

	packet.time = HAL_GetTick();
	packet.payloadType = 1;

	uint8_t checksum = 0xFF;

	uint8_t rawPacket[sizeof(TransmitFrameHeader) + sizeof(Packet) + sizeof(DroneBasicPayload) + sizeof(checksum)];
	TransmitFrameHeader tf = {};

	uint8_t* tfArr = (uint8_t*)&tf;
	uint8_t* packetArr = (uint8_t*)&packet;
	uint8_t* payloadArr = (uint8_t*)payload;

	// TODO set the length


	tf.messageLength = sizeof(TransmitFrameHeader) + sizeof(Packet) + sizeof(DroneBasicPayload) - 3;

	// convert short address to big endian
	tf.shortAddress = (tf.shortAddress & 0xFF00) >> 8 | (tf.shortAddress & 0x00FF) << 8;
	//convert destination address to big endian
	rev((uint8_t*)&tf.destAddress, sizeof(tf.destAddress));

	rawPacket[0] = tfArr[0];
	// this looks sketchy but it converts message length into big endian
	rawPacket[1] = tfArr[2];
	rawPacket[2] = tfArr[1];

	int index = 3;
	for (int x = 3; x < sizeof(TransmitFrameHeader); x++) {
			rawPacket[index] = tfArr[x];
			checksum -= tfArr[x];
			index++;
	}

	for (int x = 0; x < sizeof(Packet);x++) {
		rawPacket[index] = packetArr[x];
		checksum -= packetArr[x];
		index++;
	}

	for (int x = 0; x < sizeof(DroneBasicPayload);x++) {
			rawPacket[index] = payloadArr[x];
			checksum -= payloadArr[x];
			index++;
	}

	rawPacket[index] = checksum;

	HAL_UART_Transmit(&huart2, rawPacket, sizeof(rawPacket),UART_TIMEOUT);


}

/*
void TransmitMessage(DroneBasicPayload payload) {
	Packet packet;
	packet.setPayloadType(1);
	packet.setTime(HAL_GetTick());
	uint8_t* payloadArr = payload.getArr();//(uint8_t*)&payload;

	packet.setPayload(payloadArr, sizeof(DroneBasicPayload));

	uint8_t* packetArr = packet.getArr();
}
*/
