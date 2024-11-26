/*
 * mcp2515.c
 *
 *  Created on: 6 Jun 2024
 *      Author: Spiropoulos Vasilis
 */

#include "mcp2515.h"

void resetMCP2515() {
	XSpi_SetSlaveSelect(&SpiInstance, 0x04);		// Select CS for MCP2515

	u8 WriteBuffer[1] = {MCP2515_RESET};
	send_spi_data(WriteBuffer, sizeof(WriteBuffer));
	usleep(200000);	// delay 200 msec to allow the MCP2515 to reset
}

void setBitrateCan() {
	// Configure MCP2515 for 20 MHz oscillator and 33333 bps CAN Bus speed
	// Configuration registers CNF1, CNF2, CNF3 need to be set

	// CNF1: SJW=0, BRP=11
	writeRegisterCan(MCP2515_CNF1, 0x0B);  // 33.333 kbps @ 20 MHz
	// writeRegisterCan(MCP2515_CNF1, 0x4E);  // 33.333 kbps @ 16 MHz

	// CNF2: BTLMODE=1, SAM=1, PHSEG1=7, PRSEG=7
	writeRegisterCan(MCP2515_CNF2, 0xFF);  // 33.333 kbps @ 20 MHz
	// writeRegisterCan(MCP2515_CNF2, 0xF1);  // 33.333 kbps @ 16 MHz

	// CNF3: SOF=1, WAKFIL=0, PHSEG2=7
	writeRegisterCan(MCP2515_CNF3, 0x87);  // 33.333 kbps @ 20 MHz
	// writeRegisterCan(MCP2515_CNF3, 0x85);  // 33.333 kbps @ 16 MHz
}

void setNormalModeCan() {
	// Set Normal mode
	modifyRegisterCan(MCP2515_CANCTRL, MCP2515_REQOP_MASK, MCP2515_MODE_NORMAL);
}

void writeRegisterCan(u8 address, u8 value) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x04);		// Select CS for MCP2515

	u8 WriteBuffer[3] = {MCP2515_WRITE, address, value};
	send_spi_data(WriteBuffer, sizeof(WriteBuffer));
	usleep(1000);	// delay 1 msec
}

u8 readRegisterCan(u8 address) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x04);		// Select CS for MCP2515

	u8 WriteBuffer[3] = {MCP2515_READ, address, 0x00};
	u8 ReadBuffer[3];
	send_spi_data_read(WriteBuffer, ReadBuffer, sizeof(WriteBuffer));
	return ReadBuffer[2];
}

void modifyRegisterCan(u8 address, u8 mask, u8 value) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x04);		// Select CS for MCP2515

	u8 WriteBuffer[4] = {MCP2515_BIT_MODIFY, address, mask, value};
	send_spi_data(WriteBuffer, sizeof(WriteBuffer));
	usleep(1000);	// delay 1 msec
}

void writeSequentialMemoryCan(u8 address, u8 *data, u8 length) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x04);		// Select CS for MCP2515

	u8 WriteBuffer[length + 2];
	WriteBuffer[0] = MCP2515_WRITE;
	WriteBuffer[1] = address;
	for (u8 i = 0; i < length; i++) {
		WriteBuffer[i + 2] = data[i];
  }

	send_spi_data(WriteBuffer, sizeof(WriteBuffer));
	usleep(1000);	// delay 1 msec
}

bool isMCP2515NormalMode() {
	u8 value = readRegisterCan(MCP2515_CANCTRL);

	return ((value & MCP2515_MODE_MASK) == MCP2515_MODE_NORMAL);
}

void writeCommandCan(u8 address) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x04);		// Select CS for MCP2515

	u8 WriteBuffer[1] = {address};
	send_spi_data(WriteBuffer, sizeof(WriteBuffer));
	usleep(1000);	// delay 1 msec
}

void sendCANMessage(CAN_FRAME *can_message) {
	// Write message identifier
	writeRegisterCan(MCP2515_TXB0SIDH, ((can_message->id) >> 3) & 0xFF); // Bits 10-3
	writeRegisterCan(MCP2515_TXB0SIDL, ((can_message->id) << 5) & 0xE0); // Bits 2-0

	// Write message data length
	writeRegisterCan(MCP2515_TXB0DLC, ((can_message->length) & 0x0F));

	// Write message data
	writeSequentialMemoryCan(MCP2515_TXB0D0, can_message->data.byte, can_message->length);

	// Set TXREQ bit to request transmission
	writeCommandCan(MCP2515_RTS_TX0);
}

void setOneShotModeCan() {
	// Set MCP2515 to One-Shot mode
	modifyRegisterCan(MCP2515_CANCTRL, MCP2515_REQOP_MASK, MCP2515_MODE_CONFIG); // Set Configuration mode
	// Set One-Shot mode
	modifyRegisterCan(MCP2515_CANCTRL, MCP2515_OSM, MCP2515_OSM); // Set OSM bit
	modifyRegisterCan(MCP2515_CANCTRL, MCP2515_REQOP_MASK, MCP2515_MODE_NORMAL); // Set Normal mode
}

void RegularOperationMode() {
	// Deactivate MCP2515 One-Shot mode
	modifyRegisterCan(MCP2515_CANCTRL, MCP2515_REQOP_MASK, MCP2515_MODE_CONFIG); // Set Configuration mode
	// Clear One-Shot Mode Bit
	modifyRegisterCan(MCP2515_CANCTRL, MCP2515_OSM, 0x00); // Clear OSM bit
	modifyRegisterCan(MCP2515_CANCTRL, MCP2515_REQOP_MASK, MCP2515_MODE_NORMAL); // Set Normal mode
}

bool checkTXREQBitCan() {
	u8 TXB0CTRL_value = readRegisterCan(MCP2515_TXB0CTRL);
	u8 mask = 0x01 << MCP2515_TXREQ_BIT;
	return ((TXB0CTRL_value & mask) != 0);
}

