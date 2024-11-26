/*
 * mcp2515.h
 *
 *  Created on: 6 Jun 2024
 *      Author: Spiropoulos Vasilis
 */

#ifndef SRC_MCP2515_H_
#define SRC_MCP2515_H_


#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xspi.h"
#include "xstatus.h"
#include "sleep.h"
#include "spi_api.h"	// include before ICs header files
#include "stdbool.h"


// MCP2515 registers
#define MCP2515_RESET           0xC0
#define MCP2515_WRITE           0x02
#define MCP2515_READ            0x03
#define MCP2515_BIT_MODIFY      0x05

// MCP2515 control register addresses
#define MCP2515_TXB0CTRL        0x30
#define MCP2515_TXB0SIDH        0x31
#define MCP2515_TXB0SIDL        0x32
#define MCP2515_RTS_TX0         0x81
#define MCP2515_CANCTRL         0x0F
#define MCP2515_CNF1            0x2A
#define MCP2515_CNF2            0x29
#define MCP2515_CNF3            0x28
#define MCP2515_CANINTE         0x2B
#define MCP2515_CANINTF         0x2C
#define MCP2515_TXB0DLC         0x35
#define MCP2515_TXB0D0          0x36
#define MCP2515_TXREQ_BIT          3

// MCP2515 control register values
#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_CONFIG     0x80
#define MCP2515_REQOP_MASK      0xE0
#define MCP2515_ABAT            0x10
#define MCP2515_OSM             0x08
#define MCP2515_CLKEN           0x04
#define MCP2515_CLKPRE_MASK     0x03
#define MCP2515_SJW_MASK        0xC0
#define MCP2515_BRP_MASK        0x3F

// CANCTRL register values
#define MCP2515_MODE_MASK       0xE0


typedef union {
	u64 value;		// LSB (8-bit) of value is the 1st transmitted data
	struct {
		u32 low;		// LSB (8-bit) of low is the 1st transmitted data
		u32 high;
	};
	struct {
		u16 s0;			// LSB (8-bit) of s0 is the 1st transmitted data
		u16 s1;
		u16 s2;
		u16 s3;
    };
	u8 byte[8];		// byte[0] is the 1st transmitted data
} BytesUnion;


typedef struct {
	u32 id;
	u8 length;
	BytesUnion data;
} CAN_FRAME;


void resetMCP2515();
void setBitrateCan();		// Set to 33333bps for 20 MHz MCP2515 clock
void setNormalModeCan();
void writeRegisterCan(u8 address, u8 value);
u8 readRegisterCan(u8 address);
void modifyRegisterCan(u8 address, u8 mask, u8 value);
void writeSequentialMemoryCan(u8 address, u8 *data, u8 length);
bool isMCP2515NormalMode();
void writeCommandCan(u8 address);
void sendCANMessage(CAN_FRAME *can_message);
void setOneShotModeCan();
void RegularOperationMode();
bool checkTXREQBitCan();


#endif /* SRC_MCP2515_H_ */
