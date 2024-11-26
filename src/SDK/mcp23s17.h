/*
 * mcp23s17.h
 *
 *  Created on: 22 May 2024
 *      Author: Spiropoulos Vasilis
 */

#ifndef SRC_MCP23S17_H_
#define SRC_MCP23S17_H_

#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xspi.h"
#include "xstatus.h"
#include "xil_printf.h"
#include "sleep.h"
#include "spi_api.h"	// include before ICs header files
#include "gpio_api.h"	// include before ICs header files


// MCP23S17 Register Addresses
#define MCP23S17_IODIRA 0x00
#define MCP23S17_IODIRB 0x01
#define MCP23S17_IOPOLA 0x02
#define MCP23S17_IOPOLB 0x03
#define MCP23S17_GPINTENA 0x04
#define MCP23S17_GPINTENB 0x05
#define MCP23S17_DEFVALA 0x06
#define MCP23S17_DEFVALB 0x07
#define MCP23S17_INTCONA 0x08
#define MCP23S17_INTCONB 0x09
#define MCP23S17_IOCON 0x0A
#define MCP23S17_GPPUA 0x0C
#define MCP23S17_GPPUB 0x0D
#define MCP23S17_GPIOA 0x12
#define MCP23S17_GPIOB 0x13

#define MCP23S17_INTFA 0x0E
#define MCP23S17_INTFB 0x0F
#define MCP23S17_INTCAPA 0x10
#define MCP23S17_INTCAPB 0x11

#define MCP_addr_1	   0x20
#define MCP_addr_2	   0x21
#define MCP_addr_3	   0x22


void mcp_reset();
void mcp_writeData(u8 addrWR, u8 opcode, u8 data);
u8 mcp_readData(u8 addrWR, u8 opcode);
void mcp_setPort(u8 icNumber, u8 port, u8 value);
u8 mcp_getPort(u8 icNumber, u8 port);
void initMCP23S17();
void mcp_intFrame(u8 *intFrame);
void dumpRegMCP23S17();
void dumpIntFrameMCP23S17();


#endif /* SRC_MCP23S17_H_ */
