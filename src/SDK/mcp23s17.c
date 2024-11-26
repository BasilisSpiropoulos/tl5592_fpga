/*
 * mcp23s17.c
 *
 *  Created on: 22 May 2024
 *      Author: Spiropoulos Vasilis
 */

#include "mcp23s17.h"

const u8 MCP[3] = {MCP_addr_1, MCP_addr_2, MCP_addr_3};


void mcp_reset() {
	XGpio_DiscreteWrite(&Gpio, GPIO_CHANNEL, 0x00000000);		// GPIO bit 0 connects to MCP23S17 reset active-low

	usleep(150000);	// delay 150 msec

	XGpio_DiscreteWrite(&Gpio, GPIO_CHANNEL, 0x00000001);		// bit 0 is MCP23S17 reset active-low
}

void mcp_writeData(u8 addrWR, u8 opcode, u8 data) {
	u8 buffer_size = 3;
	u8 WriteBuffer[buffer_size];

	WriteBuffer[0] = addrWR;
	WriteBuffer[1] = opcode;
	WriteBuffer[2] = data;

	send_spi_data(WriteBuffer, sizeof(WriteBuffer));
}

u8 mcp_readData(u8 addrWR, u8 opcode) {
	u8 buffer_size = 3;
	u8 WriteBuffer[buffer_size];
	u8 ReadBuffer[buffer_size];

	WriteBuffer[0] = addrWR;
	WriteBuffer[1] = opcode;
	WriteBuffer[2] = 0x00;

	send_spi_data_read(WriteBuffer, ReadBuffer, sizeof(WriteBuffer));
	return ReadBuffer[buffer_size - 1];
}

// Function to write to GPIO Port A or Port B
void mcp_setPort(u8 icNumber, u8 port, u8 value) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x01);					    // Select CS for MCP23S17s

	u8 MCPaddrW = MCP[icNumber] << 1 | 0x40;
	u8 opcode;
	if (port == 'A') {
		opcode = MCP23S17_GPIOA; // Write to GPIOA
	} else if (port == 'B') {
		opcode = MCP23S17_GPIOB; // Write to GPIOB
	}

	mcp_writeData(MCPaddrW, opcode, value);
}

// Function to read GPIO Port A or Port B
u8 mcp_getPort(u8 icNumber, u8 port) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x01);					    // Select CS for MCP23S17s

	u8 MCPaddrR = MCP[icNumber] << 1 | 0x41;
	u8 opcode;
	u8 value = 0x00;
    if (port == 'A') {
    	opcode = MCP23S17_GPIOA; // Write to GPIOA
    } else if (port == 'B') {
    	opcode = MCP23S17_GPIOB; // Write to GPIOB
    }

    value = mcp_readData(MCPaddrR, opcode);
    return value;
}

// Function to initialize MCP23S17
void initMCP23S17() {
	XSpi_SetSlaveSelect(&SpiInstance, 0x01);					    // Select CS for MCP23S17s

	// Initialize MCP23S17 devices
	// Device 0x20 (Address for the 1st MCP23S17)
	u8 addr_1 = MCP[0] << 1 | 0x40;
	mcp_writeData(addr_1, MCP23S17_IODIRA, 0xFF);   // Set IODIRA (Port A direction) to 0xFF (all inputs)
	mcp_writeData(addr_1, MCP23S17_IODIRB, 0xFF);   // Set IODIRB (Port B direction) to 0xFF (all inputs)
	mcp_writeData(addr_1, MCP23S17_IOPOLA, 0x00);   // Set IOPOLA (Port A polarity) to 0x00 (no inversion)
	mcp_writeData(addr_1, MCP23S17_IOPOLB, 0x00);   // Set IOPOLB (Port B polarity) to 0x00 (no inversion)
	mcp_writeData(addr_1, MCP23S17_GPINTENA, 0xFF); // Enable interrupts on all Port A pins
	mcp_writeData(addr_1, MCP23S17_GPINTENB, 0xFF); // Enable interrupts on all Port B pins
	mcp_writeData(addr_1, MCP23S17_DEFVALA, 0x00);  // Default value for comparison on interrupts for Port A pins
	mcp_writeData(addr_1, MCP23S17_DEFVALB, 0x00);  // Default value for comparison on interrupts for Port B pins
	mcp_writeData(addr_1, MCP23S17_INTCONA, 0x00);  // Set interrupt-on-change control for Port A pins (compared against the previous pin value)
	mcp_writeData(addr_1, MCP23S17_INTCONB, 0x00);  // Set interrupt-on-change control for Port B pins (compared against the previous pin value)
	mcp_writeData(addr_1, MCP23S17_IOCON, 0x6A);    // Configure IOCON (I/O expander configuration) register
	mcp_writeData(addr_1, MCP23S17_GPPUA, 0x00);    // Disable internal pull-up resistors for Port A pins
	mcp_writeData(addr_1, MCP23S17_GPPUB, 0x00);    // Disable internal pull-up resistors for Port B pins

	// Device 0x21 (Address for the 2nd MCP23S17)
	u8 addr_2 = MCP[1] << 1 | 0x40;
	mcp_writeData(addr_2, MCP23S17_IODIRA, 0xFF);   // Set IODIRA (Port A direction) to 0xFF (all inputs)
	mcp_writeData(addr_2, MCP23S17_IODIRB, 0xFF);   // Set IODIRB (Port B direction) to 0xFF (all inputs)
	mcp_writeData(addr_2, MCP23S17_IOPOLA, 0x00);   // Set IOPOLA (Port A polarity) to 0x00 (no inversion)
	mcp_writeData(addr_2, MCP23S17_IOPOLB, 0x00);   // Set IOPOLB (Port B polarity) to 0x00 (no inversion)
	mcp_writeData(addr_2, MCP23S17_GPINTENA, 0xFF); // Enable interrupts on all Port A pins
	mcp_writeData(addr_2, MCP23S17_GPINTENB, 0xFF); // Enable interrupts on all Port B pins
	mcp_writeData(addr_2, MCP23S17_DEFVALA, 0x00);  // Default value for comparison on interrupts for Port A pins
	mcp_writeData(addr_2, MCP23S17_DEFVALB, 0x00);  // Default value for comparison on interrupts for Port B pins
	mcp_writeData(addr_2, MCP23S17_INTCONA, 0x00);  // Set interrupt-on-change control for Port A pins (compared against the previous pin value)
	mcp_writeData(addr_2, MCP23S17_INTCONB, 0x00);  // Set interrupt-on-change control for Port B pins (compared against the previous pin value)
	mcp_writeData(addr_2, MCP23S17_IOCON, 0x6A);    // Configure IOCON (I/O expander configuration) register
	mcp_writeData(addr_2, MCP23S17_GPPUA, 0x00);    // Disable internal pull-up resistors for Port A pins
	mcp_writeData(addr_2, MCP23S17_GPPUB, 0x00);    // Disable internal pull-up resistors for Port B pins

	// Device 0x22 (Address for the 3rd MCP23S17)
	u8 addr_3 = MCP[2] << 1 | 0x40;
	mcp_writeData(addr_3, MCP23S17_IODIRA, 0xFF);   // Set IODIRA (Port A direction) to 0xFF (all inputs)
	mcp_writeData(addr_3, MCP23S17_IODIRB, 0x0F);   // Set IODIRB (Port B direction) to 0x0F (all inputs, except upper half nibble (bits 4-7) as outputs)
	mcp_writeData(addr_3, MCP23S17_IOPOLA, 0x00);   // Set IOPOLA (Port A polarity) to 0x00 (no inversion)
	mcp_writeData(addr_3, MCP23S17_IOPOLB, 0x00);   // Set IOPOLB (Port B polarity) to 0x00 (no inversion)
	mcp_writeData(addr_3, MCP23S17_GPINTENA, 0xFF); // Enable interrupts on all Port A pins
	mcp_writeData(addr_3, MCP23S17_GPINTENB, 0x0F); // Enable interrupts on all Port B (input) pins
	mcp_writeData(addr_3, MCP23S17_DEFVALA, 0x00);  // Default value for comparison on interrupts for Port A pins
	mcp_writeData(addr_3, MCP23S17_DEFVALB, 0x00);  // Default value for comparison on interrupts for Port B pins
	mcp_writeData(addr_3, MCP23S17_INTCONA, 0x00);  // Set interrupt-on-change control for Port A pins (compared against the previous pin value)
	mcp_writeData(addr_3, MCP23S17_INTCONB, 0x00);  // Set interrupt-on-change control for Port B pins (compared against the previous pin value)
	mcp_writeData(addr_3, MCP23S17_IOCON, 0x6A);    // Configure IOCON (I/O expander configuration) register
	mcp_writeData(addr_3, MCP23S17_GPPUA, 0x00);    // Disable internal pull-up resistors for Port A pins
	mcp_writeData(addr_3, MCP23S17_GPPUB, 0x00);    // Disable internal pull-up resistors for Port B pins
}

// Function to read MCP23S17 interrupt captured registers
void mcp_intFrame(u8 *intFrame) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x01);					    // Select CS for MCP23S17s

	// Device 0x20 (Address for the 1st MCP23S17)
	u8 addr_1 = MCP[0] << 1 | 0x41;
	* intFrame    = mcp_readData(addr_1, MCP23S17_INTCAPA);   // Read INTCAPA (Port A interrupt captured register)
	*(intFrame+1) = mcp_readData(addr_1, MCP23S17_INTCAPB);   // Read INTCAPB (Port B interrupt captured register)

	// Device 0x21 (Address for the 2nd MCP23S17)
	u8 addr_2 = MCP[1] << 1 | 0x41;
	*(intFrame+2) = mcp_readData(addr_2, MCP23S17_INTCAPA);   // Read INTCAPA (Port A interrupt captured register)
	*(intFrame+3) = mcp_readData(addr_2, MCP23S17_INTCAPB);   // Read INTCAPB (Port B interrupt captured register)

	// Device 0x22 (Address for the 3rd MCP23S17)
	u8 addr_3 = MCP[2] << 1 | 0x41;
	*(intFrame+4) = mcp_readData(addr_3, MCP23S17_INTCAPA);   // Read INTCAPA (Port A interrupt captured register)
	*(intFrame+5) = mcp_readData(addr_3, MCP23S17_INTCAPB);   // Read INTCAPB (Port B interrupt captured register)
}

// Function to dump MCP23S17 register file
void dumpRegMCP23S17() {
	XSpi_SetSlaveSelect(&SpiInstance, 0x01);					    // Select CS for MCP23S17s

	u8 value;
	// Device 0x20 (Address for the 1st MCP23S17)
	u8 addr_1 = MCP[0] << 1 | 0x41;
	value = mcp_readData(addr_1, MCP23S17_IODIRA);   // Read IODIRA (Port A direction)
	xil_printf("IC 1, IODIRA : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_IODIRB);   // Read IODIRB (Port B direction)
	xil_printf("IC 1, IODIRB : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_IOPOLA);   // Read IOPOLA (Port A polarity)
	xil_printf("IC 1, IOPOLA : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_IOPOLB);   // Read IOPOLB (Port B polarity)
	xil_printf("IC 1, IOPOLB : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_GPINTENA); // Read interrupt-enable for Port A
	xil_printf("IC 1, GPINTENA : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_GPINTENB); // Read interrupt-enable for Port B
	xil_printf("IC 1, GPINTENB : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_DEFVALA);  // Read default value for comparison on interrupts for Port A pins
	xil_printf("IC 1, DEFVALA : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_DEFVALB);  // Read default value for comparison on interrupts for Port B pins
	xil_printf("IC 1, DEFVALB : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_INTCONA);  // Read interrupt-on-change control for Port A pins
	xil_printf("IC 1, INTCONA : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_INTCONB);  // Read interrupt-on-change control for Port B pins
	xil_printf("IC 1, INTCONB : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_IOCON);    // Read IOCON (I/O expander configuration) register
	xil_printf("IC 1, IOCON : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_GPPUA);    // Read internal pull-up state for Port A pins
	xil_printf("IC 1, GPPUA : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_GPPUB);    // Read internal pull-up state for Port B pins
	xil_printf("IC 1, GPPUB : %02X \n", value);
	xil_printf("-------------------- \n", value);

	// Device 0x21 (Address for the 2nd MCP23S17)
	u8 addr_2 = MCP[1] << 1 | 0x41;
	value = mcp_readData(addr_2, MCP23S17_IODIRA);   //	Read IODIRA (Port A direction)
	xil_printf("IC 2, IODIRA : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_IODIRB);   // Read IODIRB (Port B direction)
	xil_printf("IC 2, IODIRB : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_IOPOLA);   // Read IOPOLA (Port A polarity)
	xil_printf("IC 2, IOPOLA : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_IOPOLB);   // Read IOPOLB (Port B polarity)
	xil_printf("IC 2, IOPOLB : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_GPINTENA); // Read interrupt-enable for Port A
	xil_printf("IC 2, GPINTENA : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_GPINTENB); // Read interrupt-enable for Port B
	xil_printf("IC 2, GPINTENB : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_DEFVALA);  // Read default value for comparison on interrupts for Port A pins
	xil_printf("IC 2, DEFVALA : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_DEFVALB);  // Read default value for comparison on interrupts for Port B pins
	xil_printf("IC 2, DEFVALB : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_INTCONA);  // Read interrupt-on-change control for Port A pins
	xil_printf("IC 2, INTCONA : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_INTCONB);  // Read interrupt-on-change control for Port B pins
	xil_printf("IC 2, INTCONB : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_IOCON);    // Read IOCON (I/O expander configuration) register
	xil_printf("IC 2, IOCON : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_GPPUA);    // Read internal pull-up state for Port A pins
	xil_printf("IC 2, GPPUA : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_GPPUB);    // Read internal pull-up state for Port B pins
	xil_printf("IC 2, GPPUB : %02X \n", value);
	xil_printf("-------------------- \n", value);

	// Device 0x22 (Address for the 3rd MCP23S17)
	u8 addr_3 = MCP[2] << 1 | 0x41;
	value = mcp_readData(addr_3, MCP23S17_IODIRA);   // Read IODIRA (Port A direction)
	xil_printf("IC 3, IODIRA : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_IODIRB);   // Read IODIRB (Port B direction)
	xil_printf("IC 3, IODIRB : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_IOPOLA);   // Read IOPOLA (Port A polarity)
	xil_printf("IC 3, IOPOLA : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_IOPOLB);   // Read IOPOLA (Port B polarity)
	xil_printf("IC 3, IOPOLB : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_GPINTENA); // Read interrupt-enable for Port A
	xil_printf("IC 3, GPINTENA : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_GPINTENB); // Read interrupt-enable for Port B
	xil_printf("IC 3, GPINTENB : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_DEFVALA);  // Read default value for comparison on interrupts for Port A pins
	xil_printf("IC 3, DEFVALA : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_DEFVALB);  // Read default value for comparison on interrupts for Port B pins
	xil_printf("IC 3, DEFVALB : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_INTCONA);  // Read interrupt-on-change control for Port A pins
	xil_printf("IC 3, INTCONA : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_INTCONB);  // Read interrupt-on-change control for Port B pins
	xil_printf("IC 3, INTCONB : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_IOCON);    // Read IOCON (I/O expander configuration) register
	xil_printf("IC 3, IOCON : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_GPPUA);    // Read internal pull-up state for Port A pins
	xil_printf("IC 3, GPPUA : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_GPPUB);    // Read internal pull-up state for Port A pins
	xil_printf("IC 3, GPPUB : %02X \n", value);
	xil_printf("-------------------- \n", value);
}

// Function to dump MCP23S17 interrupt frame
void dumpIntFrameMCP23S17() {
	XSpi_SetSlaveSelect(&SpiInstance, 0x01);					    // Select CS for MCP23S17s

	u8 value;
	// Device 0x20 (Address for the 1st MCP23S17)
	u8 addr_1 = MCP[0] << 1 | 0x41;
	value = mcp_readData(addr_1, MCP23S17_INTFA);   // Read INTFA register from the 1st MCP23S17
	xil_printf("IC 1, INTFA : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_INTFB);   // Read INTFB register from the 1st MCP23S17
	xil_printf("IC 1, INTFB : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_INTCAPA);	// Read INTCAPA register from the 1st MCP23S17
	xil_printf("IC 1, INTCAPA : %02X \n", value);
	value = mcp_readData(addr_1, MCP23S17_INTCAPB);	// Read INTCAPB register from the 1st MCP23S17
	xil_printf("IC 1, INTCAPB : %02X \n", value);
	xil_printf("-------------------- \n", value);

	// Device 0x21 (Address for the 2nd MCP23S17)
	u8 addr_2 = MCP[1] << 1 | 0x41;
	value = mcp_readData(addr_2, MCP23S17_INTFA);   // Read INTFA register from the 2nd MCP23S17
	xil_printf("IC 2, INTFA : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_INTFB);   // Read INTFB register from the 2nd MCP23S17
	xil_printf("IC 2, INTFB : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_INTCAPA);	// Read INTCAPA register from the 2nd MCP23S17
	xil_printf("IC 2, INTCAPA : %02X \n", value);
	value = mcp_readData(addr_2, MCP23S17_INTCAPB);	// Read INTCAPB register from the 2nd MCP23S17
	xil_printf("IC 2, INTCAPB : %02X \n", value);
	xil_printf("-------------------- \n", value);

	// Device 0x22 (Address for the 3rd MCP23S17)
	u8 addr_3 = MCP[2] << 1 | 0x41;
	value = mcp_readData(addr_3, MCP23S17_INTFA);   // Read INTFA register from the 3rd MCP23S17
	xil_printf("IC 3, INTFA : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_INTFB);   // Read INTFB register from the 3rd MCP23S17
	xil_printf("IC 3, INTFB : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_INTCAPA);	// Read INTCAPA register from the 3rd MCP23S17
	xil_printf("IC 3, INTCAPA : %02X \n", value);
	value = mcp_readData(addr_3, MCP23S17_INTCAPB);	// Read INTCAPB register from the 3rd MCP23S17
	xil_printf("IC 3, INTCAPB : %02X \n", value);
	xil_printf("-------------------- \n", value);
}
