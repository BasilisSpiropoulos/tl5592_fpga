/*
 * max7221.c
 *
 *  Created on: 2 May 2024
 *      Author: Spiropoulos Vasilis
 */


#include "max7221.h"


void initMAX7221(u8 icNumber) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x02);					    // Select CS for MAX7221s
	sendSPICommand(icNumber, MAX7221_SHUTDOWN_REG, MAX7221_SHUTDOWN_MODE); // Put MAX7221 into shutdown mode
	usleep(10000);	// delay 10 msec


	// Turn off all LEDs
	for (u8 digit = 0; digit < 8; digit++)
		sendSPICommand(icNumber, MAX7221_DIGIT0_REG + digit, 0x00);

	sendSPICommand(icNumber, MAX7221_DECODE_MODE_REG, MAX7221_DECODE_NONE); // Disable digit decoding
	sendSPICommand(icNumber, MAX7221_INTENSITY_REG, MAX7221_INTENSITY_LEVEL); // Set intensity level
	sendSPICommand(icNumber, MAX7221_SCAN_LIMIT_REG, MAX7221_SCAN_LIMIT_ALL); // Set scan limit to all digits
	sendSPICommand(icNumber, MAX7221_DISPLAY_TEST_REG, MAX7221_DISPLAY_TEST_NORMAL); // Disable display test
	sendSPICommand(icNumber, MAX7221_SHUTDOWN_REG, MAX7221_SHUTDOWN_NORMAL); // Bring MAX7221 out of shutdown mode
}

void setIntensity(u8 icNumber, u8 intensity) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x02);					    // Select CS for MAX7221s
	sendSPICommand(icNumber, MAX7221_INTENSITY_REG, intensity); // Set intensity
}

void setRow(u8 icNumber, u8 digit, u8 value) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x02);					    // Select CS for MAX7221s
	sendSPICommand(icNumber, MAX7221_DIGIT0_REG + digit, value);
}

void sendSPICommand(u8 icNumber, u8 opcode, u8 data) {
	u8 buffer_size;
	buffer_size = MAX_CNT * 2;	// 16 bits (2 bytes) per MAX7221
	u8 WriteBuffer[buffer_size];
	for (u8 i = 0; i < buffer_size; i++)
		WriteBuffer[i] = 0;

	WriteBuffer[4 - (icNumber * 2)] = opcode;			// = ((3 - icNumber) * 2) - 2
	WriteBuffer[5 - (icNumber * 2)] = data;			// = ((3 - icNumber) * 2) - 1

	send_spi_data(WriteBuffer, sizeof(WriteBuffer));
}

void setDecode(u8 icNumber, u8 value) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x02);					    // Select CS for MAX7221s
	// Make sure the value is within the valid range
	if (value > 0xFF) {
		value = 0xFF; // Limit value to 0xFF (bits 0-7)
	}

	// Set the decode mode register
	sendSPICommand(icNumber, MAX7221_DECODE_MODE_REG, value);
}

void setNum(u8 icNumber, u8 digit, u8 value) {
	XSpi_SetSlaveSelect(&SpiInstance, 0x02);					    // Select CS for MAX7221s
	// Display number in one segment (decode mode must be enabled on this segment)
	sendSPICommand(icNumber, MAX7221_DIGIT0_REG + digit, value);
}


void testMax7221() {
	XSpi_SetSlaveSelect(&SpiInstance, 0x02);					    // Select CS for MAX7221s
	u8 value = 0;
	for (u8 k = 0; k < 3; k++) { // Iterate over each MAX7221
		for (u8 i = 0; i < 8; i++) { // Iterate over each digit
			for (u8 j = 0; j < 8; j++) { // Iterate over each segment
				value = 1 << j;
				setRow(MAX_1 + k, i, value); // Turn on segment j of digit i
				//for (u32 delay_time = 0; delay_time < 1250000; delay_time++) {}	// 120us per 1000 iterations => 150ms
				usleep(150000);	// delay 150 msec
			}
			setRow(MAX_1 + k, i, 0); // Turn off all segments of digit i
		}
	}

	for (u8 k = 0; k < 3; k++) { // Iterate over each MAX7221
		for (u8 i = 0; i < 8; i++) { // Iterate over each digit
			setRow(MAX_1 + k, i, 0xFF); // Turn on all segments of digit i
	  }
	}
	sleep(1);	// delay 1 sec

	for (u8 k = 0; k < 3; k++) { // Iterate over each MAX7221
		for (u8 i = 0; i < 8; i++) { // Iterate over each digit
			setRow(MAX_1 + k, i, 0x00); // Turn off all segments of digit i
		}
	}
	sleep(1);	// delay 1 sec

	for (u8 k = 0; k < 3; k++) { // Iterate over each MAX7221
		for (u8 i = 0; i < 8; i++) { // Iterate over each digit
			setRow(MAX_1 + k, i, 0xFF); // Turn on all segments of digit i
		}
	}
	//for (u32 delay_time = 0; delay_time < 8000000; delay_time++) {}	// 120us per 1000 iterations => 960ms
	sleep(1);	// delay 1 sec

	for (u8 k = 0; k < 3; k++) { // Iterate over each MAX7221
		for (u8 i = 0; i < 8; i++) { // Iterate over each digit
			setRow(MAX_1 + k, i, 0x00); // Turn off all segments of digit i
		}
	}
	sleep(1);	// delay 1 sec
}

void fastTestMax7221() {
	XSpi_SetSlaveSelect(&SpiInstance, 0x02);					    // Select CS for MAX7221s
	for (u8 j = 0; j < 3; j++) { // Set which MAX7221 will be On, others Off
		for (u8 k = 0; k < 3; k++) { // Iterate over each MAX7221
			for (u8 i = 0; i < 8; i++) { // Iterate over each digit
				if (k == j){
					setRow(MAX_1 + k, i, 0xFF); // Turn on all segments of digit i
				}else{
					setRow(MAX_1 + k, i, 0x00); // Turn off all segments of digit i
				}
			}
		}
		usleep(250000);	// delay 250 msec
	}

	for (u8 k = 0; k < 3; k++) { // Iterate over each MAX7221
		for (u8 i = 0; i < 8; i++) { // Iterate over each digit
			setRow(MAX_1 + k, i, 0x00); // Turn off all segments of digit i
		}
	}
 	usleep(250000);	// delay 250 msec
}
