/*
 * max7221.h
 *
 *  Created on: 2 May 2024
 *      Author: Spiropoulos Vasilis
 */

#ifndef SRC_MAX7221_H_
#define SRC_MAX7221_H_

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xspi.h"
#include "xstatus.h"
#include "sleep.h"
#include "spi_api.h"	// include before ICs header files

// MAX7221 constants
#define MAX_1				 0x00
#define MAX_2				 0x01
#define MAX_3				 0x02
#define MAX_CNT			 0x03		// number of MAX7221 ICs

#define MAX7221_INTENSITY_REG  0x0A
#define MAX7221_NO_OP_REG    0x00
#define MAX7221_DIGIT0_REG   0x01
#define MAX7221_DIGIT1_REG   0x02
#define MAX7221_DIGIT2_REG   0x03
#define MAX7221_DIGIT3_REG   0x04
#define MAX7221_DIGIT4_REG   0x05
#define MAX7221_DIGIT5_REG   0x06
#define MAX7221_DIGIT6_REG   0x07
#define MAX7221_DIGIT7_REG   0x08
#define MAX7221_DECODE_MODE_REG 0x09
#define MAX7221_INTENSITY_REG   0x0A
#define MAX7221_SCAN_LIMIT_REG  0x0B
#define MAX7221_SHUTDOWN_REG    0x0C
#define MAX7221_DISPLAY_TEST_REG  0x0F

#define MAX7221_DECODE_NONE 0x00
#define MAX7221_INTENSITY_LEVEL 0x05
#define MAX7221_SCAN_LIMIT_ALL 0x07
#define MAX7221_SHUTDOWN_NORMAL 0x01
#define MAX7221_SHUTDOWN_MODE 0x00
#define MAX7221_DISPLAY_TEST_NORMAL 0x00


void initMAX7221(u8 icNumber);
void setIntensity(u8 icNumber, u8 intensity);
void setRow(u8 icNumber, u8 digit, u8 value);
void sendSPICommand(u8 icNumber, u8 opcode, u8 data);
void setDecode(u8 icNumber, u8 value);
void setNum(u8 icNumber, u8 digit, u8 value);
void testMax7221();
void fastTestMax7221();


#endif /* SRC_MAX7221_H_ */
