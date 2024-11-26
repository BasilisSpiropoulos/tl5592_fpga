/*
 * spi_api.h
 *
 *  Created on: 26 Apr 2024
 *      Author: Spiropoulos Vasilis
 */

#ifndef SRC_SPI_API_H_
#define SRC_SPI_API_H_


#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xspi.h"
#include "xstatus.h"
#include "sleep.h"

#define SPI_DEVICE_ID XPAR_AXI_QUAD_SPI_0_DEVICE_ID

XSpi SpiInstance;


int spi_init();
void send_spi_data(u8 *Data, int ByteCount);
void send_spi_data_read(u8 *DataW, u8 *DataR, int ByteCount);


#endif /* SRC_SPI_API_H_ */
