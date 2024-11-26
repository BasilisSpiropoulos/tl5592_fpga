/*
 * spi_api.c
 *
 *  Created on: 26 Apr 2024
 *      Author: Spiropoulos Vasilis
 */

#include "spi_api.h"


int spi_init() {
	XSpi_Config *ConfigPtr;
	int Status;

	ConfigPtr = XSpi_LookupConfig(SPI_DEVICE_ID);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}

	Status = XSpi_CfgInitialize(&SpiInstance, ConfigPtr, ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XSpi_SetOptions(&SpiInstance, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XSpi_Start(&SpiInstance);

	return XST_SUCCESS;
}


void send_spi_data(u8 *Data, int ByteCount) {
	XSpi_Transfer(&SpiInstance, Data, NULL, ByteCount);
}


void send_spi_data_read(u8 *DataW, u8 *DataR, int ByteCount) {
	XSpi_Transfer(&SpiInstance, DataW, DataR, ByteCount);
}
