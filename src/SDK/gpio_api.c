/*
 * gpio_api.c
 *
 *  Created on: 6 Jun 2024
 *      Author: Spiropoulos Vasilis
 */

#include "gpio_api.h"

int gpio_init(){
	int Status;
	// Initialize the GPIO driver
	#ifndef SDT
		Status = XGpio_Initialize(&Gpio, GPIO_DEVICE_ID);
	#else
		Status = XGpio_Initialize(&Gpio, XGPIO_AXI_BASEADDRESS);
	#endif

	if (Status != XST_SUCCESS) {
		xil_printf("Gpio Initialization Failed\r\n");
		return XST_FAILURE;
	}

	// Set the GPIO direction to all outputs
	XGpio_SetDataDirection(&Gpio, GPIO_CHANNEL, 0x00000000);

	return XST_SUCCESS;
}
