/*
 * gpio_api.h
 *
 *  Created on: 6 Jun 2024
 *      Author: Spiropoulos Vasilis
 */

#ifndef SRC_GPIO_API_H_
#define SRC_GPIO_API_H_

#include "xgpio.h"
#include "xil_printf.h"

#define GPIO_DEVICE_ID  XPAR_GPIO_0_DEVICE_ID
#define GPIO_CHANNEL 1
XGpio Gpio;		/* The Instance of the GPIO Driver */

int gpio_init();


#endif /* SRC_GPIO_API_H_ */
