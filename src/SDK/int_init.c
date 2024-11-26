/*
 * int_init.c
 *
 *  Created on: 25 Jun 2024
 *      Author: Spiropoulos Vasilis
 */

#include "int_init.h"


// Function to set the interrupt type to edge-sensitive
void SetInterruptEdgeType() {
	uint32_t Mask = XIntc_In32(XPAR_INTC_0_BASEADDR + XIN_INT0_EDGE_OFFSET);

	// Set the bit corresponding to the external interrupt pin to 1 for edge-sensitive
	Mask |= XPAR_SYSTEM_MCP_INT_MASK; // Use the mask defined in xparameters.h

	XIntc_Out32(XPAR_INTC_0_BASEADDR + XIN_INT0_EDGE_OFFSET, Mask);
}


int initInterruptController() {
	// Interrupt Controller stuff
	int Status;
	XTmrCtr_Config *TimerConfig;

	// Initialize the AXI Timer
	TimerConfig = XTmrCtr_LookupConfig(TIMER_DEVICE_ID);
	if (TimerConfig == NULL) {
		xil_printf("Timer configuration lookup failed.\r\n");
		return XST_FAILURE;
	}

	// Initialize the timer
	XTmrCtr_CfgInitialize(&TimerInstance, TimerConfig, TimerConfig->BaseAddress);

	// Set the load value for Timer 1 (Count down from this value)
	xil_printf("Setting Timer Load Value: %u\r\n", TIMER1_LOAD_VALUE);
	XTmrCtr_SetResetValue(&TimerInstance, 0, TIMER1_LOAD_VALUE);  // Using Timer 1 (index 0)

	// Set options for down-counting, auto-reload, and interrupt mode for Timer 1
	XTmrCtr_SetOptions(&TimerInstance, 0, XTC_DOWN_COUNT_OPTION | XTC_AUTO_RELOAD_OPTION | XTC_INT_MODE_OPTION);


	// Set the load value for Timer 2 (Count down from this value)
	xil_printf("Setting Timer 2 Load Value: %u\r\n", TIMER2_LOAD_VALUE);
	XTmrCtr_SetResetValue(&TimerInstance, 1, TIMER2_LOAD_VALUE);  // Using Timer 2 (index 1)

	// Set options for down-counting, auto-reload, and interrupt mode for Timer 2
	XTmrCtr_SetOptions(&TimerInstance, 1, XTC_DOWN_COUNT_OPTION | XTC_AUTO_RELOAD_OPTION | XTC_INT_MODE_OPTION);


	// Initialize the interrupt controller
	Status = XIntc_Initialize(&InterruptController, INTC_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("Initializing interrupt controller failed.\r\n");
		return XST_FAILURE;
	}

	// Connect the interrupt handler for the external pin
	Status = XIntc_Connect(&InterruptController, XPAR_MICROBLAZE_0_AXI_INTC_SYSTEM_MCP_INT_INTR,
                           (XInterruptHandler)IntPinHandler, NULL);
	if (Status != XST_SUCCESS) {
		xil_printf("Connecting external pin interrupt handler failed.\r\n");
		return XST_FAILURE;
	}

	// Connect the timer interrupt handler
	Status = XIntc_Connect(&InterruptController, TIMER_INTERRUPT_ID,
                           (XInterruptHandler)XTmrCtr_InterruptHandler,
                           (void *)&TimerInstance);
	if (Status != XST_SUCCESS) {
		xil_printf("Connecting timer interrupt handler failed.\r\n");
		return XST_FAILURE;
	}

	// Set the interrupt type to edge-sensitive
	SetInterruptEdgeType();

	// Start the interrupt controller
	Status = XIntc_Start(&InterruptController, XIN_REAL_MODE);
	if (Status != XST_SUCCESS) {
		xil_printf("Starting interrupt controller failed.\r\n");
		return XST_FAILURE;
	}

	// Enable interrupts
	XIntc_Enable(&InterruptController, XPAR_MICROBLAZE_0_AXI_INTC_SYSTEM_MCP_INT_INTR);
	XIntc_Enable(&InterruptController, TIMER_INTERRUPT_ID);

	// Initialize exceptions and register the interrupt handler
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)XIntc_InterruptHandler,
                                 &InterruptController);
	Xil_ExceptionEnable();

	// Set the timer handler only once and check the TmrCtrNumber inside the handler
	XTmrCtr_SetHandler(&TimerInstance, TimerHandler, &TimerInstance);

	// Start both timers
	XTmrCtr_Start(&TimerInstance, 0);  // Start Timer 1
	XTmrCtr_Start(&TimerInstance, 1);  // Start Timer 2

	return XST_SUCCESS;
}

void stopTimer2(){
	// Disable Timer 2 interrupts
	XTmrCtr_Stop(&TimerInstance, 1);  // Stop Timer 2

	// Optional: Clear any pending Timer 2 interrupts
	XIntc_Acknowledge(&InterruptController, TIMER_INTERRUPT_ID);  // Assuming TIMER_INTERRUPT_ID is the interrupt ID for Timer 2

	// Print a message to indicate Timer 2 interrupts are disabled
	xil_printf("Timer 2 interrupts disabled.\r\n");
}

void startTimer2() {
	// Start Timer 2
	XTmrCtr_Start(&TimerInstance, 1);  // Start Timer 2

	// Enable Timer 2 interrupts in the interrupt controller
	XIntc_Enable(&InterruptController, TIMER_INTERRUPT_ID);

	// Print a message to indicate Timer 2 interrupts are re-enabled
	xil_printf("Timer 2 interrupts re-enabled.\r\n");
}
