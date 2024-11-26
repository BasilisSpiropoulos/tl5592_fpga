/*
 * int_init.h
 *
 *  Created on: 25 Jun 2024
 *      Author: Spiropoulos Vasilis
 */

#ifndef SRC_INT_INIT_H_
#define SRC_INT_INIT_H_

#include "xparameters.h"
#include "xil_exception.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "xil_printf.h"
#include "sleep.h"

// Definitions based on the system
#define TIMER_DEVICE_ID      XPAR_TMRCTR_0_DEVICE_ID
#define INTC_DEVICE_ID       XPAR_INTC_0_DEVICE_ID
#define TIMER_INTERRUPT_ID   XPAR_INTC_0_TMRCTR_0_VEC_ID
#define CLOCK_FREQUENCY      100000000  // 100 MHz
#define TIMER1_LOAD_VALUE    100000000  // Load value for 1 second period (CLOCK_FREQUENCY * 1)
#define TIMER2_LOAD_VALUE       100000	// Load value for 1 milisecond period (CLOCK_FREQUENCY * 0.001)

// Register offsets for the interrupt controller
#define XIN_INT0_EDGE_OFFSET  0x008   // Interrupt Edge/Level Select Register

// Declare instances for the interrupt controller
XTmrCtr TimerInstance;
XIntc InterruptController;

void SetInterruptEdgeType();
int initInterruptController();
void stopTimer2();
void startTimer2();

extern void IntPinHandler(void *CallbackRef);
extern void TimerHandler(void *CallBackRef, u8 TmrCtrNumber);

#endif /* SRC_INT_INIT_H_ */
