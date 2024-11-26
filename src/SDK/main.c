/******************************************************************************
 *                                                                            *
 * Program Name: Instrument Panel Cluster Controller                          *
 *                                                                            *
 * Description:                                                               *
 * This program compiles in Xilinx SDK, runs on a Xilinx FPGA development     *
 * board, and controls the LED indications and the dials of an Instrument     *
 * Panel Cluster (IPC). It does so via a CAN-Bus interface to the IPC. The    *
 * FPGA development board also connects via an SPI interface to a Control     *
 * Board to get inputs and display feedback on it.                            *
 *                                                                            *
 * Author: Spiropoulos Vasilis                                                *
 *                                                                            *
 * Date: 2024-06-25                                                           *
 *                                                                            *
 * Version: 1.0                                                               *
 *                                                                            *
 ******************************************************************************/


#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xspi.h"
#include "xstatus.h"
#include "xil_exception.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "xil_printf.h"
#include "sleep.h"
#include "spi_api.h"		// include before ICs header files
#include "gpio_api.h"		// include before ICs header files
#include "max7221.h"		// Led Controller     IC max7221  header file
#include "mcp23s17.h"		// I/O Expander       IC mcp23s17 header file
#include "mcp2515.h"		// CAN-Bus Controller IC mcp2515  header file
#include "stdbool.h"
#include "int_init.h"


// u8 = unsigned char
// u32 = unsigned int

// Delay times for demonstration
#define demoDelayTime1 100
#define demoDelayTime2 600
#define demoDelayTime3 150
#define demoDelayTime4 600
#define demoDelayTime5 200
#define demoDelayTime6 200

// Maximum number of dial leds
#define rpm_max  17
#define fuel_max  9
#define sp_max   25

// Interrupt driven flags
volatile bool flg = false;          // the flag is set when switch is pressed/released or rotary enc rotated
volatile bool wake = false;         // the flag is set every 1 second

// Amount of time in milliseconds passed since Timer2 initialized
u32 millis = 0;

typedef struct {                    // demo_dial_leds function
	int time;
	unsigned char rpm;
	unsigned char fuel;
	unsigned char sp;
} DemoData;


void MCPisCalling();                // Interrupt Service Routine for MCP23S17 port status change
void WakeUp();                      // Timer1 calls WakeUp() every 1 second to set the "wake" flag, in order to send a wake-up message over CAN-Bus

void fill_led_table(bool led_rpm[], bool led_fuel[], bool led_sp[], bool led_sw[], bool *led_sw11_color, unsigned char mx[][8]);
void set_led_table(unsigned char mx[][8]);
void calc_rpm_leds(unsigned char *rpm, bool mode[], bool led_rpm[]);
void calc_fuel_leds(unsigned char *fuel, bool mode[], bool led_fuel[]);
void calc_sp_leds(unsigned char *sp, bool mode[], bool led_sp[]);
void show_leds(unsigned char mx[][8], unsigned char *info_led);
void show_num_rpm(unsigned int rpm_value, unsigned char *info_led);
void show_num_fuel(unsigned int fuel_value, unsigned char *info_led);
void show_num_sp(unsigned int sp_value, unsigned char *info_led);
void set_num_all(unsigned char *info_led);
void clear_leds_on_lcd(bool led_sw[], bool led_sw_old[], unsigned char lights_status[], unsigned char *info_led);    // Not used
void clear_switch_leds(bool led_sw[], bool led_sw_old[], unsigned char lights_status[], unsigned char *info_led);
void clear_dials(unsigned char *rpm, unsigned char *fuel, unsigned char *sp, bool mode[], bool led_rpm[], bool led_fuel[], bool led_sp[], unsigned char *info_led);
void demo_segs(unsigned char demoSeg[], unsigned char rfs[]);
void demo_switch_leds(bool led_sw[], unsigned char demoButtons[], bool led_rpm[], bool led_fuel[], bool led_sp[], bool *led_sw11_color, unsigned char mx[][8], unsigned char *info_led);
void demo_dial_leds(DemoData demo[], bool led_sw[], bool *led_sw11_color, unsigned char mx[][8], unsigned char *info_led);


// Main application setup
int main() {
	init_platform();		// Initialize platform

	gpio_init();			// Initialize AXI GPIO

	spi_init();				// Initialize AXI SPI
	XSpi_IntrGlobalDisable(&SpiInstance);

	initInterruptController();	// Initialize Interrupt Controller

	unsigned char mx[3][8] = {0};   // MAX7221 byte value tables

    // Switch leds & dial leds
	bool led_rpm[rpm_max] = {0};    // RPM dial leds
	bool led_fuel[fuel_max] = {0};  // Fuel dial leds
	bool led_sp[sp_max] = {0};      // Speed dial leds
	bool mode[4] = {0};             // Dial leds mode, 0=Led_bar_mode 1=Dot_led_mode, index 0 not used, (rpm,fuel,speed)=(1,2,3)
	bool led_sw[36] = {0};          // Switch leds, index 0 not used
	bool led_sw_old[36] = {0};      // Switch leds previous state, index 0 not used
	bool led_sw11_color = 0;        // 0 is Yellow, 1 is Red
	bool led_sw11_changed = 0;      // flag to change state: Off -> On Yellow -> On Red -> Off
	bool sw25_on = 0;               // flag to auto reset switch 25 led (Beep switch)

	bool sw[36] = {0};              // Switch state, 0=released 1=pressed, index 0 not used
	bool enc_sw[4] = {0};           // Rotary Enc switch state, 0=released 1=pressed, index 0 not used
	unsigned char enc[4] = {0};     // Rotary Enc state, 0=released 1=pressed, index 0 not used, (enc_1,enc_2,enc_3)=(1,2,3)
	unsigned char thisState[4] = {0}; // bits 1-0 for ind (knobPosition index), index 0 not used
	unsigned char oldState[4] = {0};  // bits 3-2 for ind (knobPosition index), index 0 not used
	const int8_t KNOBDIR[] = {        // Array decisive for completeness of 4 states per Rotary Enc click, CW or CCW
		 0,  1, -1,  0,
		-1,  0,  0,  1,
	     1,  0,  0, -1,
		 0, -1,  1,  0  };

	signed char knobPosition[4] = {0};  // Rotary Enc state during movement, CW: 0 ->  1 ->  2 ->  3 ->  4(=0)    (enc_1,enc_2,enc_3)=(1,2,3)
	int ind;                        // index for knobPosition               CCW: 0 -> -1 -> -2 -> -3 -> -4(=0)    (index 0 not used)
	bool rotary_move[4] = {0};      // Rotary Enc in movement, 1 means the 4 states per click not completed, index 0 is any R.Enc in movement

	// RPM/Fuel/Speed dial leds
	unsigned char rpm = 0;
	unsigned char fuel = 0;
	unsigned char sp = 0;

	unsigned char port_1A;  // IC1 Port A
	unsigned char port_1B;  // IC1 Port A
	unsigned char port_2A;  // IC2 Port A
	unsigned char port_2B;  // IC2 Port B
	unsigned char port_3A;  // IC3 Port A
	unsigned char port_3B;  // IC3 Port B
	unsigned char info_led = 0xF0;    // bit-encoded variable: bits 7:4 represent: "Led", "Switch", "R.Encoder", "Seg.Display" activity. All leds active-low

	unsigned char cnt = 0;            // variable that counts how many times the flag "wake" becomes true (every second)

	// Instrument Panel Cluster calibrated dial values (precise dial indication)
	const unsigned char rpm_dial[rpm_max] = {0, 10, 21, 30, 41, 50, 60, 70, 79, 89, 99, 109, 118, 128, 138, 148, 158};
	const unsigned char fuel_dial[fuel_max] = {0, 29, 61, 90, 122, 153, 183, 214, 245};
	const unsigned char sp_dial[sp_max] = {0, 6, 12, 19, 25, 31, 37, 43, 49, 55, 61, 68, 74, 80, 86, 92, 98, 104, 110, 116, 123, 129, 136, 142, 149};

	// 7-seg led display message data
	unsigned char demoSeg[8] = {0x02, 0x40, 0x20, 0x01, 0x04, 0x08, 0x10, 0x01};                      // snake movement: F, A, B, G, E, D, C, G
	unsigned char rfs[12] = {0x05, 0x67, 0x15, 0x11 ,0x47, 0x3E, 0x4F, 0x0E, 0x5B, 0x67, 0x4F, 0x4F}; // "rPri", "FuEL" and "SPEE" meaning "RPM", "FUEL" and "SPEED"
	unsigned char errMCP[4] = {0x00, 0x4F, 0x05, 0x05};                                               // " Err" meaning "Error"		(not used in SDK)

	DemoData demo[33] = {{   0,  0, 0,  0},     // dial leds demonstration, parallel dial movement
                         {  50,  0, 0,  1},     // (time from demo start, rpm leds, fuel leds, speed leds)
                         {  75,  1, 0,  1},
                         { 100,  1, 0,  2},
                         { 150,  2, 1,  3},
                         { 200,  2, 1,  4},
                         { 225,  3, 1,  4},
                         { 250,  3, 1,  5},
                         { 300,  4, 2,  6},
                         { 350,  4, 2,  7},
                         { 375,  5, 2,  7},
                         { 400,  5, 2,  8},
                         { 450,  6, 3,  9},
                         { 500,  6, 3, 10},
                         { 525,  7, 3, 10},
                         { 550,  7, 3, 11},
                         { 600,  8, 4, 12},
                         { 650,  8, 4, 13},
                         { 675,  9, 4, 13},
                         { 700,  9, 4, 14},
                         { 750, 10, 5, 15},
                         { 800, 10, 5, 16},
                         { 825, 11, 5, 16},
                         { 850, 11, 5, 17},
                         { 900, 12, 6, 18},
                         { 950, 12, 6, 19},
                         { 975, 13, 6, 19},
                         {1000, 13, 6, 20},
                         {1050, 14, 7, 21},
                         {1100, 14, 7, 22},
                         {1125, 15, 7, 22},
                         {1150, 15, 7, 23},
                         {1200, 16, 8, 24}};

	// Order of switch led illumination during demo
	unsigned char demoButtons[35] = {4, 3, 2, 1, 10, 11, 12, 13, 8, 7, 6, 5, 9, 14, 15, 16, 17, 18, 19, 34, 33, 32, 31, 27, 28, 29, 30, 26, 20, 21, 22, 23, 24, 25, 35};

	// CAN-Bus values for switch leds representation on Instrument Panel Cluster (CAN_Bus data byte 3, bytes 4 & 5 bit position mask), index = switch , index 0 not used
	//                    Row :        0         1         2         3         4         5         6         7         8         9    |   Column
	unsigned char lights[36][2] = {{ 0, 0 }, { 1, 1 }, { 1, 2 }, { 1, 0 }, { 1, 7 }, { 4, 7 }, { 1, 3 }, { 1, 4 }, { 5, 0 }, { 1, 5 },  // + 00
                                   { 4, 3 }, { 2, 4 }, { 4, 2 }, { 4, 4 }, { 2, 7 }, { 3, 4 }, { 3, 7 }, { 3, 1 }, { 4, 0 }, { 3, 5 },  // + 10
                                   { 2, 1 }, { 2, 2 }, { 0, 0 }, { 4, 5 }, { 0, 0 }, { 0, 0 }, { 3, 0 }, { 3, 2 }, { 2, 6 }, { 2, 5 },  // + 20
                                   { 3, 3 }, { 3, 6 }, { 1, 6 }, { 2, 0 }, { 4, 1 }, { 0, 0 }};                                         // + 30

	unsigned char lights_status[6] = {0};   // Keeps track of the state of the switch leds, index = CAN_Bus data byte 3  , index 0 not used

	// MCP23S17 Reset and Initialization
	mcp_reset();
	initMCP23S17();

	// MAX7221 Reset and Initialization
	initMAX7221(MAX_1);
	initMAX7221(MAX_2);
	initMAX7221(MAX_3);
	usleep(300000);	// delay 300 msec


	// set the intensity of the leds (range: 1 to 15)
	setIntensity(MAX_1, 8); // Intensity ranges from 1 to 15
	setIntensity(MAX_2, 8); // Intensity ranges from 1 to 15
	setIntensity(MAX_3, 8); // Intensity ranges from 1 to 15
	usleep(300000);	// delay 300 msec



  // Demonstration
	demo_segs(demoSeg, rfs);  // 7-seg snake demonstration
	demo_switch_leds(led_sw, demoButtons, led_rpm, led_fuel, led_sp, &led_sw11_color, mx, &info_led); // switch leds demonstration

	// define which digits use the internal segments decoder
	setDecode(MAX_1, 0xF0);
	setDecode(MAX_2, 0xF0);
	setDecode(MAX_3, 0xF0);

	// Demonstration
	demo_dial_leds(demo, led_sw, &led_sw11_color, mx, &info_led); // dial leds demonstration

	stopTimer2();		// Stop Timer 2 interrupts (every 1 msec), needed only for demo_dial_leds()

	// Display initial switch leds and dial leds
	calc_rpm_leds(&rpm, mode, led_rpm);
	calc_fuel_leds(&fuel, mode, led_fuel);
	calc_sp_leds(&sp, mode, led_sp);
	fill_led_table(led_rpm, led_fuel, led_sp, led_sw, &led_sw11_color, mx);
	show_leds(mx, &info_led);


	// MCP2515 Reset and Initialization
	resetMCP2515();
	setBitrateCan();				// Set Configuration
	setNormalModeCan();			// Set Normal mode (not Sleep/Loopback/Listen-Only/Configuration mode)
	RegularOperationMode();	// Set Regular mode (not One-Shot mode)


	// Main application loop
	while(1) {
		// No switch pressed(/released), nor any rotary enc rotated, or less than 4 secs passed (cnt < 4)
		while((flg == false) && (cnt < 4)){  // flag flg is true when a switch is pressed or a rotary encoder is rotated
			if (wake == true){                  // flag wake is true every 1 second
				if (sw25_on == 1){                // switch S25 pressed in previous session
					sw25_on = 0;
					led_sw[25] = 0;                 // auto turn off switch led S25
				}
				cnt += 1;                         // Increase 2-second flag counter
				info_led = 0xF0;                  // Reset all 4 info leds to off state
				CAN_FRAME frame_wake;             // Send out the GMLAN wake-up message on whichever mailbox is free or queue it for sending when there is an opening. CAN-Bus Wake up message
				frame_wake.id = 0x632;            // or queue it for sending when there is an opening
				frame_wake.length = 8;
				frame_wake.data.low  = 0x00504800;
				frame_wake.data.high = 0x00000000;
				sendCANMessage(&frame_wake);
				wake = false;
			}
		}

		// A switch pressed(/released) or a rotary enc rotated, or 4 secs passed (cnt is 4)
		flg = false;                          // Reset key-pressed flag

		// Query the ports A & B status, for IC1, IC2 & IC3
		port_1A = mcp_getPort(MAX_1, 'A'); // query the complete port status
		port_1B = mcp_getPort(MAX_1, 'B'); // query the complete port status
		port_2A = mcp_getPort(MAX_2, 'A'); // query the complete port status
		port_2B = mcp_getPort(MAX_2, 'B'); // query the complete port status
		port_3A = mcp_getPort(MAX_3, 'A'); // query the complete port status
		port_3B = mcp_getPort(MAX_3, 'B'); // query the (complete) port status [only (3:0)]

		// Calculate switch & rotary enc states
		sw[1] = port_1A & 0x01;   // IC1 Port A_0
		sw[2] = port_1A & 0x02;   // IC1 Port A_1
		sw[3] = port_1A & 0x04;   // IC1 Port A_2
		sw[4] = port_1A & 0x08;   // IC1 Port A_3
		sw[5] = port_1A & 0x10;   // IC1 Port A_4
		sw[6] = port_1A & 0x20;   // IC1 Port A_5
		sw[7] = port_1A & 0x40;   // IC1 Port A_6
		sw[8] = port_1A & 0x80;   // IC1 Port A_7

		sw[9] = port_1B & 0x01;  // IC1 Port B_0
		sw[10] = port_1B & 0x10;  // IC1 Port B_4
		sw[11] = port_1B & 0x20;  // IC1 Port B_5
		sw[12] = port_1B & 0x40;  // IC1 Port B_6
		sw[13] = port_1B & 0x80;  // IC1 Port B_7

		sw[14] = port_2A & 0x01;  // IC2 Port A_0
		sw[15] = port_2A & 0x02;  // IC2 Port A_1
		sw[16] = port_2A & 0x04;  // IC2 Port A_2
		sw[17] = port_2A & 0x40;  // IC2 Port A_6
		sw[18] = port_2A & 0x80;  // IC2 Port A_7

		sw[19] = port_2B & 0x01;  // IC2 Port B_0
		sw[20] = port_2B & 0x02;  // IC2 Port B_1
		sw[21] = port_2B & 0x04;  // IC2 Port B_2
		sw[22] = port_2B & 0x08;  // IC2 Port B_3
		sw[23] = port_2B & 0x10;  // IC2 Port B_4
		sw[24] = port_2B & 0x20;  // IC2 Port B_5
		sw[25] = port_2B & 0x40;  // IC2 Port B_6

		sw[26] = port_3A & 0x08;  // IC3 Port A_3
		sw[27] = port_3A & 0x10;  // IC3 Port A_4
		sw[28] = port_3A & 0x20;  // IC3 Port A_5
		sw[29] = port_3A & 0x40;  // IC3 Port A_6
		sw[30] = port_3A & 0x80;  // IC3 Port A_7

		sw[31] = port_3B & 0x01;  // IC3 Port B_0
		sw[32] = port_3B & 0x02;  // IC3 Port B_1
		sw[33] = port_3B & 0x04;  // IC3 Port B_2
		sw[34] = port_3B & 0x08;  // IC3 Port B_3

		sw[35] = port_2B & 0x80;  // IC2 Port B_7

		enc_sw[1] = port_1B & 0x08; // IC1 Port B_3
		enc_sw[2] = port_2A & 0x20; // IC2 Port A_5
		enc_sw[3] = port_3A & 0x04; // IC3 Port A_2

		enc[1] = (port_1B & 0x06) >> 1; // IC1 Port B_2,1
		enc[2] = (port_2A & 0x18) >> 3; // IC2 Port A_4,3
		enc[3] =  port_3A & 0x03;       // IC3 Port A_1,0

		// Any rotary enc moved
		if (enc[1] || enc[2] || enc[3] || rotary_move[0]){
			info_led = info_led & 0xD0;   // clear bit 5 (R.Encoder)
			if (enc[1] || enc[2] || enc[3]){      // Set rotary_move(1, 2 or 3] if corresponding enc[] is set, but not clear it when corresponding enc[] is clear
				rotary_move[1] = enc[1];
				rotary_move[2] = enc[2];
				rotary_move[3] = enc[3];
			}

			// Template CAN-Bus "dials" message
			CAN_FRAME frame_rotary;
			frame_rotary.id = 0x255;
			frame_rotary.length = 8;
			frame_rotary.data.low  = 0x0100AE04;
			frame_rotary.data.high = 0x00000000;

			// Rotary Encoders 1,2 & 3 states
			for (int enc_num=1; enc_num<4; enc_num++){
				oldState[enc_num] = thisState[enc_num];
				thisState[enc_num] = enc[enc_num];
				ind = thisState[enc_num] | (oldState[enc_num] << 2);
				knobPosition[enc_num] += KNOBDIR[ind];
				if (knobPosition[enc_num] == 4){    // This rotary encoder completed 4 states (1 click) CW
					knobPosition[enc_num] = 0;
					rotary_move[enc_num] = 0; // Clear rotary_move(1, 2 or 3) if corresponding thisState[] = 0 and oldState[] was not 0 . Sequence of 4 states is complete, for this rotary encoder click
					switch (enc_num){
					case 1:
						if (rpm<16){          // Increase RPM leds
							rpm += 1;
							calc_rpm_leds(&rpm, mode, led_rpm);
							show_num_rpm((unsigned int)(rpm) * 500, &info_led);
							frame_rotary.data.byte[2] = 0x09;
							frame_rotary.data.byte[4] = rpm_dial[rpm];
							sendCANMessage(&frame_rotary);
						}
						break;
					case 2:
						if (fuel<8){          // Increase fuel leds
							fuel += 1;
							calc_fuel_leds(&fuel, mode, led_fuel);
							show_num_fuel((unsigned int)(fuel) * 6, &info_led);
							frame_rotary.data.byte[2] = 0x0A;
							frame_rotary.data.byte[4] = fuel_dial[fuel];
							sendCANMessage(&frame_rotary);
						}
						break;
					default:
						if (sp<24){          // Increase speed leds
							sp += 1;
							calc_sp_leds(&sp, mode, led_sp);
							show_num_sp((unsigned int)(sp) * 10, &info_led);
							frame_rotary.data.byte[2] = 0x08;
							frame_rotary.data.byte[4] = sp_dial[sp];
							sendCANMessage(&frame_rotary);
						}
						break;
					}
				}

				if (knobPosition[enc_num] == -4){    // This rotary encoder completed 4 states (1 click) CCW
					knobPosition[enc_num] = 0;
					rotary_move[enc_num] = 0;
					switch (enc_num){
					case 1:
						if (rpm>0){          // Decrease RPM leds
							rpm -= 1;
							calc_rpm_leds(&rpm, mode, led_rpm);
							show_num_rpm((unsigned int)(rpm) * 500, &info_led);
							frame_rotary.data.byte[2] = 0x09;
							frame_rotary.data.byte[4] = rpm_dial[rpm];
							sendCANMessage(&frame_rotary);
						}
						break;
					case 2:
						if (fuel>0){          // Decrease fuel leds
							fuel -= 1;
							calc_fuel_leds(&fuel, mode, led_fuel);
							show_num_fuel((unsigned int)(fuel) * 6, &info_led);
							frame_rotary.data.byte[2] = 0x0A;
							frame_rotary.data.byte[4] = fuel_dial[fuel];
							sendCANMessage(&frame_rotary);
						}
						break;
					default:
						if (sp>0){          // Decrease speed leds
							sp -= 1;
							calc_sp_leds(&sp, mode, led_sp);
							show_num_sp((unsigned int)(sp) * 10, &info_led);
							frame_rotary.data.byte[2] = 0x08;
							frame_rotary.data.byte[4] = sp_dial[sp];
							sendCANMessage(&frame_rotary);
						}
						break;
					}
				}
				rotary_move[0] = rotary_move[1] | rotary_move[2] | rotary_move[3];
			}   // end "Rotary Encoders 1,2 & 3 states"
		}     // end "Any rotary enc moved"

		// Template CAN-Bus "dials" message
		CAN_FRAME frame_switch;
		frame_switch.id = 0x255;
		frame_switch.length = 8;
		frame_switch.data.low  = 0x0000AE04;
		frame_switch.data.high = 0x00000000;

		// Update switch actions
		for (int sw_i = 1; sw_i<36; sw_i++){
			// Update switch leds
			if (sw_i == 11){
				if (sw[11]){              // Switch S11 pressed,
					led_sw11_changed = 1;   // loop through Off -> Yellow -> Red -> Off
					if (led_sw[11]){
						if (led_sw11_color){
							led_sw11_color = 0;
							led_sw[11] = 0;
						}
						else{
							led_sw11_color = 1;
						}
					}
					else{
						led_sw[11] = 1;
					}
				}
			}
			else {                                    // sw_i not equals 11
				led_sw[sw_i] = led_sw[sw_i] ^ sw[sw_i]; // change led_sw state
			}

			if ((led_sw[sw_i] ^ led_sw_old[sw_i]) && (sw_i != 11)) {     // Switch led (other than S11) changed
				info_led = info_led & 0xB0;   // clear bit 6 (Switch)
				cnt = 0;

				// Special switch functions: S22, S24, S25, S35
				switch (sw_i) {
				case 22: {            // Odometer LCD Test
					CAN_FRAME frame_odo;
					frame_odo.id = 0x255;        // OFF to Odometer LCD Test
					frame_odo.length = 8;
					frame_odo.data.low  = 0x010DAE04;
					frame_odo.data.high = 0x00000001;

					if (led_sw[22]){    // ON
						frame_odo.data.byte[3] = 0x01;
						frame_odo.data.byte[4] = 0x01;
					}
					else{               // OFF
						frame_odo.data.byte[3] = 0x00;
						frame_odo.data.byte[4] = 0x00;
					}
					sendCANMessage(&frame_odo);
					break;}
				case 24:              // Emergency Lights
					frame_switch.id = 0x260;
					frame_switch.length = 3;
					if (led_sw[24]){    // ON
						clear_switch_leds(led_sw, led_sw_old, lights_status, &info_led);
						clear_dials(&rpm, &fuel, &sp, mode, led_rpm, led_fuel, led_sp, &info_led);
						CAN_FRAME frame_odo;
						frame_odo.id = 0x255;        // turn off "Odometer LCD Test"
						frame_odo.length = 8;
						frame_odo.data.low  = 0x000DAE04;
						frame_odo.data.high = 0x00000000;
						sendCANMessage(&frame_odo);
						usleep(30000);

						frame_switch.data.low  = 0x0080327F;
						frame_switch.data.high = 0x00000000;
					}
					else{               // OFF
						frame_switch.data.low  = 0x00000000;
						frame_switch.data.high = 0x00000000;
					}
					sendCANMessage(&frame_switch);
					break;
				case 25:              // Beep Sound (auto switch led to off)
					if (led_sw[25]){
						if (led_sw[22]){
							CAN_FRAME frame_odo;
							frame_odo.id = 0x255;        // turn off "Odometer LCD Test"
							frame_odo.length = 8;
							frame_odo.data.low  = 0x000DAE04;
							frame_odo.data.high = 0x00000000;
							sendCANMessage(&frame_odo);
							usleep(30000);
						}

						clear_switch_leds(led_sw, led_sw_old, lights_status, &info_led);
						clear_dials(&rpm, &fuel, &sp, mode, led_rpm, led_fuel, led_sp, &info_led);
						sw25_on = 1;

						frame_switch.id = 0x281;  // ON to Beep
						frame_switch.length = 5;
						frame_switch.data.low  = 0x021E0560;
						frame_switch.data.high = 0x00000033;
						sendCANMessage(&frame_switch);
					}
					break;
				case 35:              // All switch leds ON test (2 seconds)
					if (led_sw[35]){
						for (unsigned char i=1; i<36; i++){
							led_sw[i] = 1;
						}
						if (led_sw[22]){
							CAN_FRAME frame_odo;
							frame_odo.id = 0x255;        // turn off "Odometer LCD Test"
							frame_odo.length = 8;
							frame_odo.data.low  = 0x000DAE04;
							frame_odo.data.high = 0x00000000;
							sendCANMessage(&frame_odo);
							usleep(30000);
						}

						if (led_sw[24]){
							frame_switch.id = 0x260;    // turn off "Emergency Lights"
							frame_switch.length = 3;
							frame_switch.data.low  = 0x00000000;
							frame_switch.data.high = 0x00000000;
							sendCANMessage(&frame_switch);
						}

						set_led_table(mx);
						show_leds(mx, &info_led);
						set_num_all(&info_led);

						sleep(2);
						for (unsigned char i=1; i<36; i++){
							led_sw[i] = 0;
							led_sw_old[i] = 0;
						}

						clear_switch_leds(led_sw, led_sw_old, lights_status, &info_led);
						clear_dials(&rpm, &fuel, &sp, mode, led_rpm, led_fuel, led_sp, &info_led);
					}
					break;
				default:      // Update switch leds status to Instrument Panel Cluster
					lights_status[lights[sw_i][0]] = lights_status[lights[sw_i][0]] ^ (0x01 << lights[sw_i][1]);

					frame_switch.data.byte[2] = lights[sw_i][0];
					frame_switch.data.byte[3] = lights_status[lights[sw_i][0]];
					frame_switch.data.byte[4] = lights_status[lights[sw_i][0]];
					sendCANMessage(&frame_switch);
					break;
				}

			}
			else if ((sw_i == 11) && (led_sw11_changed == 1)){         // Switch led S11 changed
				info_led = info_led & 0xB0;   // clear bit 6 (Switch)
				cnt = 0;
				led_sw11_changed = 0;

				// Update switch led status for S11 to Instrument Panel Cluster
				if (led_sw[11]){
					if (led_sw11_color){
						lights_status[lights[sw_i][0]] = lights_status[lights[sw_i][0]] | 0x10;
						lights_status[lights[sw_i][0]] = lights_status[lights[sw_i][0]] & 0xF7;
					}
					else{
						lights_status[lights[sw_i][0]] = lights_status[lights[sw_i][0]] | 0x08;
						lights_status[lights[sw_i][0]] = lights_status[lights[sw_i][0]] & 0xEF;
					}
				}
				else{
					lights_status[lights[sw_i][0]] = lights_status[lights[sw_i][0]] & 0xE7;
				}

				frame_switch.data.byte[2] = lights[sw_i][0];
				frame_switch.data.byte[3] = lights_status[lights[sw_i][0]];
				frame_switch.data.byte[4] = lights_status[lights[sw_i][0]];
				sendCANMessage(&frame_switch);
			}
			else if (cnt > 3){                            // no switch led changed and 4 seconds passed:
				frame_switch.data.byte[2] = lights[1][0];   // switch led status to IPC must be updated every 4 seconds
				frame_switch.data.byte[3] = lights_status[lights[1][0]];
				frame_switch.data.byte[4] = lights_status[lights[1][0]];
				sendCANMessage(&frame_switch);
				usleep(30000);

				frame_switch.data.byte[2] = lights[2][0];
				frame_switch.data.byte[3] = lights_status[lights[2][0]];
				frame_switch.data.byte[4] = lights_status[lights[2][0]];
				sendCANMessage(&frame_switch);
				usleep(30000);

				frame_switch.data.byte[2] = lights[3][0];
				frame_switch.data.byte[3] = lights_status[lights[3][0]];
				frame_switch.data.byte[4] = lights_status[lights[3][0]];
				sendCANMessage(&frame_switch);
				usleep(30000);

				frame_switch.data.byte[2] = lights[4][0];
				frame_switch.data.byte[3] = lights_status[lights[4][0]];
				frame_switch.data.byte[4] = lights_status[lights[4][0]];
				sendCANMessage(&frame_switch);
				usleep(30000);

				frame_switch.data.byte[2] = lights[5][0];
				frame_switch.data.byte[3] = lights_status[lights[5][0]];
				frame_switch.data.byte[4] = lights_status[lights[5][0]];
				sendCANMessage(&frame_switch);

				cnt = 0;
			}

			led_sw_old[sw_i] = led_sw[sw_i];
		}   // end "Update switch actions"

		// Any rotary enc switch pressed
		if (enc_sw[1] || enc_sw[2] || enc_sw[3]){
			if (enc_sw[1]){
				mode[1] = !mode[1];
				calc_rpm_leds(&rpm, mode, led_rpm);
			}
			if (enc_sw[2]){
				mode[2] = !mode[2];
				calc_fuel_leds(&fuel, mode, led_fuel);
			}
			if (enc_sw[3]){
				mode[3] = !mode[3];
				calc_sp_leds(&sp, mode, led_sp);
			}
		}   // end "Any rotary enc switch pressed"


		mcp_setPort(2, 'B', info_led);

		// Update switch leds and dial leds
		fill_led_table(led_rpm, led_fuel, led_sp, led_sw, &led_sw11_color, mx);
		show_leds(mx, &info_led);

	}    // end while(1){

	cleanup_platform();	// extra
	return 0;
}   // end main(){




// External Pin ISR
void IntPinHandler(void *CallbackRef) {
	// Clear the interrupt
	XIntc_AckIntr(XPAR_INTC_0_BASEADDR, XPAR_SYSTEM_MCP_INT_MASK);

	MCPisCalling();
}


// Timer ISR
void TimerHandler(void *CallBackRef, u8 TmrCtrNumber) {
	XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

	if (TmrCtrNumber == 0) {
		// Timer 1 ISR actions
		XTmrCtr_Stop(InstancePtr, 0); // Acknowledge Timer 1 interrupt
		XTmrCtr_Start(InstancePtr, 0);
		WakeUp();
	} else if (TmrCtrNumber == 1) {
		// Timer 2 ISR actions
		XTmrCtr_Stop(InstancePtr, 1); // Acknowledge Timer 2 interrupt
		XTmrCtr_Start(InstancePtr, 1);
		millis++;
	}
}



// Interrupt Service Routine for MCP23S17 port status change. ISR is called when INT goes high
void MCPisCalling() {
	flg = true;
}


// Timer1 Interrupt Service Routine. ISR is called every every 1 second to set the "wake" flag, in order to send a wake-up message over CAN-Bus
void WakeUp() {
	wake = true;
}



/**
 * @brief Calculates MAX7221 led controller register values,
 *        that are connected to switch leds and dial leds,
 *        from their bit values.
 *
 * @param led_rpm RPM dial leds 0 to 16 boolean values.
 * @param led_fuel Fuel dial leds 0 to 8 boolean values.
 * @param led_sp Speed dial leds 0 to 24 boolean values.
 * @param led_sw Switch leds 1 to 35 boolean values. Switch 11 is bi-color
 * @param led_sw11_color Switch 11 red/yellow led boolean value. True is red.
 * @param mx A 2D array of unsigned characters where the MAX7221
 *           led controller register values will be updated.
 *           The dimensions of the array are 3 x 8.
 *
 * @note For all led boolean values: true is led on, false is led off.
 */

void fill_led_table(bool led_rpm[], bool led_fuel[], bool led_sp[], bool led_sw[], bool *led_sw11_color, unsigned char mx[][8]){
	bool led_sw11_R, led_sw11_Y;  // Switch 11 red & yellow on/off status
	led_sw11_R = led_sw[11] & (*led_sw11_color);
	led_sw11_Y = led_sw[11] & (!(*led_sw11_color));

	mx[0][0] =   led_sw[1] | (  led_sw[2] << 1) | (  led_sw[3] << 2) | (  led_sw[4] << 3) | (  led_sw[5] << 4) | (  led_sw[6] << 5) | (  led_sw[7] << 6) | (  led_sw[8] << 7);  // mx[0][0]
	mx[0][1] =   led_sw[9] | ( led_sw[10] << 1) | ( led_sw11_R << 2) | ( led_sw11_Y << 3) | ( led_sw[12] << 4) | ( led_sw[13] << 5) | ( led_sw[14] << 6) | ( led_rpm[0] << 7);  // mx[0][1]
	mx[0][2] =  led_rpm[1] | ( led_rpm[2] << 1) | ( led_rpm[3] << 2) | ( led_rpm[4] << 3) | ( led_rpm[5] << 4) | ( led_rpm[6] << 5) | ( led_rpm[7] << 6) | ( led_rpm[8] << 7);  // mx[0][2]
	mx[0][3] =  led_rpm[9] | (led_rpm[10] << 1) | (led_rpm[11] << 2) | (led_rpm[12] << 3) | (led_rpm[13] << 4) | (led_rpm[14] << 5) | (led_rpm[15] << 6) | (led_rpm[16] << 7);  // mx[0][3]

	mx[1][1] =  led_sw[23] | ( led_sw[22] << 1) | ( led_sw[21] << 2) | ( led_sw[20] << 3) | ( led_sw[24] << 4) | ( led_sw[25] << 5) | ( led_sw[35] << 6) | (  led_sp[0] << 7);  // mx[1][1]
	mx[1][2] =  led_sw[15] | ( led_sw[16] << 1) | ( led_sw[17] << 2) | ( led_sw[18] << 3) | ( led_sw[19] << 4) | ( led_sw[26] << 5)                      | (led_fuel[0] << 7);  // mx[1][2] , bit 6 is not used
	mx[1][3] = led_fuel[1] | (led_fuel[2] << 1) | (led_fuel[3] << 2) | (led_fuel[4] << 3) | (led_fuel[5] << 4) | (led_fuel[6] << 5) | (led_fuel[7] << 6) | (led_fuel[8] << 7);  // mx[1][3]

	mx[2][0] =  led_sw[27] | ( led_sw[28] << 1) | ( led_sw[29] << 2) | ( led_sw[30] << 3) | ( led_sw[31] << 4) | ( led_sw[32] << 5) | ( led_sw[33] << 6) | ( led_sw[34] << 7);  // mx[2][0]
	mx[2][1] =   led_sp[1] | (  led_sp[2] << 1) | (  led_sp[3] << 2) | (  led_sp[4] << 3) | (  led_sp[5] << 4) | (  led_sp[6] << 5) | (  led_sp[7] << 6) | (  led_sp[8] << 7);  // mx[2][1]
	mx[2][2] =   led_sp[9] | ( led_sp[10] << 1) | ( led_sp[11] << 2) | ( led_sp[12] << 3) | ( led_sp[13] << 4) | ( led_sp[14] << 5) | ( led_sp[15] << 6) | ( led_sp[16] << 7);  // mx[2][2]
	mx[2][3] =  led_sp[17] | ( led_sp[18] << 1) | ( led_sp[19] << 2) | ( led_sp[20] << 3) | ( led_sp[21] << 4) | ( led_sp[22] << 5) | ( led_sp[23] << 6) | ( led_sp[24] << 7);  // mx[2][3]
}



/**
 * @brief Sets MAX7221 led controller register values,
 *        that are connected to switch leds and dial leds (RPM, fuel & speed).
 *
 * @param mx A 2D array of unsigned characters where the MAX7221
 *           led controller register values will be updated.
 *           The dimensions of the array are 3 x 8.
 *
 * @note For all led boolean values: true is led on.
 */

void set_led_table(unsigned char mx[][8]){
	mx[0][0] =   0xFF;  // mx[0][0]
	mx[0][1] =   0xF7;  // mx[0][1]  S11 Yellow led (bit 3) is off, S11 Red led (bit 2) is on
	mx[0][2] =   0xFF;  // mx[0][2]
	mx[0][3] =   0xFF;  // mx[0][3]

	mx[1][1] =   0xFF;  // mx[1][1]
	mx[1][2] =   0xBF;  // mx[1][2]  bit 6 is not used
	mx[1][3] =   0xFF;  // mx[1][3]

	mx[2][0] =   0xFF;  // mx[2][0]
	mx[2][1] =   0xFF;  // mx[2][1]
	mx[2][2] =   0xFF;  // mx[2][2]
	mx[2][3] =   0xFF;  // mx[2][3]
}



/**
 * @brief Calculates the RPM dial leds 0 to 16 boolean values,
 *        that are connected to RPM dial leds,
 *        in relation to the RPM dial leds selected mode of appearance.
 *        mode = 0 : the leds from 0 to rpm (led count) are on, rest are off
 *        mode = 1 : only led @ rpm position is on
 *
 * @param rpm Number of RPM dial leds that are on, starting
 *            from led @ position 1. The led @ position 0 is
 *            always on when its mode = 0.
 * @param mode Array of bolean values for RPM/fuel/sp dial leds selected mode
 *             of appearance.
 * @param led_rpm A boolean array where the RPM dial leds will be updated.
 *
 * @note For all led boolean values: true is led on, false is led off.
 */

void calc_rpm_leds(unsigned char *rpm, bool mode[], bool led_rpm[]){
	for (unsigned char i=0; i<(*rpm); i++){
		led_rpm[i] = !mode[1];
	}
	led_rpm[(*rpm)] = 1;
	for (unsigned char i=(*rpm) + 1; i<rpm_max; i++){
		led_rpm[i] = 0;
	}
}



/**
 * @brief Calculates the fuel dial leds 0 to 8 boolean values,
 *        that are connected to fuel dial leds,
 *        in relation to the fuel dial leds selected mode of appearance.
 *        mode = 0 : the leds from 0 to fuel (led count) are on, rest are off
 *        mode = 1 : only led @ fuel position is on
 *
 * @param fuel Number of fuel dial leds that are on, starting
 *             from led @ position 1. The led @ position 0 is
 *             always on when its mode = 0.
 * @param mode Array of bolean values for RPM/fuel/sp dial leds selected mode
 *             of appearance.
 * @param led_fuel A boolean array where the fuel dial leds will be updated.
 *
 * @note For all led boolean values: true is led on, false is led off.
 */

void calc_fuel_leds(unsigned char *fuel, bool mode[], bool led_fuel[]){
	for (unsigned char i=0; i<(*fuel); i++){
		led_fuel[i] = !mode[2];
	}
	led_fuel[(*fuel)] = 1;
	for (unsigned char i=(*fuel) + 1; i<fuel_max; i++){
		led_fuel[i] = 0;
	}
}



/**
 * @brief Calculates the speed dial leds 0 to 24 boolean values,
 *        that are connected to speed dial leds,
 *        in relation to the speed dial leds selected mode of appearance.
 *        mode = 0 : the leds from 0 to speed (led count) are on, rest are off
 *        mode = 1 : only led @ speed position is on
 *
 * @param sp Number of speed dial leds that are on, starting
 *            from led @ position 1. The led @ position 0 is
 *            always on when its mode = 0.
 * @param mode Array of bolean values for RPM/fuel/sp dial leds selected mode
 *             of appearance.
 * @param led_sp A boolean array where where the speed dial leds will be updated.
 *
 * @note For all led boolean values: true is led on, false is led off.
 */

void calc_sp_leds(unsigned char *sp, bool mode[], bool led_sp[]){
	for (unsigned char i=0; i<(*sp); i++){
		led_sp[i] = !mode[3];
	}
	led_sp[(*sp)] = 1;
	for (unsigned char i=(*sp) + 1; i<sp_max; i++){
		led_sp[i] = 0;
	}
}



/**
 * @brief Sends to MAX7221 led controller, via SPI, the register values,
 *        to turn on/off the connected switch leds and dial leds.
 *
 * @param mx A 2D array of unsigned characters and dimensions of 3 x 8 .
 *           A part of this array stores information about the on/off state
 *           of the connected switch leds and dial leds.
 *
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 7 is connected to the "Led" led.
 *                 When on, it indicates any switch led / dial led update.
 *
 * @note For all led boolean values: true is led on, false is led off.
 *       The info leds are active low: true is led off, false is led on.
 */

void show_leds(unsigned char mx[][8], unsigned char *info_led){
	setRow(MAX_1,0,mx[0][0]);  //icNumber=0 & row=0 receives mx[0][0]
	setRow(MAX_1,1,mx[0][1]);  //icNumber=0 & row=1 receives mx[0][1]
	setRow(MAX_1,2,mx[0][2]);  //icNumber=0 & row=2 receives mx[0][2]
	setRow(MAX_1,3,mx[0][3]);  //icNumber=0 & row=3 receives mx[0][3]
	// rows 4, 5, 6, 7 are 7-seg displays

	// row 0 is now used.
	setRow(MAX_2,1,mx[1][1]);  //icNumber=1 & row=1 receives mx[1][1]
	setRow(MAX_2,2,mx[1][2]);  //icNumber=1 & row=2 receives mx[1][2]
	setRow(MAX_2,3,mx[1][3]);  //icNumber=1 & row=3 receives mx[1][3]
	// rows 4, 5, 6, 7 are 7-seg displays

	setRow(MAX_3,0,mx[2][0]);  //icNumber=2 & row=0 receives mx[2][0]
	setRow(MAX_3,1,mx[2][1]);  //icNumber=2 & row=1 receives mx[2][1]
	setRow(MAX_3,2,mx[2][2]);  //icNumber=2 & row=2 receives mx[2][2]
	setRow(MAX_3,3,mx[2][3]);  //icNumber=2 & row=3 receives mx[2][3]
	// rows 4, 5, 6, 7 are 7-seg displays

	*info_led = (*info_led) & 0x70;   // clear bit 7 to turn on "Led"
}



/**
 * @brief Calculates thousands, hundrends, tens & units from the RPM value
 *        rpm_value and sends it to MAX7221 led controller, via SPI, to be
 *        displayed on RPM 7-seg led display, in format XXXX .
 *
 * @param rpm_value An unsigned int value, which stores the RPM decimal
 *                  number that is dial indicated on the Instrument Panel
 *                  Cluster.
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 4 is connected to the "Seg.Display" led.
 *                 When on, it indicates any 7-seg led display update.
 *
 * @note For all 7-seg led display boolean values: true is led on, false
 *       is led off.
 *       The info leds are active low: true is led off, false is led on.
 *       Code B decode must be turned on for digits 7-4,
 *       that are connected to the 7-seg led displays.
 *       Variable bit order  :  7 6 5 4 3 2 1 0  (msb to lsb)
 *       Led display segment : DP A B C D E F G
 */

void show_num_rpm(unsigned int rpm_value, unsigned char *info_led){
	unsigned char rpm_th, rpm_hun, rpm_ten, rpm_un;

	rpm_th = rpm_value /1000;
	rpm_value = rpm_value - (rpm_th * 1000);
	rpm_hun = rpm_value /100;
	rpm_value = rpm_value - (rpm_hun * 100);
	rpm_ten = rpm_value /10;
	rpm_value = rpm_value - (rpm_ten * 10);
	rpm_un = rpm_value % 10;

	setNum(MAX_1, 0x07, rpm_th);
	setNum(MAX_1, 0x06, rpm_hun);
	setNum(MAX_1, 0x05, rpm_ten);
	setNum(MAX_1, 0x04, rpm_un);

	*info_led = (*info_led) & 0xE0;   // clear bit 4 to turn on "Seg.Display"
}



/**
 * @brief Calculates tens & units from the fuel value
 *        fuel_value and sends it to MAX7221 led controller, via SPI, to be
 *        displayed on fuel 7-seg led display, in format XX.00 .
 *
 * @param fuel_value An unsigned int value, which stores the fuel decimal
 *                   number (& 2 dummy float digits) that is dial indicated
 *                   on the Instrument Panel Cluster.
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 4 is connected to the "Seg.Display" led.
 *                 When on, it indicates any 7-seg led display update.
 *
 * @note For all 7-seg led display boolean values: true is led on, false
 *       is led off.
 *       The info leds are active low: true is led off, false is led on.
 *       Code B decode must be turned on for digits 7-4,
 *       that are connected to the 7-seg led displays.
 *       Variable bit order  :  7 6 5 4 3 2 1 0  (msb to lsb)
 *       Led display segment : DP A B C D E F G
 */

void show_num_fuel(unsigned int fuel_value, unsigned char *info_led){
	unsigned char fuel_ten, fuel_un, fuel_dec1, fuel_dec2;

	fuel_ten = fuel_value / 10;
	fuel_un = 128 + (fuel_value % 10); // 128 is for Decimal Point
	fuel_dec1 = 0;
	fuel_dec2 = 0;

	setNum(MAX_2, 0x07, fuel_ten);
	setNum(MAX_2, 0x06, fuel_un);
	setNum(MAX_2, 0x05, fuel_dec1);
	setNum(MAX_2, 0x04, fuel_dec2);

	*info_led = (*info_led) & 0xE0;   // clear bit 4 to turn on "Seg.Display"
}



/**
 * @brief Calculates thousands, hundrends, tens & units from the speed value
 *        sp_value and sends it to MAX7221 led controller, via SPI, to be
 *        displayed on speed 7-seg led display, in format XXXX .
 *
 * @param sp_value An unsigned int value, which stores the speed
 *                 decimal number that is dial indicated on the Instrument
 *                 Panel Cluster.
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 4 is connected to the "Seg.Display" led.
 *                 When on, it indicates any 7-seg led display update.
 *
 * @note For all 7-seg led display boolean values: true is led on, false
 *       is led off.
 *       The info leds are active low: true is led off, false is led on.
 *       Code B decode must be turned on for digits 7-4,
 *       that are connected to the 7-seg led displays.
 *       Variable bit order  :  7 6 5 4 3 2 1 0  (msb to lsb)
 *       Led display segment : DP A B C D E F G
 */

void show_num_sp(unsigned int sp_value, unsigned char *info_led){
	unsigned char sp_th, sp_hun, sp_ten, sp_un;

	sp_th = sp_value /1000;
	sp_value = sp_value - (sp_th * 1000);
	sp_hun = sp_value /100;
	sp_value = sp_value - (sp_hun * 100);
	sp_ten = sp_value /10;
	sp_value = sp_value - (sp_ten * 10);
	sp_un = sp_value % 10;

	setNum(MAX_3, 0x07, sp_th);
	setNum(MAX_3, 0x06, sp_hun);
	setNum(MAX_3, 0x05, sp_ten);
	setNum(MAX_3, 0x04, sp_un);

	*info_led = (*info_led) & 0xE0;   // clear bit 4 to turn on "Seg.Display"
}



/**
 * @brief Sends to MAX7221 led controller, via SPI, the register values,
 *        to turn on all the segments on the connected 7-seg led displays.
 *
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 4 is connected to the "Seg.Display" led.
 *                 When on, it indicates any 7-seg led display update.
 *
 * @note For all 7-seg led display boolean values: true is led on, false
 *       is led off.
 *       The info leds are active low: true is led off, false is led on.
 *       Code B decode must be turned on for digits 7-4,
 *       that are connected to the 7-seg led displays.
 *       Variable bit order  :  7 6 5 4 3 2 1 0  (msb to lsb)
 *       Led display segment : DP A B C D E F G
 */

void set_num_all(unsigned char *info_led){
	for (unsigned char icNumber=0; icNumber<3; icNumber++){
		for (unsigned char digit=7; digit>3; digit--){
			// Display "8" & Decimal Point (value 128). 128 + 8 = 136
			setNum(icNumber, digit, 136);
		}
	}
	*info_led = (*info_led) & 0xE0;   // clear bit 4 to turn on "Seg.Display"
}



/**
 * @brief Updates the values which store the state of the switch leds
 *        S20, S21 & S23, to switched off. Also it sends to the Instrument
 *        Panel Cluster, via CAN_Bus, the register values, to turn off
 *        S20, S21 & S23 indications.
 *
 * @param led_sw An array of boolean values where the switch led states will
 *               be updated. The dimension of the array is 36.
 * @param led_sw_old Pointer to the first element of an array of boolean values
 *                   where the previous state of the switch leds, will be updated.
 *                   The dimension of the array is 36.
 * @param lights_status An array of unsigned characters that keeps track of
 *                      the state of the switch leds, grouped (per index) in
 *                      line with the CAN-Bus message byte 3
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 7 is connected to the "Led" led.
 *                 When on, it indicates any switch led / dial led update.
 *
 * @note For all led boolean values: true is led on, false is led off.
 */

void clear_leds_on_lcd(bool led_sw[], bool led_sw_old[], unsigned char lights_status[], unsigned char *info_led){
	// Turn off switch leds S20, S21 & S23
	led_sw[20] = 0;
	led_sw[21] = 0;
	led_sw[23] = 0;
	led_sw_old[20] = 0;
	led_sw_old[21] = 0;
	led_sw_old[23] = 0;

	// Template CAN-Bus "indications" message
	CAN_FRAME frame_switch;
	frame_switch.id = 0x255;
	frame_switch.length = 8;
	frame_switch.data.low  = 0x0000AE04;
	frame_switch.data.high = 0x00000000;

	// Turn off S21 & S20 indications
	frame_switch.data.byte[2] = 0x02;
	frame_switch.data.byte[3] = lights_status[0x02] & 0xF9;
	frame_switch.data.byte[4] = lights_status[0x02] & 0xF9;
	sendCANMessage(&frame_switch);

	// Turn off S23 indication
	frame_switch.data.byte[2] = 0x04;
	frame_switch.data.byte[3] = lights_status[0x02] & 0xDF;
	frame_switch.data.byte[4] = lights_status[0x02] & 0xDF;
	sendCANMessage(&frame_switch);
	usleep(30000);

	*info_led = (*info_led) & 0x70;   // clear bit 7 to turn on "Led"
}



/**
 * @brief Updates the values which store the state of all the switch leds,
 *        to switched off, except S24, S25 & S35. Also it sends to the Instrument
 *        Panel Cluster, via CAN_Bus, the register values, to turn off
 *        all indications, except S24, S25 & S35.
 *
 * @param led_sw An array of boolean values where the switch led states will
 *               be updated. The dimension of the array is 36.
 * @param led_sw_old Pointer to the first element of an array of boolean values
 *                   where the previous state of the switch leds, will be updated.
 *                   The dimension of the array is 36.
 * @param lights_status An array of unsigned characters that keeps track of
 *                      the state of the switch leds, grouped (per index) in
 *                      line with the CAN-Bus message byte 3
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 7 is connected to the "Led" led.
 *                 When on, it indicates any switch led / dial led update.
 *
 * @note For all led boolean values: true is led on, false is led off.
 *       The info leds are active low: true is led off, false is led on.
 */

void clear_switch_leds(bool led_sw[], bool led_sw_old[], unsigned char lights_status[], unsigned char *info_led){
	// Turn off all switch leds, except S24, S25 and S35
	for (unsigned char i=1; i<24; i++){
		led_sw[i] = 0;
		led_sw_old[i] = 0;
	}
	for (unsigned char i=26; i<35; i++){
		led_sw[i] = 0;
		led_sw_old[i] = 0;
	}

	// Template CAN-Bus "indications" message
	CAN_FRAME frame_switch;
	frame_switch.id = 0x255;
	frame_switch.length = 8;
	frame_switch.data.low  = 0x0000AE04;
	frame_switch.data.high = 0x00000000;

	// Clear the indication register file and turn off all indications
	for (unsigned char i=1; i<6; i++){
		lights_status[i] = 0x00;
		frame_switch.data.byte[2] = i;
		sendCANMessage(&frame_switch);
		usleep(30000);
	}

	*info_led = (*info_led) & 0x70;   // clear bit 7 to turn on "Led"
}



/**
 * @brief Zeroizes the RPM/fuel/speed dial leds variables. Calculates
 *        the RPM/fuel/speed dial leds boolean values, that are connected
 *        to RPM/fuel/speed dial leds, in relation to the RPM/fuel/speed
 *        dial leds selected mode of appearance.
 *        mode = 0 : the leds from 0 to RPM/fuel/speed value are on, rest are off
 *        mode = 1 : only led @ RPM/fuel/speed position is on
 *        Calculates (thousands, hundrends,) tens & units for the RPM/fuel/speed
 *        values and sends them to MAX7221 led controller, via SPI, to be
 *        displayed on speed 7-seg led display, in format XXXX and XX.00 .
 *        Also, it sends to the Instrument Panel Cluster, via CAN_Bus, the
 *        register values, to reset to zero position the RPM/fuel/speed
 *        dial indicators (on the Instrument Panel Cluster).
 *
 * @param rpm Number of RPM dial leds that are on, starting
 *            from led @ position 1. The led @ position 0 is
 *            always on when its mode = 0.
 * @param fuel Number of fuel dial leds that are on, starting
 *             from led @ position 1. The led @ position 0 is
 *             always on when its mode = 0.
 * @param sp Number of speed dial leds that are on, starting
 *           from led @ position 1. The led @ position 0 is
 *           always on when its mode = 0.
 * @param mode Array of bolean values for RPM/fuel/sp dial leds selected mode
 *             of appearance.
 * @param led_rpm A boolean array where the RPM dial leds will be updated.
 * @param led_fuel A boolean array where the fuel dial leds will be updated.
 * @param led_sp A boolean array where where the speed dial leds will be updated.
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 7 is connected to the "Led" led.
 *                 When on, it indicates any switch led / dial led update.
 *
 * @note For all led boolean values: true is led on, false is led off.
 *       The info leds are active low: true is led off, false is led on.
 */

void clear_dials(unsigned char *rpm, unsigned char *fuel, unsigned char *sp, bool mode[], bool led_rpm[], bool led_fuel[], bool led_sp[], unsigned char *info_led){
	// Template CAN-Bus "dials" message
	CAN_FRAME frame_rotary;
	frame_rotary.id = 0x255;
	frame_rotary.length = 8;
	frame_rotary.data.low  = 0x0100AE04;
	frame_rotary.data.high = 0x00000000;

	*rpm = 0;
	calc_rpm_leds(rpm, mode, led_rpm);
	show_num_rpm((unsigned int)(*rpm) * 500, info_led);
	// Reset RPM dial indicator to zero position
	frame_rotary.data.byte[2] = 0x09;
	frame_rotary.data.byte[4] = 0x00;
	sendCANMessage(&frame_rotary);
	usleep(30000);

	*fuel = 0;
	calc_fuel_leds(fuel, mode, led_fuel);
	show_num_fuel((unsigned int)(*fuel) * 6, info_led);
	// Reset Fuel Tank Level dial indicator to zero position
	frame_rotary.data.byte[2] = 0x0A;
	frame_rotary.data.byte[4] = 0x00;
	sendCANMessage(&frame_rotary);
	usleep(30000);

	*sp = 0;
	calc_sp_leds(sp, mode, led_sp);
	show_num_sp((unsigned int)(*sp) * 10, info_led);
	// Reset Speed dial indicator to zero position
	frame_rotary.data.byte[2] = 0x08;
	frame_rotary.data.byte[4] = 0x00;
	sendCANMessage(&frame_rotary);
	usleep(30000);

	*info_led = (*info_led) & 0x60; // clear bit 4 to turn on "Seg.Display"
									// and clear bit 7 to turn on "Led"
}



/**
 * @brief Demonstration of all the 7-seg led displays (RPM/fuel/speed)
 *        based on a preset of values, one go with dot point switched on,
 *        and another go with dot point switched off.
 *        Follows up all segments on, then all segments off.
 *        Finnaly, tries to display " rP", "FuEL" and "SPEE" on the
 *        7-seg led displays, meaning "RPM", "FUEL" and "SPEED"
 *
 * @param demoSeg An array of unsigned characters which stores a
 *                preset of values, to display a moving snake on
 *                the 7-seg led displays.
 * @param rfs An array of unsigned characters which stores a preset
 *            of values, to display " rP", "FuEL" and "SPEE" on the
 *            7-seg led displays, meaning "RPM", "FUEL" and "SPEED"
 *
 * @note For all 7-seg led display boolean values: true is led on, false
 *       is led off.
 *       The info leds are not affected during demonstration.
 *       Code B decode must be turned off for digits 7-4, that are
 *       connected to the 7-seg led displays, during demonstartion
 *       Variable bit order  :  7 6 5 4 3 2 1 0  (msb to lsb)
 *       Led display segment : DP A B C D E F G
 */

void demo_segs(unsigned char demoSeg[], unsigned char rfs[]){
	for (unsigned char dot_point_switch=0; dot_point_switch<2; dot_point_switch++){
		for (unsigned char i=0; i<8; i++){
			for (unsigned char icNumber=0; icNumber<3; icNumber++){
				for (unsigned char segment=7; segment>3; segment--){
					setRow(icNumber, segment, demoSeg[i] + dot_point_switch*128);
				}
			}
			usleep(demoDelayTime1 * 1000);
		}
	}
	for (unsigned char icNumber=0; icNumber<3; icNumber++){
		for (unsigned char segment=7; segment>3; segment--){
			setRow(icNumber, segment, 0xFF);
		}
	}
	usleep(demoDelayTime2 * 1000);
	for (unsigned char icNumber=0; icNumber<3; icNumber++){
		for (unsigned char segment=7; segment>3; segment--){
			setRow(icNumber, segment, 0x00);
		}
	}
	usleep(demoDelayTime2 * 1000);
	unsigned char i=0;
	for (unsigned char icNumber=0; icNumber<3; icNumber++){
		for (unsigned char segment=7; segment>3; segment--){
			setRow(icNumber, segment, rfs[i++]);
		}
	}
	usleep(demoDelayTime2 * 1000);
}



/**
 * @brief Demonstration of all the switch leds.
 *        The leds are switched on, one after another and they stay on,
 *        in order, based on a preset order. Switch S11 is bi-color, so
 *        it switches between yellow and red.
 *        Finally, it switches off all switch leds.
 *
 * @param led_sw An array of boolean values where the switch led states will
 *               be updated. The dimension of the array is 36.
 * @param demoButtons An array of unsigned char values which stores a
 *                    preset of led lighting order, for demonstration
 * @param led_rpm A boolean array where the RPM dial leds will be updated.
 * @param led_fuel A boolean array where the fuel dial leds will be updated.
 * @param led_sp A boolean array where where the speed dial leds will be updated.
 * @param led_sw11_color Switch 11 red/yellow led boolean value. True is red.

 * @param led_sw Pointer to the first element of an array of boolean values         // delete
 *           where the switch led states will be updated.
 *           The dimension of the array is 36.
 * @param led_rpm[] RPM dial leds 0 to 16 boolean values.
 * @param led_fuel[] Fuel dial leds 0 to 8 boolean values.
 * @param led_sp[] Speed dial leds 0 to 24 boolean values.
 * @param led_sw[] Switch leds 1 to 35 boolean values. Switch 11 is bi-color.
 * @param led_sw11_color Switch 11 red/yellow led boolean value. True is red.
 * @param mx A 2D array of unsigned characters and dimensions of 3 x 8 .
 *           A part of this array stores information about the on/off state
 *           of the connected switch leds and dial leds.
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 7 is connected to the "Led" led.
 *                 When on, it indicates any switch led / dial led update.
 *
 * @note For all led boolean values: true is led on, false is led off.
 *       The info leds are active low: true is led off, false is led on.
 */

void demo_switch_leds(bool led_sw[], unsigned char demoButtons[], bool led_rpm[], bool led_fuel[], bool led_sp[], bool *led_sw11_color, unsigned char mx[][8], unsigned char *info_led){
	for (unsigned char i=0; i<36; i++){
		led_sw[demoButtons[i]] = 1;
		fill_led_table(led_rpm, led_fuel, led_sp, led_sw, led_sw11_color, mx);
		show_leds(mx, info_led);
		usleep(demoDelayTime3 * 1000);

		if (demoButtons[i] == 11){
			// demoButtons[5] is 11, and switch S11 led is bi-color
			usleep(2 * demoDelayTime3 * 1000);
			*led_sw11_color = 1;
			fill_led_table(led_rpm, led_fuel, led_sp, led_sw, led_sw11_color, mx);
			show_leds(mx, info_led);
			usleep(demoDelayTime3 * 1000);
		}
	}
	usleep(demoDelayTime4 * 1000);
	for (unsigned char i=0; i<36; i++){
		led_sw[demoButtons[i]] = 0;
	}
	*led_sw11_color = 0;
	fill_led_table(led_rpm, led_fuel, led_sp, led_sw, led_sw11_color, mx);
	show_leds(mx, info_led);
}



/**
 * @brief Demonstration of all the dial leds.
 *        The RPM/fuel/speed dial leds are switched on, in the same angular
 *        speed, so that they reach their minimum / maximum value at the
 *        same time. A preset table of values helps in this procedure.
 *        The demonstration runs two times, one in dot mode of appearance,
 *        and another one in led bar mode of appearance.
 *        mode = 0 : the leds from 0 to RPM/fuel/speed value are on, rest are off.
 *        mode = 1 : only led @ RPM/fuel/speed position is on.
 *        Calculates (thousands, hundrends,) tens & units for the RPM/fuel/speed
 *        values and sends them to MAX7221 led controller, via SPI, to be
 *        displayed on speed 7-seg led display, in format XXXX and XX.00 .
 *
 * @param demo A struct of type DemoData that stores preset values, that indicates
 *             at which time of running the demonstration, the RPM/fuel/speed
 *             values change, and their dial leds value at that time.
 * @param led_sw An array of boolean values where the switch led states will
 *               be updated. The dimension of the array is 36.
 * @param led_sw11_color Switch 11 red/yellow led boolean value. True is red.
 * @param mx A 2D array of unsigned characters and dimensions of 3 x 8 .
 *           A part of this array stores information about the on/off state
 *           of the connected switch leds and dial leds.
 * @param info_led An unsigned char value, of which the upper nibble stores
 *                 information about the 4 status active-low leds.
 *                 Its bit 7 is connected to the "Led" led.
 *                 When on, it indicates any switch led / dial led update.
 *
 * @note For all led boolean values: true is led on, false is led off.
 *       The info leds are active low: true is led off, false is led on.
 */

void demo_dial_leds(DemoData demo[], bool led_sw[], bool *led_sw11_color, unsigned char mx[][8], unsigned char *info_led){
	// Temporary RPM/fuel/speed dial leds bool arrays
	bool led_rpm_demo[rpm_max] = {0};
	bool led_fuel_demo[fuel_max] = {0};
	bool led_sp_demo[sp_max] = {0};
	bool mode_demo[4] = {0};

	bool change = false;
	unsigned char rpm = 0, fuel = 0, sp = 0;
	unsigned char rpm_old = 255, fuel_old = 255, sp_old = 255;
	unsigned long t, t_now, t_start;

	for (bool demo_mode = true; ; demo_mode = !demo_mode) {
		mode_demo[1] = demo_mode;
		mode_demo[2] = demo_mode;
		mode_demo[3] = demo_mode;
		// Forward
		t_start = millis;
		for (unsigned char i=0; i<33; i++){
			t = (unsigned long)(demo[i].time);
			rpm  = demo[i].rpm;
			fuel = demo[i].fuel;
			sp   = demo[i].sp;
			do {
				t_now = millis;
			} while ((t_now - t_start) < t);
			if (rpm != rpm_old){
				calc_rpm_leds(&rpm, mode_demo, led_rpm_demo);
				show_num_rpm((unsigned int)(rpm) * 500, info_led);
				rpm_old = rpm;
				change = true;
			}
			if (fuel != fuel_old){
				calc_fuel_leds(&fuel, mode_demo, led_fuel_demo);
				show_num_fuel((unsigned int)(fuel) * 6, info_led);
				fuel_old = fuel;
				change = true;
			}
			if (sp != sp_old){
				calc_sp_leds(&sp, mode_demo, led_sp_demo);
				show_num_sp((unsigned int)(sp) * 10, info_led);
				sp_old = sp;
				change = true;
			}
			if (change){
				fill_led_table(led_rpm_demo, led_fuel_demo, led_sp_demo, led_sw, led_sw11_color, mx);
				show_leds(mx, info_led);
				change = false;
			}
		}
		usleep(demoDelayTime5 * 1000);
		// Backward
		t_start = millis;
		for (unsigned char i=0; i<33; i++){
			t = (unsigned long)(demo[i].time);
			rpm  = demo[32-i].rpm;
			fuel = demo[32-i].fuel;
			sp   = demo[32-i].sp;
			do {
				t_now = millis;
			} while ((t_now - t_start) < t);
			if (rpm != rpm_old){
				calc_rpm_leds(&rpm, mode_demo, led_rpm_demo);
				show_num_rpm((unsigned int)(rpm) * 500, info_led);
				rpm_old = rpm;
				change = true;
			}
			if (fuel != fuel_old){
				calc_fuel_leds(&fuel, mode_demo, led_fuel_demo);
				show_num_fuel((unsigned int)(fuel) * 6, info_led);
				fuel_old = fuel;
				change = true;
			}
			if (sp != sp_old){
				calc_sp_leds(&sp, mode_demo, led_sp_demo);
				show_num_sp((unsigned int)(sp) * 10, info_led);
				sp_old = sp;
				change = true;
			}
			if (change == 1){
				fill_led_table(led_rpm_demo, led_fuel_demo, led_sp_demo, led_sw, led_sw11_color, mx);
				show_leds(mx, info_led);
				change = false;
			}
		}
		usleep(demoDelayTime5 * 1000);
		if (!demo_mode) {
			break;
		}
	}
}
