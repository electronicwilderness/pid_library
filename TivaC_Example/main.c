/******************************************************************************
 * FILE NAME:  main.c
 *
 * FILE DESCRIPTION:
 * An example implementation of the PID library.  It shows the initialization
 * and operation of PIDS, where PID_COUNT represents the number of PIDs that
 * the application implements.
 *
 * The application will make variables for the set point, process input and
 * process output.  The PID library uses the reference to these variables in
 * the calculation.
 *
 * This example uses random numbers to generate the system reaction and
 * operation.  There are delays to simulate the speed at which the operation
 * can be running at.
 *
 * LICENSE:
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Electronic Wilderness
 * 						(https://github.com/electronicwilderness)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * FILE NOTES:
 * None.
 *
 *****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "drivers/pinout.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "main.h"
#include "pid/pid.h"

/******************************************************************************
 *                          GLOBAL DEFINITIONS
 *****************************************************************************/
// This will make the process have the range from 0 to PROCESS_OUTPUT_RANGE
#define PROCESS_OUTPUT_RANGE    	100
// When operating the process, this determines how fast it will move or react
#define PROCESS_MOVEMENT_RANGE   	10
// The number of PIDs that are created by the application
#define PID_COUNT					3

/******************************************************************************
 *                          APPLICATION VARIABLES
 *****************************************************************************/
uint32_t g_ui32SysClock;		// The clock rate in Hz
pid app_pid[PID_COUNT];			// The array of pointers to the PID structure.
// The set point of the system.
double app_setpoint[PID_COUNT] = { 0 };
// The input to the process or also known as the output from the PID.
double app_process_input[PID_COUNT] = { 0 };
// The output value from the process which is the input to the difference of
// the PID.
double app_process_output[PID_COUNT] = { 0 };
// The speed (or reaction) at which the process the PID is trying to operate.
double app_process_movement[PID_COUNT] = { 0 };

/******************************************************************************
 *                          FUNCTION PROTOTYPES
 *****************************************************************************/
void OperateMachine();

/******************************************************************************
 * FUNCTION NAME:  main
 *
 * FUNCTION DESCRIPTION:
 * This is the application calling function for the PID library.  It is an
 * example of the use of the PID library.
 *
 * FUNCTION PARAMETERS:
 * None.
 *
 * FUNCTION RETURN VALUE:
 * int
 *
 * FUNCTION NOTES:
 * None.
 *
 *****************************************************************************/
int main(void) {
	int i, j;
	return_pid_e pid_result;

	// Run the PLL at 120 MHz.
	g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
	SYSCTL_CFG_VCO_480),
											120000000);

	PinoutSet(false, false);	// Configure the device pins.

	// Enable the GPIO pin for the LED D1 (PN1).
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

	// Display general program information.
	debug("-------------------------------------------------%s", "\r\n");
	debug("- Compiled: %s %s\t\t-\r\n", __DATE__, __TIME__);
	debug("-------------------------------------------------%s", "\r\n");
	debug("- RAM Size: %d\tFlash Size: %d\t-\r\n", SysCtlSRAMSizeGet(),
			SysCtlFlashSizeGet());
	debug("-------------------------------------------------%s", "\r\n");
	debug("- PID Library Version: %s\t\t\t-\r\n", PID_LIBRARY_VERSION);
	debug("-------------------------------------------------%s", "\r\n");

	srand(g_ui32SysClock);    //seed the random number

	for (i = 0; i < PID_COUNT; i++) {
		//allocate memory to the application pid pointer array
		app_pid[i] = malloc(sizeof(pid));
		//Initialize the PID
		pid_result = InitializePID((struct pid_t **) app_pid[i],
									&app_setpoint[i], &app_process_output[i],
									&app_process_input[i], 1.02, 0.001, 0.0);
		if (pid_result != SUCCESSFUL) {
			//error - freeze program
			debug("Error during initializing of PID.%s", "\r\n");
			while (1) {
			}
		}
		app_process_input[i] = 0;
		app_process_output[i] = 0;
		// PID operation with limits set.
		/*pid_result = SetPIDLimits((struct pid_t **) app_pid[i], 40, 70);
		 if (pid_result != SUCCESSFUL){
		 //error - freeze program
		 debug("Error during setting of PID limits.%s","\r\n");
		 while (1) {}
		 }*/
	}

	while (1) {
		//update set point
		for (i = 0; i < PID_COUNT; i++) {
			app_setpoint[i] = rand() / (RAND_MAX / PROCESS_OUTPUT_RANGE);
			debug("Set point: %d \tProcess movement range: %d\tProcess Input:" " %d \tProcess output: %d\r\n",
					(int ) app_setpoint[i], (int ) app_process_movement[i],
					(int ) app_process_input[i], (int ) app_process_output[i]);
		}
		LEDWrite(CLP_D1, 1);	// Turn on D1.
		// Operate the system for 50 iterations
		for (i = 0; i < 50; i++) {
			for (j = 0; j < PID_COUNT; j++) {// For each of the PID control loops
				OperateMachine(j);	// operate the virtual machine
				// Call the PID function to update the control_variable
				OperatePID((struct pid_t **) app_pid[j]);
			}
			SysCtlDelay(g_ui32SysClock / 10 / 3);	// delay
		}
		LEDWrite(CLP_D1, 0);	// Turn off D1.
		SysCtlDelay(g_ui32SysClock / 10 / 3);	// delay for visual LED effect
	}
}

/******************************************************************************
 * FUNCTION NAME:  DeInitializePID
 *
 * FUNCTION DESCRIPTION:
 * This function generates a simulated operation on a process based on the
 * reaction, or speed of movement, during operation of a real system.  It uses
 * a random number to make the system change.
 *
 * FUNCTION PARAMETERS:
 * int	index	Index of the PID that is being updated.
 *
 * FUNCTION RETURN VALUE:
 * None.
 *
 * FUNCTION NOTES:
 * None.
 *
 *****************************************************************************/
void OperateMachine(int index) {

	// Generate a normalized random movement value for this itteration of the
	// machine operation.
	app_process_movement[index] = (rand() / (RAND_MAX / PROCESS_MOVEMENT_RANGE))
			/ 1.0587;
	// Determine whether the operation is additive or subtractive.
	if (app_process_output[index] > app_process_input[index]) {
		app_process_output[index] = app_process_output[index]
				- app_process_movement[index];
	} else if (app_process_output[index] < app_process_input[index]) {
		app_process_output[index] = app_process_output[index]
				+ app_process_movement[index];
	}
}
