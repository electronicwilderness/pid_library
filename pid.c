/******************************************************************************
 * FILE NAME:  pid.c
 *
 * FILE DESCRIPTION:
 * This file provides a PID control loop mechanism with configurable
 * coefficients that operate the P, I and D.  The PID output or control
 * variable can be limited as well.
 *
 * The operation of the PID is done by having the calling application create a
 * reference to the pid_t structure and then initializing that structure.  The
 * calling of the function that operates the PID is to be intended to be
 * called after a specific amount of time determined by the calling function.
 * Which allows for greater flexibility by the user of this library to
 * determine when the operations with the PID occur in the process.
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
 * The general usage of this library can be seen in the pseudo code example
 * below:
 *		pid app_pid;
 *		app_pid = malloc(sizeof(pid));
 * 		InitializePID((struct pid_t **) app_pid, &setpoint, &process_output,
 * 				&process_input, 1, 0.0, 0.0);
 * 		while(1){
 * 			if (timer < SETPOINT_TIMER){
 * 				SETPOINT_TIMER = timer + TIME_SETPOINT;
 * 				//update set point
 * 				app_setpoint = AppGetSetPoint();
 * 			}
 * 			if (timer < OPERATE_PID_TIMER){
 * 				OPERATE_PID_TIMER = timer + TIME_OPERATE_PID;
 * 				// operate the PID to recalculate the process input
 * 				OperatePID((struct pid_t **) app_pid);
 * 				// Now that there is a new process input update the machine
 * 				OperateMachine();
 * 			}
 * 		}
 *
 *****************************************************************************/

/******************************************************************************
 *                              FILE INCLUDES
 *****************************************************************************/
#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/******************************************************************************
 *                              TYPE DEFINES
 *****************************************************************************/
// This is the internal structure that is used by the library.
typedef struct {
	double *set_point_ptr;			// set point
	double *process_value_ptr;		// process value
	double *control_variable_ptr;	// control variable
	double proportional;			// proportional coefficient
	double integral;				// integral coefficient
	double differential;			// differential coefficient
	double previous_process_value;	// previous process value
	double integeral_error_sum;		// running sum of error during operation
	double error;					// current error of the system
	double low_limit;				// low limit of control variable
	double high_limit;				// high limit of control variable
} pid_t;

/******************************************************************************
 * FUNCTION NAME:  InitializePID
 *
 * FUNCTION DESCRIPTION:
 * This function initializes the pid_t structure that is used in operation of
 * the PID.  Error checking, variable setting and memory allocation is
 * performed in this function.
 *
 * FUNCTION PARAMETERS:
 * pid_t	**user_pid			The pointer of a pointer to the internal PID
 * 								structure that gets initialized in this
 * 								function.
 * double   *set_point_ptr      The memory pointer to the set point of the
 *                              system from the calling application.
 * double   *process_output_ptr The memory pointer to the output of the process
 *                              or the input into the system error sum.
 * double   *process_input_ptr  The memory pointer to the input of the process.
 * double   kp                  The proportional constant of the PID.
 * double   ki                  The integral constant of the PID.
 * double   kd                  The differential constant of the PID.
 *
 * FUNCTION RETURN VALUE:
 * return_pid_e     The return value from this function with the possible
 *                  results:
 *                      SUCCESSFUL - The initialization has been successful.
 *                      INVALID_POINTER - At least one of the pointers passed
 *                      into this function was Null and is therefore invalid.
 *                      INVALID_COEFFICIENT - At least one of the coefficients
 *                      is zero, they all must be non-negative.
 *
 * FUNCTION NOTES:
 * When the PID is not needed, the DeInitializePID() function needs to be
 * called to free the memory.
 *
 *****************************************************************************/
return_pid_e InitializePID(struct pid_t **user_pid, double *set_point_ptr,
							double *process_output_ptr,
							double *process_input_ptr, double kp, double ki,
							double kd) {
	volatile return_pid_e result = SUCCESSFUL;
	volatile pid_t *temp_pid;

	// Check that the pointers passed in are valid.
	if (user_pid == NULL || set_point_ptr == NULL || process_output_ptr == NULL
			|| process_input_ptr == NULL) {
		result = INVALID_POINTER;
	} else {
		// Check that the coefficients are valid.
		if (kp < 0.0 || ki < 0.0 || kd < 0.0) {
			result = INVALID_COEFFICIENT;
		} else {
			// Allocate memory to the PID structure.
			temp_pid = calloc(1, sizeof(pid_t));
			// Update the pointer of the application
			*user_pid = (struct pid_t *) temp_pid;
			// Update the values of the PID.
			temp_pid->set_point_ptr = set_point_ptr;
			temp_pid->process_value_ptr = process_output_ptr;
			temp_pid->control_variable_ptr = process_input_ptr;
			temp_pid->proportional = kp;
			temp_pid->integral = ki;
			temp_pid->differential = kd;
			temp_pid->previous_process_value = 0.0;
			temp_pid->integeral_error_sum = 0.0;
			temp_pid->error = 0.0;
			temp_pid->low_limit = NAN;	// set to use as condition if used
			temp_pid->high_limit = NAN;	// set to use as condition if used
		}
	}

	return result;
}

/******************************************************************************
 * FUNCTION NAME:  DeInitializePID
 *
 * FUNCTION DESCRIPTION:
 * Dereferences the PID pointers from the calling application and frees the
 * allocated memory to internal variables.
 *
 * FUNCTION PARAMETERS:
 * pid_t	**user_pid			The pointer of a pointer to the internal PID
 * 								structure that gets initialized in this
 * 								function.
 *
 * FUNCTION RETURN VALUE:
 * None.
 *
 * FUNCTION NOTES:
 * There is not checking of a valid user_pid value.
 *
 *****************************************************************************/
void DeInitializePID(struct pid_t **user_pid) {
	volatile pid_t *temp_pid = (volatile pid_t *) *user_pid;

	//set PID pointer values to Null and free the memory for the PID
	temp_pid->set_point_ptr = NULL;
	temp_pid->process_value_ptr = NULL;
	temp_pid->control_variable_ptr = NULL;
	free((void *) temp_pid);
}

/******************************************************************************
 * FUNCTION NAME:  SetCoefficients
 *
 * FUNCTION DESCRIPTION:
 * Updates the PID coefficients that are used with the PID including error
 * checking of the coefficients.
 *
 * FUNCTION PARAMETERS:
 * pid_t	**user_pid		The pointer of a pointer to the internal PID
 * 							structure that gets initialized in this function.
 * double   kp      		The updated proportional PID term.
 * double   ki     			The updated integral PID term.
 * double   kd      		The updated differential PID term.
 *
 * FUNCTION RETURN VALUE:
 * return_pid_e     Returns SUCCESSFUL when the operation completed.  When the
 *                  PID coefficients are less then 0.0, INVALID_COEFFICIENT
 *                  will be returned.
 *
 * FUNCTION NOTES:
 * None.
 *
 *****************************************************************************/
return_pid_e SetCoefficients(struct pid_t **user_pid, double kp, double ki,
								double kd) {
	volatile return_pid_e result = SUCCESSFUL;
	volatile pid_t *temp_pid = (volatile pid_t *) *user_pid;

	// Check to see if the user_pid is valid.
	if (user_pid == NULL) {
		return INVALID_POINTER;
	}

	// Check if the coefficients are valid.
	if (kp < 0.0 || ki < 0.0 || kd < 0.0) {
		result = INVALID_COEFFICIENT;
	} else {
		temp_pid->proportional = kp;
		temp_pid->integral = ki;
		temp_pid->differential = kd;
	}

	return result;
}

/******************************************************************************
 * FUNCTION NAME:  GetKp
 *
 * FUNCTION DESCRIPTION:
 * Retrieves the proportional PID term.
 *
 * FUNCTION PARAMETERS:
 * pid_t	**user_pid		The pointer of a pointer to the internal PID
 * 							structure that gets initialized in this function.
 *
 * FUNCTION RETURN VALUE:
 * double   The proportional PID term.
 *
 * FUNCTION NOTES:
 * There is not checking of a valid user_pid value.
 *
 *****************************************************************************/
double GetKp(struct pid_t **user_pid) {
	volatile pid_t *temp_pid = (volatile pid_t *) *user_pid;

	return temp_pid->proportional;
}

/******************************************************************************
 * FUNCTION NAME:  GetKi
 *
 * FUNCTION DESCRIPTION:
 * Retrieves the integral PID term.
 *
 * FUNCTION PARAMETERS:
 * pid_t	**user_pid		The pointer of a pointer to the internal PID
 * 							structure that gets initialized in this function.
 *
 * FUNCTION RETURN VALUE:
 * double   The integral PID term.
 *
 * FUNCTION NOTES:
 * There is not checking of a valid user_pid value.
 *
 *****************************************************************************/
double GetKi(struct pid_t **user_pid) {
	volatile pid_t *temp_pid = (volatile pid_t *) *user_pid;

	return temp_pid->integral;
}

/******************************************************************************
 * FUNCTION NAME:  GetKd
 *
 * FUNCTION DESCRIPTION:
 * Retrieves the differential PID term.
 *
 * FUNCTION PARAMETERS:
 * pid_t	**user_pid		The pointer of a pointer to the internal PID
 * 							structure that gets initialized in this function.
 *
 * FUNCTION RETURN VALUE:
 * double   The differential PID term.
 *
 * FUNCTION NOTES:
 * There is not checking of a valid user_pid value.
 *
 *****************************************************************************/
double GetKd(struct pid_t **user_pid) {
	volatile pid_t *temp_pid = (volatile pid_t *) *user_pid;

	return temp_pid->differential;
}

/******************************************************************************
 * FUNCTION NAME:  OperatePID
 *
 * FUNCTION DESCRIPTION:
 * Operates the PID based upon the user_pid passed in.  This function is
 * intended to operate from the application on a specific time interval
 * determined by the application.
 *
 * The PID operates on the following formula:
 * 		error = set_point - process_value
 * 		proportinal_term = kp * error
 * 		integeral_sum += ki * error
 * 		derivative_term = kd * (error - (set_point - prev_process_value))
 * 		control_variable = process_value + proportinal_term + integral_sum
 * 				+ derivative_term
 * This formula uses the process_value directly in determining the
 * control_variable.  This means that the error, input and output of the
 * system is based upon the same unit of measure.
 *
 * FUNCTION PARAMETERS:
 * pid_t	**user_pid		The pointer of a pointer to the internal PID
 * 							structure that gets initialized in this function.
 *
 * FUNCTION RETURN VALUE:
 * return_pid_e		A successful function call returns SUCCESSFUL to the
 * 					calling application.  If the calling function does not
 * 					have a valid pointer, INVALID_POINTER is returned.
 *
 * FUNCTION NOTES:
 * None.
 *
 *****************************************************************************/
return_pid_e OperatePID(struct pid_t **user_pid) {
	volatile double error_differential;
	volatile pid_t *temp_pid = (volatile pid_t *) *user_pid;

	// Check to see if the user_pid is valid.
	if (user_pid == NULL) {
		return INVALID_POINTER;
	}

	// Update the error for this iteration.
	temp_pid->error = *temp_pid->set_point_ptr - *temp_pid->process_value_ptr;

	// Calculate the integral error for this iteration.
	temp_pid->integeral_error_sum += temp_pid->error * temp_pid->integral;

	// Calculate the differential error for this iteration.
	error_differential = temp_pid->error
			- (*temp_pid->set_point_ptr - temp_pid->previous_process_value);

	// Update the previous value with the present one.
	temp_pid->previous_process_value = *temp_pid->process_value_ptr;

	//Calculate the control variable using the coefficients.
	*temp_pid->control_variable_ptr = *temp_pid->process_value_ptr
			+ (temp_pid->proportional * temp_pid->error)
			+ (temp_pid->integeral_error_sum)
			+ (temp_pid->differential * error_differential);

	// Check to see if there are limits to check for the PID control variable.
	if (temp_pid->low_limit != NAN && temp_pid->high_limit != NAN) {
		// Check the high limit.
		if (*temp_pid->control_variable_ptr > temp_pid->high_limit) {
			*temp_pid->control_variable_ptr = temp_pid->high_limit;
		}
		// Check the low limit.
		if (*temp_pid->control_variable_ptr < temp_pid->low_limit) {
			*temp_pid->control_variable_ptr = temp_pid->low_limit;
		}
	}

	return SUCCESSFUL;
}

/******************************************************************************
 * FUNCTION NAME:  SetPIDLimits
 *
 * FUNCTION DESCRIPTION:
 * Setting of the PID output (control variable) limit.  A value of NAN for the
 * low_value or high_value is used as an indicator by the function OperatePID()
 * to determine if there are PID limits used.
 *
 * FUNCTION PARAMETERS:
 * pid_t	**user_pid		The pointer of a pointer to the internal PID
 * 							structure that gets initialized in this function.
 * double	low_value		The lowest value that the PID can output to the
 * 							process.
 * double 	high_value		The highest value that the PID can output to the
 * 							process.
 *
 * FUNCTION RETURN VALUE:
 * return_pid_e		A successful function call returns SUCCESSFUL to the
 * 					calling application.  If the calling function does not
 * 					have a valid pointer, INVALID_POINTER is returned.  If the
 * 					calling function does not have valid low_value and
 * 					high_value parameters, INVALID_NUMBER is returned.
 *
 * FUNCTION NOTES:
 * None.
 *
 *****************************************************************************/
return_pid_e SetPIDLimits(struct pid_t **user_pid, double low_value,
							double high_value) {
	volatile return_pid_e result = SUCCESSFUL;
	volatile pid_t *temp_pid = (volatile pid_t *) *user_pid;

	// Check to see if the user_pid is valid.
	if (user_pid == NULL) {
		return INVALID_POINTER;
	}

	// Check if the low_value is equal to or greater then the high value, the
	// low_value = NAN or the high_value = NAN.
	if ((low_value >= high_value) || (low_value == NAN)
			|| (high_value == NAN)) {
		result = INVALID_NUMBER;
	} else {
		//set the low and high limits for the PID control variable
		temp_pid->low_limit = low_value;
		temp_pid->high_limit = high_value;
	}

	return result;
}
