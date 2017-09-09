/******************************************************************************
 * FILE NAME:  pid.h
 *
 * FILE DESCRIPTION:
 * The header file that defines the structures and functions that can be
 * accessed by an application.
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

#ifndef PID_PID_H_
#define PID_PID_H_

/******************************************************************************
 *                          DEFINITIONS
 *****************************************************************************/
#define PID_LIBRARY_VERSION    "0.01"

/******************************************************************************
 *                              TYPE DEFINES
 *****************************************************************************/
// An opaque pointer to the internal library structure.
typedef struct pid_t *pid;

// PID library return types.
typedef enum {
	SUCCESSFUL,				// The function was successfully completed
	INVALID_COEFFICIENT,	// The coefficient is not in the proper range
	INVALID_POINTER,		// The pointer is not valid
	INVALID_NUMBER			// The number being used is not in the proper range
} return_pid_e;

/******************************************************************************
 *                          FUNCTION PROTOTYPES
 *****************************************************************************/
return_pid_e InitializePID(struct pid_t **user_pid, double *set_point_ptr,
							double *input_ptr, double *output_ptr, double kp,
							double ki, double kd);
return_pid_e SetCoefficients(struct pid_t **user_pid, double kp, double ki,
								double kd);
double GetKp(struct pid_t **user_pid);
double GetKi(struct pid_t **user_pid);
double GetKd(struct pid_t **user_pid);
return_pid_e OperatePID(struct pid_t **user_pid);
void DeInitializePID(struct pid_t **user_pid);
return_pid_e SetPIDLimits(struct pid_t **user_pid, double low_value,
							double high_value);

#endif /* PID_PID_H_ */
