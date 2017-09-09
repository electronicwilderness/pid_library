# pid_library

A library that provides a PID control loop mechanism with configurable coefficients that operate the P, I and D. The calling application can create one or more PIDs using this library.

The operation of the PID is done by having the calling application create a reference to the pid_t structure and then initializing that structure. The calling of the function that operates the PID is to be intended to be called after a specific amount of time determined by the calling function. Which allows for greater flexibility by the user of this library to determine when the operations with the PID occur in the process.

The general usage of this library can be seen in the pseudo code example below: 
```c	
  pid app_pid; 
	app_pid = malloc(sizeof(pid));
	InitializePID((struct pid_t **) app_pid, &setpoint, &process_output, &process_input, 1, 0.0, 0.0);
	while(1){ 
		if (timer < SETPOINT_TIMER){
			SETPOINT_TIMER = timer + TIME_SETPOINT; //update set point 
			app_setpoint = AppGetSetPoint(); 
		} 
		if (timer < OPERATE_PID_TIMER){ 
			OPERATE_PID_TIMER = timer + TIME_OPERATE_PID; 
			// operate the PID to recalculate the process input 
			OperatePID((struct pid_t **) app_pid); 
			// Now that there is a new process input update the machine 
			OperateMachine(); 
		} 
	}
```

Details on use of library and function calls are available in function headings of pid.c.

![alt text](https://github.com/electronicwilderness/pid_library/PID Library Diagram.png "PID Library Diagram")
