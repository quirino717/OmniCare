#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct{

	double set_point;  // setPoint
	double input;
	double output; // Value

	double last_error;

	float kp;
	float ki;
	float kd;

	double integrated_error;
	double MAX_INTEGRAL_ERROR;

	uint32_t last_time_call_ms;

	double max_output;

	int first_time = 0;
} PID;

void initPID(PID *pid, float kp, float ki, float kd, double max_output);
void update(PID *pid);

#endif /* INC_PID_H_ */
