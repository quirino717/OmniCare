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
	double min_output;
} PID;

void initPID(PID *pid, float kp, float ki, float kd,
		     double max_output, double min_output, double max_integral_error);
double update(PID *pid, double input);
void setGains(PID *pid, float kp, float ki, float kd);

#endif /* INC_PID_H_ */
