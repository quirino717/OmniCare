#include "pid.h"

#include <math.h>

#include "tx_api.h"

void initPID(PID *pid, float kp, float ki, float kd,
		     double max_output, double min_output, double max_integral_error)
{
	pid->set_point  = 0;
	pid->input      = 0;
	pid->output     = 0;
	pid->last_error = 0;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->integrated_error = 0;
	pid->MAX_INTEGRAL_ERROR = max_integral_error;

	unsigned long atual_ms = (tx_time_get() * 1000) / TX_TIMER_TICKS_PER_SECOND;
	pid->last_time_call_ms = atual_ms;

	pid->max_output = max_output;
	pid->min_output = min_output;
}

double update(PID *pid, double input)
{
	unsigned long current_time_ms = (tx_time_get() * 1000) / TX_TIMER_TICKS_PER_SECOND;
	uint32_t dt = current_time_ms - pid->last_time_call_ms;
	double error = pid->set_point - input;

	pid->integrated_error += error * dt;

	// Evitar windup da integral
	if (pid->integrated_error >  pid->MAX_INTEGRAL_ERROR) pid->integrated_error =  pid->MAX_INTEGRAL_ERROR;
	if (pid->integrated_error < -pid->MAX_INTEGRAL_ERROR) pid->integrated_error = -pid->MAX_INTEGRAL_ERROR;

	double error_derivate = (error - pid->last_error)/dt;

	pid->last_error = error;

	pid->output += pid->kp * error +
				   pid->ki * pid->integrated_error +
				   pid->kd * error_derivate;

	// TODO: arrumar essa gambiarra
	if(fabs(pid->output) >= 100000)
	{
		pid->output = fabs(pid->output)/pid->output * -15;
	}

	if(pid->output > pid->max_output ) pid->output = pid->max_output;
	if(pid->output < pid->min_output ) pid->output = pid->min_output;

	pid->last_time_call_ms = current_time_ms;

	return pid->output;
}

void setGains(PID *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}
