#include "pid.h"

//#include "tx_api.h"

//TODO: Arrumar o last_time_call
void initPID(PID *pid, float kp, float ki, float kd,
		     double max_output, double max_integral_error,
			 uint32_t last_time_call)
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

	pid->last_time_call_ms = last_time_call_ms;

	pid->max_output = max_output;
}

void update(PID *pid, double input, uint32_t atual_ms)
{

	uint32_t dt = atual_ms - pid->last_time_call_ms;
	double error = pid->set_point - input;

	pid->integrated_error += error * dt;

	// Evitar windup da integral
	if (pid->integrated_error >  pid->MAX_INTEGRAL_ERROR) pid->integrated_error =  pid->MAX_INTEGRAL_ERROR;
	if (pid->integrated_error < -pid->MAX_INTEGRAL_ERROR) pid->integrated_error = -pid->MAX_INTEGRAL_ERROR;


	derivate 	= (error - pid->last_error)/dt;

	pid->last_error = error;


			motor_vel_pwm[i] += motors_data[i].PID[0] * error[i] +  motors_data[i].PID[1] * integral[i] + motors_data[i].PID[2] * derivate[i];
			motors_data[i].PWM = motor_vel_pwm[i];

			if(abs(motor_vel_pwm[i]) >= 100000)
			{
				motor_vel_pwm[i] = abs(motor_vel_pwm[i])/motor_vel_pwm[i] * -15;
			}
		}
		update_velocity(motor_vel_pwm);
	}
	last_time_ms = atual_ms;
}
}
