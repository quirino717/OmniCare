#include <motor_base.h>
#include <stdlib.h>

#include "tx_api.h"

void init_motor_robot_base(RobotBaseMotor *robot_base_motor,
						   float kp, float ki, float kd,
						   double max_output, double min_output,
						   double max_integral_error)
{
	robot_base_motor->current_velocity    = 0.0;

	robot_base_motor->encoder_last_count   = 0;
	robot_base_motor->encoder_curret_count = 0;

	unsigned long current_time_ms = (tx_time_get() * 1000) / TX_TIMER_TICKS_PER_SECOND;
	robot_base_motor->last_time_call_ms = current_time_ms;

	initPID(robot_base_motor->pid, kp, ki, kd,
			max_output, min_output,
			max_integral_error);
}

void read_velocity_motor_base(RobotBaseMotor *robot_base_motor)
{
	unsigned long current_time_ms = (tx_time_get() * 1000) / TX_TIMER_TICKS_PER_SECOND;
	unsigned long dt_ms = current_time_ms - robot_base_motor->last_time_call_ms;
	int32_t diff_count = robot_base_motor->encoder_curret_count - robot_base_motor->encoder_last_count;

	// verificacao para tratar estouro do buffer
	if(abs(diff_count) > 2e16)
	{
		diff_count = (2e32 - abs(diff_count)) * (-diff_count/(abs(diff_count)));
	}

	// pulse/second = (pulse/(ms * 10 ^-3))
	int32_t encoder_per_second = (int32_t) (diff_count/(dt_ms*1e-3));

	robot_base_motor->current_velocity = encoder_per_second;

	// Salvando o valor do count para proxima atualizacao de dados
	robot_base_motor->encoder_last_count = robot_base_motor->encoder_curret_count;
	robot_base_motor->last_time_call_ms = current_time_ms;
}

void set_velocity_motor_base_set_point(RobotBaseMotor *robot_base_motor, int32_t velocity)
{
	robot_base_motor->pid->set_point = velocity;
}

void compute_motor_base_output(RobotBaseMotor *robot_base_motor)
{
	read_velocity_motor_base(robot_base_motor);

}


