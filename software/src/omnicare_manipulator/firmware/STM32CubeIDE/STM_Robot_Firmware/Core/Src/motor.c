#include "motor.h"

void init_motor_robot_base(RobotBaseMotor *robot_base_motor,
						   float kp, float ki, float kd,
						   double max_output, double min_output,
						   double max_integral_error, uint32_t last_time_call_ms)
{
	robot_base_motor->set_point_velocity = 0.0;
	robot_base_motor->current_velocity    = 0.0;

	robot_base_motor->encoder_last_count   = 0;
	robot_base_motor->encoder_curret_count = 0;

	initPID(robot_base_motor->pid, kp, ki, kd,
			max_output, min_output,
			max_integral_error, last_time_call_ms);
}




