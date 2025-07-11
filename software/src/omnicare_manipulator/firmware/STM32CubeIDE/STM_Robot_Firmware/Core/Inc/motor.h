#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "pid.h"

typedef struct
{
	double set_point_velocity;  // encoders tick / second
	double current_velocity;  	// encoders tick / second

	uint32_t encoder_last_count;
	uint32_t encoder_curret_count;

	PID *pid;
} RobotBaseMotor;

void init_motor_robot_base(RobotBaseMotor *robot_base_motor,
						   float kp, float ki, float kd,
						   double max_output, double min_output,
						   double max_integral_error, uint32_t last_time_call_ms);

#endif /* INC_MOTOR_H_ */
