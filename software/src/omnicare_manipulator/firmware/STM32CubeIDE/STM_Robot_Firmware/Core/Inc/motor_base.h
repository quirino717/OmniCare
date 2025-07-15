#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "pid.h"

typedef struct
{
	double current_velocity;  	// encoders tick / second

	uint32_t last_time_call_ms;

	uint32_t encoder_last_count;
	uint32_t encoder_curret_count;

	PID *pid;
} RobotBaseMotor;

void init_motor_robot_base(RobotBaseMotor *robot_base_motor,
						   float kp, float ki, float kd,
						   double max_output, double min_output,
						   double max_integral_error);
void read_velocity_motor_base(RobotBaseMotor *robot_base_motor);
void set_velocity_motor_base_set_point(RobotBaseMotor *robot_base_motor, int32_t velocity);
void compute_motor_base_output(RobotBaseMotor *robot_base_motor);






#endif /* INC_MOTOR_H_ */
