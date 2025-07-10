#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

typedef struct
{
	int32_t set_point_velocity; // definir qual a unidade de medida dessa variavel no ros2 control
	int32_t atual_velocity;  	// definir qual a unidade de medida dessa variavel no ros2 control

	uint32_t encoder_last_count;
	uint32_t encoder_actual_count;


} MotorRobotBase;




#endif /* INC_MOTOR_H_ */
