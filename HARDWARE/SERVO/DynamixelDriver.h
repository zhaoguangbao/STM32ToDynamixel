#ifndef _DYNAMIXEL_DRIVER_H_
#define _DYNAMIXEL_DRIVER_H_


enum CONTROL_ITEM{
	model_number						=						0,
	version_of_firmware			=						2,
	dynamixel_ID						=						3,
	dynamixel_baudrate			=						4,
	return_delay_time				=						5,
	CW_angle_limit					=						6,
	CCW_angle_limit					=						8,
	drive_mode							=						10,
	max_temperature_limit		=						11,
	min_voltage_limit				=						12,
	max_voltage_limit				=						13,
	max_torque							=						14,
	status_return_level			=						16,
	alarm_LED								=						17,
	alarm_shutdown					=						18,
	multi_turn_offset				=						20,
	resolution_dividor			=						22,
	torque_enable						=						24,
	dynamixel_LED						=						25,
	position_d_gain					=						26,
	position_i_gain					=						27,
	position_p_gain					=						28,
	goal_position						=						30,
	goal_velocity						=						32,
	goal_torque							=						34,
	present_position				=						36,
	present_velocity				=						38,
	present_load						=						40,
	present_voltage					=						42,
	present_temperature			=						43,
	registered_instruction	=						44,
	is_moving								=						46,
	EEPROM_lock							=						47,
	punch										=						48,
	current_consumption			=						68,
	torque_control_mode			=						70,
	torque_control_goal			=						71,
	goal_acceleration				=						73
};


enum CONTROL_ITEM_SZ{
	model_number_sz					=						2,
	version_of_firmware_sz	=						1,
	dynamixel_ID_sz					=						1,
	dynamixel_baudrate_sz		=						1,
	return_delay_time_sz		=						1,
	CW_angle_limit_sz				=						2,
	CCW_angle_limit_sz			=						2,
	drive_mode_sz						=						1,
	max_temperature_limit_sz=						1,
	min_voltage_limit_sz		=						1,
	max_voltage_limit_sz		=						1,
	max_torque_sz						=						2,
	status_return_level_sz	=						1,
	alarm_LED_sz						=						1,
	alarm_shutdown_sz				=						1,
	multi_turn_offset_sz		=						2,
	resolution_dividor_sz		=						1,
	torque_enable_sz				=						1,
	dynamixel_LED_sz				=						1,
	position_d_gain_sz			=						1,
	position_i_gain_sz			=						1,
	position_p_gain_sz			=						1,
	goal_position_sz				=						2,
	goal_velocity_sz				=						2,
	goal_torque_sz					=						2,
	present_position_sz			=						2,
	present_velocity_sz			=						2,
	present_load_sz					=						2,
	present_voltage_sz			=						1,
	present_temperature_sz	=						1,
	registered_instruction_sz=					1,
	is_moving_sz						=						1,
	EEPROM_lock_sz					=						1,
	punch_sz								=						2,
	current_consumption_sz	=						2,
	torque_control_mode_sz	=						1,
	torque_control_goal_sz	=						2,
	goal_acceleration_sz		=						1
};

// operating mode
enum OPERATING_MODE{
	CURRENT_CONTROL_MODE=0,
	VELOCITY_CONTROL_MODE=1,
	POSITION_CONTROL_MODE=3,
	EXTENDED_POSITION_CONTROL_MODE=4,
	CURRENT_BASED_POSITION_CONTROL_MODE=5,
	PWM_CONTROL_MODE=16,
	TORQUE_CONTROL_MODE=100,
	MULTI_TURN_MODE=101
};

					




#endif
