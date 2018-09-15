

#ifndef SmartRobotMessages_h
#define SmartRobotMessages_h

#include <Arduino.h>

enum class SmartrobotMessageTypes { ChangeState = 1, DataQuery = 2};



struct ChangeStateMessage
{
	int motor_speed1;
	int motor_speed2;
	int vision_angle_ver;
	int vision_angle_hor;
	bool need_data;

	ChangeStateMessage() {};
	ChangeStateMessage(const uint8_t* message_bytes);

};

struct DataMessage
{
	const uint8_t message_type = 2;
	int light_sensor_left;
	int light_sensor_right;
	int ultrasonic_distance;
	float temperature;

	float position_angle;
	float position_x;
	float position_y;
	float total_distance;

};

#endif