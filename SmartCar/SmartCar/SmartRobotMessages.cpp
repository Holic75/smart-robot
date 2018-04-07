#include "SmartRobotMessages.h"



ChangeStateMessage::ChangeStateMessage(const uint8_t * message_bytes)
{
	//int16 - motor_speed1, int16 - motor_speed2, int16 -direction_angle, int16 - vision_angle, byte - need to send info - 9 bytes
	
	motor_speed1 = *((const int16_t*)message_bytes);
	motor_speed2 = *((const int16_t*)(message_bytes+ sizeof(int16_t)));
	direction_angle = *((const int16_t*)(message_bytes + 2*sizeof(int16_t)));
	vision_angle = *((const int16_t*)(message_bytes + 3*sizeof(int16_t)));
	need_data = *((const bool*)(message_bytes + 4*sizeof(int16_t)));

}
