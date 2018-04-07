

#ifndef MakeBlockInit_h
#define MakeBlockInit_h

#include <Arduino.h>
#include <avr/wdt.h>
#include <MeAuriga.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"



//#define DEBUG_INFO
//#define DEBUG_INFO1

extern Servo servos[12];


typedef struct MeModule
{
	int16_t device;
	int16_t port;
	int16_t slot;
	int16_t pin;
	int16_t index;
	float values[3];
} MeModule;

union
{
	uint8_t byteVal[4];
	float floatVal;
	long longVal;
}val;

union
{
	uint8_t byteVal[8];
	double doubleVal;
}valDouble;

union
{
	uint8_t byteVal[2];
	int16_t shortVal;
}valShort;



#define MOVE_STOP       0x00
#define MOVE_FORWARD    0x01
#define MOVE_BACKWARD   0x02

#define BLUETOOTH_MODE                       0x00
#define AUTOMATIC_OBSTACLE_AVOIDANCE_MODE    0x01
#define BALANCED_MODE                        0x02
#define IR_REMOTE_MODE                       0x03
#define LINE_FOLLOW_MODE                     0x04
#define MAX_MODE                             0x05

#define POWER_PORT                           A4
#define BUZZER_PORT                          45
#define RGBLED_PORT                          44



extern String mVersion;

//////////////////////////////////////////////////////////////////////////////////////
#define PWM_MIN_OFFSET   5

#define VERSION                0
#define ULTRASONIC_SENSOR      1
#define TEMPERATURE_SENSOR     2
#define LIGHT_SENSOR           3
#define POTENTIONMETER         4
#define JOYSTICK               5
#define GYRO                   6
#define SOUND_SENSOR           7
#define RGBLED                 8
#define SEVSEG                 9
#define MOTOR                  10
#define SERVO                  11
#define ENCODER                12
#define IR                     13
#define PIRMOTION              15
#define INFRARED               16
#define LINEFOLLOWER           17
#define SHUTTER                20
#define LIMITSWITCH            21
#define BUTTON                 22
#define HUMITURE               23
#define FLAMESENSOR            24
#define GASSENSOR              25
#define COMPASS                26
#define TEMPERATURE_SENSOR_1   27
#define DIGITAL                30
#define ANALOG                 31
#define PWM                    32
#define SERVO_PIN              33
#define TONE                   34
#define BUTTON_INNER           35
#define ULTRASONIC_ARDUINO     36
#define PULSEIN                37
#define STEPPER                40
#define LEDMATRIX              41
#define TIMER                  50
#define TOUCH_SENSOR           51
#define JOYSTICK_MOVE          52
#define COMMON_COMMONCMD       60
										   //Secondary command
#define SET_STARTER_MODE     0x10
#define SET_AURIGA_MODE      0x11
#define SET_MEGAPI_MODE      0x12
#define GET_BATTERY_POWER    0x70
#define GET_AURIGA_MODE      0x71
#define GET_MEGAPI_MODE      0x72
#define ENCODER_BOARD          61
										   //Read type
#define ENCODER_BOARD_POS    0x01
#define ENCODER_BOARD_SPEED  0x02

#define ENCODER_PID_MOTION     62
										   //Secondary command
#define ENCODER_BOARD_POS_MOTION         0x01
#define ENCODER_BOARD_SPEED_MOTION       0x02
#define ENCODER_BOARD_PWM_MOTION         0x03
#define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
#define ENCODER_BOARD_CAR_POS_MOTION     0x05

#define GET 1
#define RUN 2
#define RESET 4
#define START 5

typedef struct
{
	double P, I, D;
	double Setpoint, Output, Integral, differential, last_error;
} PID;



#endif
