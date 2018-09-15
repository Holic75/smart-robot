

#ifndef SmartRobot_h
#define SmartRobot_h

#include "MakeBlockInit.h"
#include "SmartRobotMessages.h"

extern void(*resetFunc) (void); //declare reset function @ address 0

class SerialCommunicator
{

	private:

		enum class CommunicatorReceptionState {AwaitingMessage, GettingMessageLength, ReceivingMessage, MessageReceived   };

		const uint8_t message_start_byte_ = 255;
		const uint8_t max_message_length_ = 255;
		uint8_t reception_buffer_[256];

		uint8_t expected_message_length_;
		uint8_t bytes_received_;
		CommunicatorReceptionState reception_state_ = CommunicatorReceptionState::AwaitingMessage;

		HardwareSerial& channel_;

		bool seekForIncomingMessage();
		bool getIncomingMessageLength();
		bool receiveIncomingMessage();
		
	public:
		SerialCommunicator(HardwareSerial& channel, uint8_t message_start_byte, unsigned long timeout)
			:channel_(channel), message_start_byte_(message_start_byte) 
		{
			reset(); channel_.setTimeout(timeout);
		};

		void reset();
		void sendMessage(const uint8_t* buffer, uint8_t length);
		const uint8_t* getReceivedMessage() const;
		uint8_t getReceivedMessageLength() const;
		void runReceptionLoop();
		

};


struct RgbColor
{
	uint8_t r_, g_, b_;
	
	RgbColor(uint8_t lum) :r_(lum), g_(lum), b_(lum) {};
	RgbColor(uint8_t r, uint8_t g, uint8_t b) : r_(r), g_(g), b_(b) {};
	RgbColor() : r_(0), g_(0), b_(0) {};
};


struct PositionData
{
	float x, y, z;
	float yaw, pitch, roll;
	float total_distance;

	PositionData():x(0), y(0), z(0), yaw(0), pitch(0), roll(0), total_distance(0) {};
};

class IMU
{
	private:
		// MPU control/status vars
		  // set true if DMP init was successful
		bool dmpReady_ = false;
		uint8_t mpuIntStatus_;   // holds actual interrupt status byte from MPU
		uint8_t devStatus_;      // return status after each device operation (0 = success, !0 = error)
		uint16_t packetSize_;    // expected DMP packet size (default is 42 bytes)
		uint16_t fifoCount_;     // count of all bytes currently in FIFO
		uint8_t fifoBuffer[64]; // FIFO storage buffer

		int16_t g_reference_value_;

		MPU6050 mpu_;
		int16_t accel_low_filter_ = 0;

		// orientation/motion vars
		Quaternion q_;           // [w, x, y, z]         quaternion container
		VectorInt16 aa_;         // [x, y, z]            accel sensor measurements
		VectorInt16 aaReal_;     // [x, y, z]            gravity-free accel sensor measurements
		VectorInt16 aaWorld_;    // [x, y, z]            world-frame accel sensor measurements
		VectorFloat gravity_;    // [x, y, z]            gravity vector
		float euler_[3];         // [psi, theta, phi]    Euler angle container
		float ypr_[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

	public:
		IMU(int i2c_address) :mpu_(i2c_address), dmpReady_(false) {};
		void initialize(uint8_t gyro_sensetivity, uint8_t accel_sensetivity, VectorInt16 gyro_offset, VectorInt16 accel_offset, int16_t g_reference_value = 8192);
		void set_accel_low_filter(int16_t accel_low_filter) { accel_low_filter_ = accel_low_filter; };
		void update();

		const Quaternion&  getQuaternion() { return q_; };
		float getYaw() const { return ypr_[0]; };
		float getPitch() const { return ypr_[1]; };
		float getRoll() const { return ypr_[2]; };
	
};





class SmartRobot
{
	private:
		Servo servo_direction_;
		Servo servo_vision_;
		MePort servo_port_;

		MeRGBLed top_leds_;
		MeRGBLed front_leds_;

		MeEncoderOnBoard& Encoder_1_;
		MeEncoderOnBoard& Encoder_2_;

		MeUltrasonicSensor ultrasonic_sensor_;
		MeOnBoardTemp temperature_onboard_;
		MeLightSensor lightsensor_left_;
		MeLightSensor lightsensor_right_;
		MeBuzzer  buzzer_;

		

		const uint8_t MAX_SPEED_ = 200;
		const uint8_t CENTER_SERVO_DIRECTION_;
		const uint8_t CENTER_SERVO_VISION_;

		const int8_t MAX_SERVO_DIRECTION_ANGLE_ = 40;
		const int8_t MAX_SERVO_VISION_ANGLE_ = 75;

		long last_encoder_value_1_ = 0;
		long last_encoder_value_2_ = 0;

		float ENCODER_SCALE_ = 2700.0f;

		IMU& imu_;
		SerialCommunicator& com_;

		PositionData position;

		void updatePostion();
		void runCommunicationLoop();

		void OnChangeStateMessageReceive(const uint8_t*);

		DataMessage packData();

		

	public:
		SmartRobot(uint8_t servo_port, uint8_t ultrasonic_sensor_port, uint8_t front_led_port, uint8_t center_servo_direction,
			uint8_t  center_servo_vision, float encoder_scale, IMU& imu, SerialCommunicator& com, MeEncoderOnBoard& Encoder_1, MeEncoderOnBoard& Encoder2_);

		void setTopLedsOn(const RgbColor& rgb_color, uint8_t start_led_id = 0, uint8_t n_leds = 12);
		void setTopLedOn(const RgbColor& rgb_color, uint8_t led_id = 1);

		void setRearMotorsSpeed(int16_t motorRatio, int16_t motorSpeed);

		void setFrontWheelsAngle(int8_t angle);
		void setVisionAngle(int8_t angle);

		void showTopLedsBusy(long duration);
		

		void stop();

		void startUp();

		void runMainLoop();

};

#endif