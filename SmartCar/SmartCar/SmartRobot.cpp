#include "SmartRobot.h"

SmartRobot::SmartRobot(uint8_t servo_port, uint8_t ultrasonic_sensor_port, uint8_t front_led_port,
	uint8_t center_servo_vision_hor, uint8_t  center_servo_vision_ver, float encoder_scale, IMU& imu, SerialCommunicator& com, 
	MeEncoderOnBoard& Encdoder_1, MeEncoderOnBoard& Encoder_2)
	: Encoder_1_(Encdoder_1), Encoder_2_(Encoder_2),
	CENTER_SERVO_VISION_HOR_(center_servo_vision_hor), CENTER_SERVO_VISION_VER_(center_servo_vision_ver),
	ultrasonic_sensor_(ultrasonic_sensor_port),
	temperature_onboard_(PORT_13),
	lightsensor_right_(11),
	lightsensor_left_(12),
	servo_port_(servo_port),
	front_leds_(front_led_port, 4),
	ENCODER_SCALE_(encoder_scale),
	imu_(imu),
	com_(com)
{
	servo_vision_hor_ = ::servos[0];
	servo_vision_ver_ = ::servos[1];
	top_leds_.setpin(RGBLED_PORT);
	buzzer_.setpin(BUZZER_PORT);
}

void SmartRobot::setTopLedsOn(const RgbColor& rgb_color, uint8_t start_led_id, uint8_t n_leds)
{
	for (uint8_t i = 0; i<12; i++) {
		top_leds_.setColor(i, 0, 0, 0);
	}

	for (uint8_t i = 0; i<n_leds; i++) {
		top_leds_.setColor((i + start_led_id) % 12 + 1, rgb_color.r_, rgb_color.g_, rgb_color.b_);
	}
	top_leds_.show();
}

void SmartRobot::setTopLedOn(const RgbColor & rgb_color, uint8_t led_id)
{
	top_leds_.setColor(1, rgb_color.r_, rgb_color.g_, rgb_color.b_);
	top_leds_.show();
}

void SmartRobot::setRearMotorsSpeed(int16_t motorSpeed1, int16_t motorSpeed2)
{
	motorSpeed1 = constrain(motorSpeed1, -100, 100);
	motorSpeed2 = constrain(motorSpeed2, -100, 100);
	motorSpeed1 = map(motorSpeed1, -100, 100, -MAX_SPEED_, MAX_SPEED_);
	motorSpeed2 = map(motorSpeed2, -100, 100, -MAX_SPEED_, MAX_SPEED_);

	Encoder_1_.setMotorPwm(-motorSpeed1);
	Encoder_2_.setMotorPwm(motorSpeed2);
}

void SmartRobot::setFrontWheelsAngle(int8_t angle)
{
	//angle = constrain(angle, -MAX_SERVO_DIRECTION_ANGLE_, MAX_SERVO_DIRECTION_ANGLE_);
	//servo_direction_.write(CENTER_SERVO_DIRECTION_ + angle);
}

void SmartRobot::setVisionAngle(int8_t angle_ver, int8_t angle_hor)
{
	angle_ver = constrain(angle_ver, -MAX_SERVO_VISION_ANGLE_VER_, MAX_SERVO_VISION_ANGLE_VER_);
	servo_vision_ver_.write(CENTER_SERVO_VISION_VER_ + angle_ver);
	angle_hor = constrain(angle_hor, -MAX_SERVO_VISION_ANGLE_HOR_, MAX_SERVO_VISION_ANGLE_HOR_);
	servo_vision_hor_.write(CENTER_SERVO_VISION_HOR_ + angle_hor);

}

void SmartRobot::showTopLedsBusy(long duration)
{
	const long cycle_duration = 25;

	for (long i = 0; i<duration/cycle_duration; i++) {
		delay(cycle_duration);
		setTopLedsOn(10, i % 12, 4);
	}
	setTopLedsOn(0);
}

void SmartRobot::stop()
{
	Encoder_1_.setMotorPwm(0);
	Encoder_2_.setMotorPwm(0);
}

void SmartRobot::updatePostion()
{
	long p1 = Encoder_1_.getPulsePos();
	long p2 = Encoder_2_.getPulsePos();
	long dp1 = p1 - last_encoder_value_1_;
	long dp2 = p2 - last_encoder_value_2_;
	VectorFloat dr((dp2 - dp1) / 2.0f / ENCODER_SCALE_, 0.0f, 0.0f);
	imu_.update();
	dr.rotate(&imu_.getQuaternion());

	position.x += dr.x;
	position.y += dr.y;
	position.z += dr.z;
	position.total_distance += dr.getMagnitude();
	position.yaw = imu_.getYaw();
	position.pitch = imu_.getPitch();
	position.roll = imu_.getRoll();

	last_encoder_value_1_ = p1;
	last_encoder_value_2_ = p2;

}

void SmartRobot::runCommunicationLoop()
{
	com_.runReceptionLoop();

	while (com_.getReceivedMessageLength())
	{
		const uint8_t* received_message_bytes = com_.getReceivedMessage();

		if (received_message_bytes[0] == uint8_t(SmartrobotMessageTypes::ChangeState)) {

			OnChangeStateMessageReceive(received_message_bytes+1);
		}

		com_.reset();
		com_.runReceptionLoop();
	}
}

void SmartRobot::OnChangeStateMessageReceive(const uint8_t * message_bytes)
{
	ChangeStateMessage state_message(message_bytes);
	//setFrontWheelsAngle(state_message.direction_angle);
	setVisionAngle(state_message.vision_angle_ver, state_message.vision_angle_hor);
	setRearMotorsSpeed(state_message.motor_speed1, state_message.motor_speed2);

	if (state_message.need_data) {
		
		DataMessage  data_message(packData());
		com_.sendMessage((uint8_t*)&data_message, sizeof(DataMessage));
	}
}

DataMessage SmartRobot::packData()
{
	DataMessage data_message;
	data_message.light_sensor_left = lightsensor_left_.read();
	data_message.light_sensor_right = lightsensor_right_.read();
	data_message.temperature = temperature_onboard_.readValue();
	data_message.ultrasonic_distance = ultrasonic_sensor_.distanceCm();
	data_message.position_angle = position.yaw;
	data_message.position_x = position.x;
	data_message.position_y = position.y;
	data_message.total_distance = position.total_distance;

	return data_message;
}

void SmartRobot::startUp()
{

	servo_vision_ver_.attach(servo_port_.pin2());
	servo_vision_hor_.attach(servo_port_.pin1());

	stop();
	buzzer_.tone(500, 200);

	front_leds_.setColor(0, 0, 0, 0);
	front_leds_.show();

	//check camera servos
	setVisionAngle(MAX_SERVO_VISION_ANGLE_VER_, 0);
	showTopLedsBusy(500);
	setVisionAngle(-MAX_SERVO_VISION_ANGLE_VER_, 0);
	showTopLedsBusy(500);
	setVisionAngle(0, 0);
	showTopLedsBusy(500);
	setVisionAngle(0, MAX_SERVO_VISION_ANGLE_HOR_);
	showTopLedsBusy(500);
	setVisionAngle(0, -MAX_SERVO_VISION_ANGLE_HOR_);
	showTopLedsBusy(500);
	setVisionAngle(0, 0);
}



void SmartRobot::runMainLoop()
{
	updatePostion();
	runCommunicationLoop();
	if (lightsensor_left_.read() < 100 && lightsensor_right_.read() < 100)
	{
		front_leds_.setColor(0, 100, 100, 100);
		front_leds_.show();
	}
	else
	{
		front_leds_.setColor(0, 0, 0, 0);
		front_leds_.show();
	}
}








///////////////////////Accelerometer-Gyro////////////////////////////////////////////////
void IMU::initialize(uint8_t gyro_sensetivity, uint8_t accel_sensetivity, VectorInt16 gyro_offset, VectorInt16 accel_offset, int16_t g_reference_value)
{
	mpu_.initialize();
	devStatus_ = mpu_.dmpInitialize(gyro_sensetivity, accel_sensetivity);
	mpu_.setXGyroOffset(gyro_offset.x);
	mpu_.setYGyroOffset(gyro_offset.y);
	mpu_.setZGyroOffset(gyro_offset.z);
	mpu_.setZAccelOffset(accel_offset.x);
	mpu_.setYAccelOffset(accel_offset.y);
	mpu_.setXAccelOffset(accel_offset.z);

	mpu_.setDMPEnabled(true);
	mpu_.setDLPFMode(5);
	mpuIntStatus_ = mpu_.getIntStatus();

	dmpReady_ = true;
	packetSize_ = mpu_.dmpGetFIFOPacketSize();
	mpu_.g_reference_value = g_reference_value_;

}

void IMU::update()
{
	fifoCount_ = mpu_.getFIFOCount();

	// wait for mpu_ extra packet(s) available
	while (fifoCount_ < packetSize_) {
		fifoCount_ = mpu_.getFIFOCount();
	}

	mpuIntStatus_ = mpu_.getIntStatus();

	VectorFloat a_xyz = { 0,0,0 }; //compu_te average acceleration over all values

	if ((mpuIntStatus_ & 0x10) || fifoCount_ == 1024) {
		// reset so we can continue cleanly
		mpu_.resetFIFO();
		//Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus_ & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount_ < packetSize_) fifoCount_ = mpu_.getFIFOCount();

		// read a packet from FIFO
		mpu_.getFIFOBytes(fifoBuffer, packetSize_);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount_ -= packetSize_;


		mpu_.dmpGetQuaternion(&q_, fifoBuffer);
		mpu_.dmpGetGravity(&gravity_, &q_);
		mpu_.dmpGetYawPitchRoll(ypr_, &q_, &gravity_);
		mpu_.dmpGetAccel(&aa_, fifoBuffer);
		mpu_.dmpGetGravity(&gravity_, &q_);
		mpu_.dmpGetLinearAccel(&aaReal_, &aa_, &gravity_);
		mpu_.dmpGetLinearAccelInWorld(&aaWorld_, &aaReal_, &q_);
		aaWorld_.filterLowValues(accel_low_filter_);
		a_xyz += aaWorld_;
	}
}


////////////////Serial Communciator/////////////////////////
//////////////////////////////////////////////////////

bool SerialCommunicator::seekForIncomingMessage()
{
	while (channel_.available()) {

		if (channel_.read() == message_start_byte_) {
			reception_state_ = CommunicatorReceptionState::GettingMessageLength;
			return true;
		}
	}

	return false;
}

bool SerialCommunicator::getIncomingMessageLength()
{
	if (channel_.available()) {

		expected_message_length_ = channel_.read();
		bytes_received_ = 0;
		reception_state_ = CommunicatorReceptionState::ReceivingMessage;
		return true;
	}

	return false;
}

bool SerialCommunicator::receiveIncomingMessage()
{
	int bytes_to_receive = expected_message_length_ - bytes_received_;

	if (channel_.available() && bytes_to_receive > 0) {
		
		bytes_received_ += channel_.readBytes(reception_buffer_ + bytes_received_, bytes_to_receive);
	}

	if (bytes_received_ >= expected_message_length_) {

		reception_state_ = CommunicatorReceptionState::MessageReceived;
	}

	return false;
}

void SerialCommunicator::reset()
{
	expected_message_length_ = 0;
	bytes_received_ = 0;
	reception_state_ = CommunicatorReceptionState::AwaitingMessage;
}

void SerialCommunicator::sendMessage(const uint8_t * buffer, uint8_t length)
{
	channel_.write(message_start_byte_);
	channel_.write(length);
	channel_.write(buffer, length);
	channel_.flush();
}

const uint8_t * SerialCommunicator::getReceivedMessage() const
{
	if (reception_state_ != CommunicatorReceptionState::MessageReceived) {
		return nullptr;
	}
	else {
		return reception_buffer_;
	}
}

uint8_t SerialCommunicator::getReceivedMessageLength() const
{
	if (reception_state_!= CommunicatorReceptionState::MessageReceived) {
		return 0;
	}
	else {
		return expected_message_length_;
	}
}

void SerialCommunicator::runReceptionLoop()
{
	bool res = true;
	while (res) {

		switch (reception_state_) {
		case CommunicatorReceptionState::AwaitingMessage:
			res = seekForIncomingMessage();
			break;
		case CommunicatorReceptionState::GettingMessageLength:
			res = getIncomingMessageLength();
			break;
		case CommunicatorReceptionState::ReceivingMessage:
			res = receiveIncomingMessage();
			break;
		case CommunicatorReceptionState::MessageReceived:
			res = false;
			break;
		}

	}
}
