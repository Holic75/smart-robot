
#include "SmartRobot.h"


MeEncoderOnBoard Encoder_1(SLOT1), Encoder_2(SLOT2);



void isr_process_encoder1(void)
{
	if (digitalRead(Encoder_1.getPortB()) == 0) {
		Encoder_1.pulsePosMinus();
	}
	else {
		Encoder_1.pulsePosPlus();
	}
}

void isr_process_encoder2(void)
{
	if (digitalRead(Encoder_2.getPortB()) == 0) {
		Encoder_2.pulsePosMinus();
	}
	else {
		Encoder_2.pulsePosPlus();
	}
}



SerialCommunicator serial_com(Serial, 255, 5);
IMU imu(0x69);


SmartRobot smart_robot(9, 7, 6, 105, 85, 2700.0f, imu, serial_com, Encoder_1, Encoder_2);


void setup()
{
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	delay(5);
	Serial.begin(115200);
	Serial.write("Started");
	delay(5);


	attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
	attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

	
	//Set Pwm 8KHz
	TCCR1A = _BV(WGM10);
	TCCR1B = _BV(CS11) | _BV(WGM12);
	TCCR2A = _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS21);
	pinMode(13, OUTPUT);

	//initialize imu
	imu.initialize(MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_2, { 150, 80, -28 }, { 1360, 37, -3011 }, 8350);
	imu.set_accel_low_filter(50);
	
	smart_robot.startUp();
}



void loop()
{
	smart_robot.runMainLoop();

}
