#include "src/TofSensor.hpp"
#include "src/ImuSensor.hpp"
#include "src/HeartBeat.hpp"

#include <Wire.h>

int16_t current_angle, delta_time, delta_angle, prev_time, curr_time, sample;
uint8_t ledState;

void setup()
{
	Serial.begin(115200);

	initHeartbeat();

	digitalWrite(A4, LOW);
	digitalWrite(A5, LOW);

	//Initialize the I2C
	Wire.begin();

	//Set the clock to 400KHz
	Wire.setClock(50000);
	//Init the sensors
	if (initTofSensors()) {
		Serial.println("e");
	}

	initImu();
	Serial.print("Setting starting angle to 0...");
	current_angle = 0;
	curr_time = millis();
	pinMode(11, OUTPUT);
	pinMode(10, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(5, OUTPUT);
	analogWrite(11, 200);
	digitalWrite(10, LOW);
	analogWrite(6, 200);
	digitalWrite(5, LOW);
}

void loop()
{
	heartbeat();
	//Read data then print
	// imu_data_t data = readAcceleration();
	// Serial.print("(");
	// Serial.print(data.x);
	// Serial.print(", ");
	// Serial.print(data.y);
	// Serial.print(", ");
	// Serial.print(data.z);
	// Serial.println(")");
	// delay(20);

	Serial.println("Reading TOFs:");
	for (int i = 0; i < SENSOR_COUNT; i++) {
		uint16_t data2 = readSensor((TOF_SENSOR)(SENSOR_1 + i));
		Serial.print(data2);
		Serial.print("mm ");
		delay(20);
	}
	Serial.println();

	Serial.println("Reading angle:");
	prev_time = curr_time;
	curr_time = millis();
	imu_data_t data = readGyro();
	sample = data.z;
	delta_time = curr_time - prev_time;
	delta_angle = sample * delta_time / 1000;
	current_angle += delta_angle;
	Serial.println(current_angle);
	Serial.println(" ");
	delay(50);
	
}
