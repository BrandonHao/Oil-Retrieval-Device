#include "src/TofSensor.hpp"
#include "src/ImuSensor.hpp"

#include <Wire.h>

int16_t current_angle, delta_time, delta_angle, prev_time, curr_time, sample; 

void setup()
{
	Serial.begin(115200);
	//Initialize the I2C
	Wire.begin();

	//Set the clock to 400KHz
	Wire.setClock(400000);
	//Init the sensors
	if(initTofSensors()){
		Serial.println("e");
	}

	initImu();
  Serial.println("Setting starting angle to 0...");
  current_angle = 0;
  curr_time = millis();

}

void loop()
{
	//Read data then print
//	imu_data_t data = readAcceleration();
//	Serial.print("(");
//	Serial.print(data.x);
//	Serial.print(", ");
//	Serial.print(data.y);
//	Serial.print(", ");
//	Serial.print(data.z);
//	Serial.println(")");
//	delay(20);

  Serial.println("Reading TOFs:");
	for(int i = 0; i < SENSOR_COUNT; i++){
		uint16_t data = readSensor((TOF_SENSOR)(SENSOR_1 + i));
		Serial.print(data);
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
  delta_angle = sample*delta_time/1000;
  current_angle += delta_angle;
  Serial.println(current_angle);
  Serial.println(" ");
	delay(50);
}
