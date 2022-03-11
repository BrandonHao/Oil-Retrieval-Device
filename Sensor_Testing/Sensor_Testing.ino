#include "src/TofSensor.hpp"
#include "src/ImuSensor.hpp"

#include <Wire.h>


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

}

void loop()
{
	//Read data then print
	imu_data_t data = readAcceleration();
	Serial.print("(");
	Serial.print(data.x);
	Serial.print(", ");
	Serial.print(data.y);
	Serial.print(", ");
	Serial.print(data.z);
	Serial.println(")");
	delay(20);

	for(int i = 0; i < SENSOR_COUNT; i++){
		uint16_t data = readSensor((TOF_SENSOR)(SENSOR_1 + i));
		Serial.print(data);
		Serial.print("mm ");
		delay(20);
	}
	Serial.println();

	delay(1000);
}
