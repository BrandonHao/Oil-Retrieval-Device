#include "src/Log.hpp"
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
	delay(1000);
}
