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
	initTofSensors();
}

void loop()
{
	//Read data then print
	uint16_t data = readSensor(SENSOR_1);
	Serial.println(data);
}
