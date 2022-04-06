#include "TofSensor.hpp"

#include <Arduino.h>
#include <VL53L1X.h>

//Default address of this sensor
#define BASE_ADDRESS    0x2A

//FOV width - The sensor has a max FOV of 16, and a min of 4. 
//Larger FOV is more sensitive and less precise, smaller FOV is the opposite
#define FOV             16

//The total amount of time between the start of each reading
#define TIMEOUT_MS      100

//The amount of time that the sensor can use to actually read the distance
#define TIME_BUDGET_MS  50


VL53L1X sensors[SENSOR_COUNT];

//Pin that must be written LOW to disable the sensor. To enable the sensor 
//leave this pin floating, i.e. set to input.
const uint8_t enPins[] = {8, 7};

//Initialize the time of flight sensors, returns 0 on success
uint8_t initTofSensors(){
    //Disable all sensors
    for(int i = 0; i < SENSOR_COUNT; i++){
        pinMode(enPins[i], OUTPUT);
        digitalWrite(enPins[i], LOW);
    }

    for(int i = 0; i < SENSOR_COUNT; i++){
        VL53L1X *sensor = &sensors[i];
        //Enable one of the sensors
        digitalWrite(enPins[i], HIGH);
        //Wait for it to turn on
        delay(10);

        //Set timeout for sensor I2C reads
        sensor->setTimeout(TIMEOUT_MS);
        //Init and return if failed
        if(!sensor->init()){
            return 1;
        }
        Serial.println(i);

        //Set each sensor to have a unique address, we will just count up from
        //the default address 0x2A
        sensor->setAddress(BASE_ADDRESS + i);
        sensor->setROISize(FOV, FOV);
        sensor->setDistanceMode(VL53L1X::DistanceMode::Long);
        sensor->setMeasurementTimingBudget(TIME_BUDGET_MS);
    }

    return 0;
}

void resetTof(){
    for(int i = 0; i < SENSOR_COUNT; i++){
        digitalWrite(enPins[i], LOW);
    }
}

//Read the specified sensor, if the reading is 0, it probably failed
uint16_t readSensor(TOF_SENSOR sensorIdx){
    VL53L1X *sensor = &sensors[sensorIdx];
    //Start a non-blocking read
    sensor->readSingle(false);

    //Wait for either the data to be ready or for the reading to timeout
    while(!sensor->dataReady() && !sensor->timeoutOccurred());

    //Return the data
    return sensor->read(false);
}

