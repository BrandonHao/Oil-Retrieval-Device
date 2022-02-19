#ifndef __TOF_SENSOR_HPP__
#define __TOF_SENSOR_HPP__

#include <inttypes.h>

//Note: For some reason blocking reads from the sensor will return 0 every
//other reading, idk why. For us the only advantage to blocking calls is 
//simplicity, so I'm not going to fix it right now. If we need it later I'll
//try to fix.
// - Brandon 

//Testing with only one sensor for now
typedef enum{
    SENSOR_1 = 0,
    //SENSOR_2,
    //SENSOR_3,
    SENSOR_COUNT
}TOF_SENSOR;

//Initialize the time of flight sensors, returns 0 on success
uint8_t initTofSensors();

//Read all sensors to an array, returns 0 on success
uint8_t readAllSensors(uint16_t data[3]);

//Read the specified sensor
uint16_t readSensor(TOF_SENSOR sensor);

#endif
