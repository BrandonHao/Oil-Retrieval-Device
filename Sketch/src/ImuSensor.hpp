#ifndef __IMU_SENSOR_HPP__
#define __IMU_SENSOR_HPP__

#include <inttypes.h>

typedef struct{
    float x;
    float y;
    float z;
}imu_data_t;

//Returns 0 on success
uint8_t initImu();

//Get the current acceleration in x, y, and z components
imu_data_t readAcceleration();

//Get the current gyroscope data in x, y, and z components
imu_data_t readGyro();

#endif