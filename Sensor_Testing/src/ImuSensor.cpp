#include "ImuSensor.hpp"

#include <Wire.h>
#include "SparkFunLSM6DSO.h"

LSM6DSO imu;

uint8_t initImu(){
    if(imu.begin()){
        return 1;
    }

    if(imu.initialize(BASIC_SETTINGS)){
        return 1;
    }

    return 0;
}

imu_data_t readAcceleration(){
    return imu_data_t { imu.readFloatAccelX(),
                        imu.readFloatAccelY(),
                        imu.readFloatAccelZ()};
}

imu_data_t readGyro(){
    return imu_data_t { imu.readFloatGyroX(),
                        imu.readFloatGyroY(),
                        imu.readFloatGyroZ()};
}
