#include "TofSensor.cpp"
#include "ImuSensor.cpp"

#define FRONT_TOF SENSOR_1
#define LEFT_TOF SENSOR_2
#define RIGHT_TOF SENSOR_3
#define IMU 1
#define TILEWIDTH 305
// change after mechanical is done
#define TILEGAP 50

// number tiles in the stopping distance per turn
const uint16_t stoppingTiles = [0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2];
uint16_t turnCount = 0;

imu_data_t gyro_data; 
int16_t current_angle, delta_time, delta_angle, prev_time, curr_time, sample; 

void setup() {

}

void loop() {
  // TODO: start motors
  startMotors();

  // detecting corners to turn at
  if (readSensor(FRONT_TOF) < TILEWIDTH*stoppingTiles[turnCount] + TILEGAP) {
    // TODO: stop motors
    stopMotors();

    // exit the program at the end 
    if (turnCount == 10) {
      exit(0);
    }

    rotate90degrees();
    turnCount++; 
  }
  
  // course correct
  align();
}

void rotate90degrees() {
  // Serial.println("Setting starting angle to 0...");
  current_angle = 0;
  curr_time = millis();
  while (current_angle > -90) {
    prev_time = curr_time;
    curr_time = millis();
    gyro_data = readGyro(); 
    sample = data.z;
    delta_time = curr_time - prev_time;
    delta_angle = sample*delta_time/1000;
    current_angle += delta_angle;
    // Serial.println(current_angle);
    delay(25);
  }
}

bool align() {
  
}
