#include <PID_v1.h>
#include <RunningAverage.h>
#include "TimerOne.h"
#include "source/TofSensor.cpp"
#include "source/ImuSensor.cpp"
#include "source/Heartbeat.cpp"

// #define SPEC_TESTING 4

// Sensors
#define FRONT_TOF SENSOR_1
#define LEFT_TOF SENSOR_2
#define IMU 1

// Physical dimentions
#define TILE_WIDTH    305
#define LEFT_GAP      110
#define FRONT_GAP     180

// Motor pins
#define MOTOR_RIGHT_FORWARD   11
#define MOTOR_RIGHT_BACKWARD  10
#define MOTOR_LEFT_BACKWARD   6
#define MOTOR_LEFT_FORWARD    5

// Motor duty cycles
#define PWM_MAX           255
#define FORWARD_RIGHT_DC  (PWM_MAX / 1)
#define FORWARD_LEFT_DC   (PWM_MAX / 1.02)
#define TURN_90_DC        (PWM_MAX / 2)

typedef enum{
    STOP = 0,
    FORWARD = 1,
    TURN_90 = 2,
    BACKWARD = 5
}MOVEMENT;


// Course path variables
const uint8_t stoppingTiles[] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2};
const uint8_t leftTiles[] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
uint8_t turnCount = 0;

// Angle calculation variables
imu_data_t gyro_data;
float current_angle, delta_angle, sample; 
float delta_time, prev_time, curr_time;

// Testing variables
RunningAverage ringBuf(50);

// PID variables
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,5,0,3, DIRECT);

long turnDetectionFreeze = 0;

void setup() {
  Serial.begin(115200);

  // Start heartbeat
	initHeartbeat();
	digitalWrite(A4, LOW);
	digitalWrite(A5, LOW);

  // Initialize the I2C
  Wire.begin();
  Wire.setWireTimeout(100000, true);
  Wire.setClock(50000);

  // Initialize the sensors
  if(initTofSensors()){
    Serial.println("e");
  }
  initImu();

  // Configure PID
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-51, 51);

  // Setup motors
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
}

void loop() {
	heartbeat();

  #ifdef SPEC_TESTING
  #if SPEC_TESTING == 1
    spec_test_straight();
  #elif SPEC_TESTING == 2
    spec_test_turn();
  #elif SPEC_TESTING == 3
    spec_test_trap();
  #else
    general_testing();
  #endif
  #endif

  #ifndef SPEC_TESTING
    // Dont turn when tilted into the traps
    if((readGyro().x >= 30 && turnCount < 4) || (readGyro().x >= 22 && turnCount >= 4)){
      turnDetectionFreeze = millis();
    }

    // Turn at the right distance unless going into a trap
    if (((millis()- turnDetectionFreeze > 450 && turnCount < 4) || (millis()- turnDetectionFreeze > 350 && turnCount >= 4)) && readSensor(FRONT_TOF) < TILE_WIDTH*stoppingTiles[turnCount] + FRONT_GAP) {
      // Stop in place
      motor_control(STOP);
      motor_control(BACKWARD);
      delay(100);
      motor_control(STOP);

      // Exit the program at the end 
      if (turnCount == 10) {
        exit(0);
      }

      // Turn
      rotate90degrees();
      turnDetectionFreeze = millis();
      turnCount++; 
    }

    // Course correct to follow a straight line
    align();
  #endif
}

void rotate90degrees() {
  motor_control(TURN_90);

  current_angle = 0;
  curr_time = micros();

  while (current_angle > -80) {   // Will turn about 90 deg, the IMU inaccuracy and momentum give about 10 degrees
    prev_time = curr_time;
    gyro_data = readGyro();
    delay(20);
    curr_time = micros();
    delta_time = curr_time - prev_time;
    delta_angle = ((gyro_data.z + 0.378)*delta_time)/1000000.0;
    current_angle += delta_angle;
  }

  motor_control(STOP);
}

void align() {
  // Compute PID based on TOF readings
  Input = (double)readSensor((TOF_SENSOR)(LEFT_TOF)) - (TILE_WIDTH*leftTiles[turnCount] + LEFT_GAP);
  myPID.Compute();

  // Use the result to change the motor power
  analogWrite(MOTOR_LEFT_FORWARD, 204 + Output);
  analogWrite(MOTOR_RIGHT_FORWARD, 204 - Output);
}

void motor_control(MOVEMENT movement) {
  switch(movement) {
    case(STOP):
      // Stop motors
      digitalWrite(MOTOR_LEFT_FORWARD, LOW);
      digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
      digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
      digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
      break;
    case(FORWARD):
      // Forward motors
      analogWrite(MOTOR_LEFT_FORWARD, FORWARD_LEFT_DC);
      analogWrite(MOTOR_RIGHT_FORWARD, FORWARD_RIGHT_DC);
      break;
    case(BACKWARD):
      // Forward motors
      analogWrite(MOTOR_LEFT_BACKWARD, FORWARD_LEFT_DC);
      analogWrite(MOTOR_RIGHT_BACKWARD, FORWARD_RIGHT_DC);
      break;
    case(TURN_90):
      // Turn motors right
      analogWrite(MOTOR_LEFT_FORWARD, TURN_90_DC);
      analogWrite(MOTOR_RIGHT_BACKWARD, TURN_90_DC);
      break;
  }
}

#ifdef SPEC_TESTING
// Straight line test
void spec_test_straight() {
  align();

  if (readSensor(FRONT_TOF) < TILE_WIDTH*stoppingTiles[turnCount] + FRONT_GAP) {
    motor_control(STOP);
    // rotate90degrees();
    delay(30000);
  }
}

// Turning test
void spec_test_turn() {
  rotate90degrees();
  delay(30000);
}

// Trap test
void spec_test_trap() {
  motor_control(FORWARD);
  delay(4000);
  motor_control(STOP);
  delay(5000);
}

// Get the max and min of a stream of readings
void general_testing(){
  gyro_data = readGyro();
  delay(20);
  ringBuf.addValue(readSensor(FRONT_TOF));

  Serial.print(":");
  Serial.print(ringBuf.getMax());
  Serial.print(" - ");
  Serial.println(ringBuf.getMin());
  // Serial.print(" --- ");
  // Serial.println(gyro_data.z, 5);
}
#endif
