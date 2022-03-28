#include "source/TofSensor.cpp"
#include "source/ImuSensor.cpp"
#include "source/Heartbeat.cpp"

#define SPEC_TESTING 1

#define FRONT_TOF SENSOR_1
#define LEFT_TOF SENSOR_3
#define RIGHT_TOF SENSOR_2
#define IMU 1
#define TILEWIDTH 305
// change after mechanical is done
#define TILEGAP 120  //Too close

// Motor pins
#define MOTOR_RIGHT_FORWARD   11
#define MOTOR_RIGHT_BACKWARD  10
#define MOTOR_LEFT_BACKWARD   6
#define MOTOR_LEFT_FORWARD    5

// Motor duty cycles
#define PWM_MAX     255
#define FORWARD_DC  (PWM_MAX / 1)
#define TURN_90_DC  (PWM_MAX / 4)
#define PULSE_DC    (PWM_MAX / 1.7)

typedef enum{
    STOP = 0,
    FORWARD = 1,
    TURN_90 = 2,
    PULSE_RIGHT = 3,
    PULSE_LEFT = 4
}MOVEMENT;

// number tiles in the stopping distance per turn
const uint8_t stoppingTiles[] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2};
const uint8_t leftTiles[] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
uint8_t turnCount = 0;

imu_data_t gyro_data;
float current_angle, delta_angle, sample; 
float delta_time, prev_time, curr_time;

uint16_t left_dist, right_dist;

void setup() {

  Serial.begin(115200);

	initHeartbeat();

	digitalWrite(A4, LOW);
	digitalWrite(A5, LOW);

  //Initialize the I2C
  Wire.begin();

  //Set the clock to 400KHz
  Wire.setClock(400000);
  //Init the sensors
  if(initTofSensors()){
    Serial.println("e");
  }

  initImu();

  //Motor setup
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
  #else
    spec_test_trap();
  #endif
  #endif

  #ifndef SPEC_TESTING

    // detecting corners to turn at
    if (readSensor(FRONT_TOF) < TILEWIDTH*stoppingTiles[turnCount] + TILEGAP) {
      motor_control(STOP);

      // exit the program at the end 
      if (turnCount == 10) {
        exit(0);
      }

      rotate90degrees();
      turnCount++; 
    }

    // course correct
    align();
  #endif
}

void rotate90degrees() {
  motor_control(TURN_90);

  Serial.println("Setting starting angle to 0...");
  current_angle = 0;
  curr_time = micros();
  while (current_angle > -85) {   //IDK WHATS HAPPENING HERE NO MAG & BUMPY WHEELE GOT HANDS
    prev_time = curr_time;
    gyro_data = readGyro();
    delay(25);
    curr_time = micros();
    delta_time = curr_time - prev_time;
    delta_angle = ((gyro_data.z + 0.378)*delta_time)/1000000;
    current_angle += delta_angle;
    // Serial.print(current_angle, 6);
    // Serial.print(" -- ");
    // Serial.print(delta_time);
    // Serial.print(" --- ");
    // Serial.println(gyro_data.z, 5);
  }

  motor_control(STOP);
}

void align() {
  // distance to left wall
  left_dist = readSensor((TOF_SENSOR)(LEFT_TOF));
  
  // if we're too far from the left wall, pulse the left motor off
  if (left_dist > 115) {//- (TILEWIDTH*leftTiles[turnCount] + TILEGAP) > 15) {
    motor_control(PULSE_LEFT);
  }
  // if we're too close to the left wall, pulse the right motor off
  else if (left_dist < 115) {//- (TILEWIDTH*leftTiles[turnCount] + TILEGAP) < -15) {
    motor_control(PULSE_RIGHT);
  }
  // else move forward
  else {
    motor_control(FORWARD);
  }
}

void motor_control(MOVEMENT movement) {
  switch(movement) {
    case(STOP):
      // Stop motors
      Serial.println("Stopping");
      digitalWrite(MOTOR_LEFT_FORWARD, LOW);
      digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
      digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
      digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
      break;
    case(FORWARD):
      // Forward motors
      Serial.println("Forward");
      analogWrite(MOTOR_LEFT_FORWARD, FORWARD_DC);
      analogWrite(MOTOR_RIGHT_FORWARD, FORWARD_DC);
      break;
    case(TURN_90):
      // Turn motors right
      Serial.println("Turning 90 degrees");
      analogWrite(MOTOR_LEFT_FORWARD, TURN_90_DC);
      analogWrite(MOTOR_RIGHT_FORWARD, TURN_90_DC);
      break;
    case(PULSE_LEFT):    //these pulse directions could be wrong
      // Pulse motors right
      Serial.println("Adjusting right");
      analogWrite(MOTOR_LEFT_FORWARD, PULSE_DC); //Might need to adjust based on rotation
      analogWrite(MOTOR_RIGHT_FORWARD, FORWARD_DC);
      break;
    case(PULSE_RIGHT):
      // Pulse motors left
      Serial.println("Adjusting left");
      analogWrite(MOTOR_LEFT_FORWARD, FORWARD_DC);
      analogWrite(MOTOR_RIGHT_FORWARD, PULSE_DC); //Might need to adjust based on rotation
      break;
  }
}

#ifdef SPEC_TESTING
// Straight line test
void spec_test_straight() {
  align();

  if (readSensor(FRONT_TOF) < TILEWIDTH*stoppingTiles[turnCount] + TILEGAP) {
    motor_control(STOP);
    //rotate90degrees();
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
#endif
