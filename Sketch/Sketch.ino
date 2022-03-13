#include "src/TofSensor.cpp"
#include "src/ImuSensor.cpp"

#define FRONT_TOF SENSOR_1
#define LEFT_TOF SENSOR_2
#define RIGHT_TOF SENSOR_3
#define IMU 1
#define TILEWIDTH 305
// change after mechanical is done
#define TILEGAP 50

// Motor pins
// Name these forward and back once known
#define MOTOR_1_PIN_1     10
#define MOTOR_1_PIN_2     11
#define MOTOR_2_PIN_1     5
#define MOTOR_2_PIN_2     6

// Motor duty cycles
#define FORWARD_DC       50
#define TURN_90_DC       30
#define PULSE_RIGHT_DC   60
#define PULSE_LEFT_DC    60

typedef enum{
    STOP = 0,
    FORWARD = 1,
    TURN_90 = 2,
    PULSE_RIGHT = 3,
    PULSE_LEFT = 4
}MOVEMENT;

// number tiles in the stopping distance per turn
const uint16_t stoppingTiles[] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2};
const uint16_t leftTiles[] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
uint16_t turnCount = 0;

imu_data_t gyro_data; 
int16_t current_angle, delta_time, delta_angle, prev_time, curr_time, sample; 

uint16_t left_dist, right_dist;

void setup() {

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

  //Motor setup
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);
}

void loop() {
  // TODO: start motors
  motor_control(FORWARD);

  // detecting corners to turn at
  if (readSensor(FRONT_TOF) < TILEWIDTH*stoppingTiles[turnCount] + TILEGAP) {
    // TODO: stop motors
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
}

void rotate90degrees() {
  motor_control(TURN_90);

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

  motor_control(STOP);
}

void align() {
  // distance to left wall
  left_dist = readSensor((TOF_SENSOR)(LEFT_TOF));
  
  // if we're too far from the left wall, pulse the left motor off
  if (left_dist - (TILEWIDTH*leftTiles[turnCount] + TILEGAP) > 15) {
    motor_control(PULSE_LEFT);
  }
  // if we're too close to the left wall, pulse the right motor off
  else if (left_dist - (TILEWIDTH*leftTiles[turnCount] + TILEGAP) < -15) {
    motor_control(PULSE_RIGHT);
  }
}

//Could get rid of this depending on if we need a more accurate pulse side to side or not
uint8_t scale_DC(uint8_t duty_cycle) {
    //PWM max is 255 so scale the duty cycle to that
    return 255 * (duty_cycle / 100);
}

// up speed to turn dont shut off avoid turning it off
void motor_control(MOVEMENT movement) {
    switch(movement) {
        case(STOP):
            //Stop motors
            Serial.println("Stopping");
            digitalWrite(MOTOR_1_PIN_1, LOW);
            digitalWrite(MOTOR_2_PIN_1, LOW);
            digitalWrite(MOTOR_1_PIN_2, LOW);
            digitalWrite(MOTOR_2_PIN_2, LOW);
            break;
        case(FORWARD):
            //Forward motors
            Serial.println("Forward");
            analogWrite(MOTOR_1_PIN_1, scale_DC(FORWARD_DC));
            analogWrite(MOTOR_2_PIN_1, scale_DC(FORWARD_DC));
            //other direction
            //analogWrite(MOTOR_1_PIN_2, MOTOR_DUTY_CYCLE);
            //analogWrite(MOTOR_2_PIN_2, MOTOR_DUTY_CYCLE);
            break;
        case(TURN_90):
            //Turn motors right
            Serial.println("Turning 90 degrees");
            analogWrite(MOTOR_1_PIN_1, scale_DC(TURN_90_DC));
            analogWrite(MOTOR_2_PIN_2, scale_DC(TURN_90_DC));
            //other direction
            //analogWrite(MOTOR_1_PIN_2, MOTOR_DUTY_CYCLE);
            //analogWrite(MOTOR_2_PIN_1, MOTOR_DUTY_CYCLE);
            break;
        case(PULSE_RIGHT):
            //Pulse motors right
            Serial.println("Adjusting right");
            analogWrite(MOTOR_1_PIN_1, scale_DC(PULSE_RIGHT_DC)); //Might need to adjust based on rotation
            analogWrite(MOTOR_2_PIN_1, scale_DC(FORWARD_DC));
            break;
        case(PULSE_LEFT):
            //Pulse motors left
            Serial.println("Adjusting left");
            analogWrite(MOTOR_1_PIN_1, scale_DC(FORWARD_DC));
            analogWrite(MOTOR_2_PIN_1, scale_DC(PULSE_LEFT_DC)); //Might need to adjust based on rotation
            break;
    }
}
