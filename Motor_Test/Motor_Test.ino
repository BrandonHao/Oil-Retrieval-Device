#define PWM_MAX           255
#define PWM_DIVIDER       2
#define MOTOR_DUTY_CYCLE  (PWM_MAX / PWM_DIVIDER)
#define MOTOR_1_PIN_1     10
#define MOTOR_1_PIN_2     11
#define MOTOR_2_PIN_1     5
#define MOTOR_2_PIN_2     6

void setup() {
    pinMode(MOTOR_1_PIN_1, OUTPUT);
    pinMode(MOTOR_1_PIN_2, OUTPUT);
    pinMode(MOTOR_1_PIN_1, OUTPUT);
    pinMode(MOTOR_1_PIN_2, OUTPUT);
}

void loop() {
    //PWM motors one direction
    analogWrite(MOTOR_1_PIN_1, MOTOR_DUTY_CYCLE);
    analogWrite(MOTOR_2_PIN_1, MOTOR_DUTY_CYCLE);
    delay(1000);
    //Turn motors off
    digitalWrite(MOTOR_1_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_1, LOW);
    delay(20);
    //PWM motors other direction
    analogWrite(MOTOR_1_PIN_2, MOTOR_DUTY_CYCLE);
    analogWrite(MOTOR_2_PIN_2, MOTOR_DUTY_CYCLE);
    delay(1000);
    //Turn motors off
    digitalWrite(MOTOR_1_PIN_2, LOW);
    digitalWrite(MOTOR_2_PIN_2, LOW);
    delay(20);
}
