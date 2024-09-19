#include <Arduino.h>
#include "Motors.h"

///////////Sensors Definition////////////
#define LED1 13
#define LED2 12

const uint8_t right_floor_sensor = A0;
const uint8_t right_sensor = A1;
const uint8_t right_front_sensor = A2;
const uint8_t left_front_sensor = A5;
const uint8_t left_sensor = A6;
const uint8_t left_floor_sensor = A7;

const int sw1 = 4;
const int sw2 = 5;
const int sw3 = 6;
// const int sw4 = 7;
const int START = 9;
const int GO = 8;
const int RDY = 12;

///////////Motor Definition////////////
const uint8_t left_dir_pin = 2;
const uint8_t left_pwm_pin = 3;

const uint8_t right_dir_pin = 7;
const uint8_t right_pwm_pin = 11;

const int MAX_VEL = 150;

Motor motor1(left_pwm_pin, left_dir_pin);
Motor motor2(right_pwm_pin, right_dir_pin);

MotorControl LeftMotor(motor1);
MotorControl RightMotor(motor2);

///////////Variables Definition////////////
int left_state = 0;
int left_front_state = 0;
int left_floor_state = 0;

int right_state = 0;
int right_front_state = 0;
int right_floor_state = 0;

int th = 1023/2; //threshlod of half 10-bit

bool DONE = false;

///////////Function Definition////////////
void setMotors(int lvel, int rvel){
  LeftMotor.move(-lvel);
  RightMotor.move(rvel);
}

void update_sensors(){
  left_state = (analogRead(left_sensor) >= 500) ? 1 : 0;
  left_floor_state = (analogRead(left_floor_sensor) >= 500) ? 0 : 1;
  left_front_state = (analogRead(left_front_sensor) >= 500) ? 1 : 0;

  right_state = (analogRead(right_sensor) >= 500) ? 1 : 0;
  right_floor_state = (analogRead(right_floor_sensor) >= 500) ? 0 : 1;
  right_front_state = (analogRead(right_front_sensor) >= 500) ? 1 : 0;
}

void return_to_sender(){
  setMotors(-50, -50);
  delay(300);
  setMotors(125, -100);
  delay(200);
}
int target_position(){
  int diff_vel = 0;
  if (left_state && !left_front_state && !right_front_state && !right_state) diff_vel = -35;
  else if (!left_state && left_front_state && !right_front_state && !right_state) diff_vel = -15;
  else if (!left_state && !left_front_state && right_front_state && !right_state) diff_vel = 15;
  else if (!left_state && !left_front_state && !right_front_state && right_state) diff_vel = 35;
  else if (left_front_state && right_front_state) diff_vel = 0;
  return diff_vel;
}

void setup() {

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(left_sensor, INPUT);
  pinMode(right_sensor, INPUT);
  pinMode(left_floor_sensor, INPUT);
  pinMode(right_floor_sensor, INPUT);
  pinMode(left_front_sensor, INPUT);
  pinMode(right_front_sensor, INPUT);

  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  pinMode(sw3, INPUT);
  pinMode(START, INPUT);
  pinMode(GO, INPUT);
  pinMode(RDY, INPUT);

  LeftMotor.begin();
  RightMotor.begin();

  setMotors(0,0);

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

}

void loop() {
  // All sensor are digital, there is no need to use analogRead() - not entirely true
  // Before moving, check if the btn GO has been pressed 
  update_sensors();
  // Rule: the robot most move fowards unitl edge
  while (!DONE){
    setMotors(30, 30);
    update_sensors();
    digitalWrite(LED1, left_floor_state);
    digitalWrite(LED2, right_floor_state);
    if (!left_floor_state && !right_floor_state){
      DONE = true;
      break;
    }
  }
  if (!left_floor_state && !right_floor_state) return_to_sender();
  else{
    int diff = target_position();
    // We need an environment free of obstacles to test this
    setMotors(20 + diff, 20 - diff);
  }
  //@TODO: Check if the function can read the values correctly, use the leds before the motors


 
}


