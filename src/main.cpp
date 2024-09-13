#include <Arduino.h>
#include "Motors.h"

#define LED1 13
#define LED2 12

const uint8_t right_floor_sensor = A0;
const uint8_t right_sensor = A1;
const uint8_t right_front_sensor = A2;
const uint8_t left_front_sensor = A7;
const uint8_t left_sensor = A6;
const uint8_t left_floor_sensor = A5;

const int sw1 = 4;
const int sw2 = 5;
const int sw3 = 6;
// const int sw4 = 7;
const int START = 9;
const int GO = 8;
const int RDY = 12;

const uint8_t left_dir_pin = 7;
const uint8_t left_pwm_pin = 8;

const uint8_t right_dir_pin = 9;
const uint8_t right_pwm_pin = 10;

const int MAX_VEL = 255;

Motor motor1(left_pwm_pin, left_dir_pin);
Motor motor2(right_pwm_pin, right_dir_pin);

MotorControl LeftMotor(motor1);
MotorControl RightMotor(motor2);

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

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

}

void loop() {
  if(digitalRead(left_sensor)){
    digitalWrite(LED1, HIGH);
  }
  else{
    digitalWrite(LED1, LOW);
  }
  if(digitalRead(right_sensor)){
    digitalWrite(LED2, HIGH);
  }
  else{
    digitalWrite(LED2, LOW);
  }
  
}


