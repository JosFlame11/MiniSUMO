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

const int MAX_VEL = 75;

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

////////////////// PID //////////////////
double error = 0;
double last_error = 0;

unsigned long curr_time = 0;
unsigned long prev_time = 0;

double kp = 2;
double ki = 0;
double kd = 10;


///////////Function Definition////////////
void setMotors(int lvel, int rvel){
  LeftMotor.move(-lvel);
  RightMotor.move(rvel);
}

void update_sensors(){
  left_state = (analogRead(left_sensor) >= 500) ? 1 : 0;
  left_floor_state = (analogRead(left_floor_sensor) >= 500) ? 1 : 0;
  left_front_state = (analogRead(left_front_sensor) >= 500) ? 1 : 0;

  right_state = (analogRead(right_sensor) >= 500) ? 1 : 0;
  right_floor_state = (analogRead(right_floor_sensor) >= 500) ? 1 : 0;
  right_front_state = (analogRead(right_front_sensor) >= 500) ? 1 : 0;
}

void return_to_sender(){
  setMotors(-50, -50);
  delay(300);
  setMotors(125, -100);
  delay(200);
}

void target_position_with_turn(){
  if (left_state && !left_front_state && !right_front_state && !right_state) setMotors(MAX_VEL - 35, MAX_VEL + 35);
  else if (!left_state && left_front_state && !right_front_state && !right_state) setMotors(MAX_VEL - 15, MAX_VEL + 15);
  else if (!left_state && !left_front_state && right_front_state && !right_state) setMotors(MAX_VEL + 35, MAX_VEL - 35);
  else if (!left_state && !left_front_state && !right_front_state && right_state) setMotors(MAX_VEL + 35, MAX_VEL - 35);
  else if (left_front_state && right_front_state) setMotors(MAX_VEL, MAX_VEL);
  else if (!left_state && !left_front_state && !right_front_state && !right_state) setMotors(MAX_VEL * 0.7, MAX_VEL * -0.7);
}

void target_position_charge(){
  if (left_front_state && right_front_state) setMotors(220, 220);
  else if (left_state) setMotors(MAX_VEL - static_cast<int>(MAX_VEL * 0.4), MAX_VEL + static_cast<int>(MAX_VEL * 0.4));
  else if (!left_state && !left_front_state && !right_front_state && !right_state) setMotors(static_cast<int>(MAX_VEL * 0.7), static_cast<int>(MAX_VEL * -0.7));
}
void PIDTracking(){
  curr_time = millis();
  double dt = static_cast<double>(curr_time - prev_time) / 1000;

  int w_sum = left_state * -10 + left_front_state * -5 + right_front_state * 5 + right_state * 10;
  int sum_states = left_state + left_front_state + right_front_state + right_state;

  int pos = w_sum / sum_states;

  error = 0 - pos;
  double I = error * dt;
  double D = (error - last_error) / dt;
  
  double output = kp * error + ki * I + kd * D;

  int u = static_cast<int>(output);

  int PWMI = constrain(MAX_VEL - u, -255, 255);
  int PWMD = constrain(MAX_VEL + u, -255, 255);
  setMotors(PWMI, PWMD);
  // setMotors(-output, output);

}
void start_fight(){
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

  if((digitalRead(GO) == 1)){
  
    update_sensors();
    // All sensor are digital, there is no need to use analogRead() - not entirely true
    // Before moving, check if the btn GO has been pressed 
    // start_fight();
    // Rule: the robot most move fowards unitl edge for stage 1 and 2
    if (!left_floor_state && !right_floor_state) return_to_sender();
    else{
      if (digitalRead(sw1) && !digitalRead(sw2) && !digitalRead(sw3)){ // Only sw1 is flip
        start_fight();
        update_sensors();
        target_position_with_turn();
      }
      else if(digitalRead(sw2) && !digitalRead(sw1) && !digitalRead(sw3)){// Only sw2 is flip
        update_sensors();
        target_position_charge();
      }
      else if (digitalRead(sw3) && !digitalRead(sw1) && !digitalRead(sw2)){// Only sw3 is flip
        start_fight();
        update_sensors();
        PIDTracking();
      }
      
    }
  }
  else{
    setMotors(0, 0);
  }
  //@TODO: Check if the function can read the values correctly, use the leds before the motors


 
}


