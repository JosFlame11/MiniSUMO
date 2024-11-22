#include <Arduino.h>
#include <servo.h>
#include "Motors.h"

///////////Sensors Definition////////////
#define LED1 13
#define LED3 1
#define LED4 0
#define clear 12
#define timeout 5000

unsigned long time_start = 0;
bool sensorActive = false;    

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

 int MAX_VEL = 150;

Servo flags_s;

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
int c = 0;
int c2 = 0;

int th = 1023/2; //threshlod of half 10-bit

bool DONE = false;
bool LINE = false;

////////////////// PID //////////////////
double error = 0;
double last_error = 0;

unsigned long curr_time = 0;
unsigned long prev_time = 0;
//base values for 150 pwm
double kp = 32;
double ki = 0;
double kd = 17;

/////////////////// PD //////////////////
int sensor_value, u, pwmi, pwmd;
int pos, sensornum, posa, p, pa, d;

void LEDS(int i){
  if (i == 1){
    digitalWrite(LED1, 0);
    digitalWrite(LED3, 1);
    digitalWrite(LED4, 0);
  }
  if (i == 2){
    digitalWrite(LED1, 0);
    digitalWrite(LED3, 0);
    digitalWrite(LED4, 1);
  }
  if (i == 3){
    digitalWrite(LED1, 0);
    digitalWrite(LED3, 1);
    digitalWrite(LED4, 1);
  }
  if (i == 4){
    digitalWrite(LED1, 1);
    digitalWrite(LED3, 0);
    digitalWrite(LED4, 0);
  }
  else{
    digitalWrite(LED1, 0);
    digitalWrite(LED3, 0);
    digitalWrite(LED4, 0);
  }
}
void status(){
  if (digitalRead(sw1) && !digitalRead(sw2) && !digitalRead(sw3)) { //100
      LEDS(1);
      } else if (digitalRead(sw2) && !digitalRead(sw1) && !digitalRead(sw3)) { //010
      LEDS(2);
      } else if (digitalRead(sw3) && !digitalRead(sw1) && !digitalRead(sw2)) { //001
      LEDS(3);
      } else if (!digitalRead(sw1) && !digitalRead(sw2) && !digitalRead(sw3)) { //000
      LEDS(4);
      }
      else{
      LEDS(0);
      }
}
///////////Function Definition////////////
void setMotors(int lvel, int rvel){
  LeftMotor.move(-lvel);
  RightMotor.move(rvel*-1);
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
  setMotors(-120, -120);
  delay(200);
  setMotors(125, -100);
  delay(65);
  setMotors(0,0);
}
void roll(){
  setMotors(20, 20);
  delay(50);
  setMotors(-70, 70);
  delay(180);
  setMotors(70, 70);
  delay(350);
  setMotors(-100, 100);
}
void dodge (){
  setMotors(80, -80);
  delay(140);
  setMotors(45, 110);
  delay(450);
  setMotors(70, 110);
  delay(120);
  setMotors(-80, 80);
  delay(180);
  setMotors(0,0);
}
void flag1(){
  setMotors(80, -82);
  delay(120);
  setMotors(90, 120);
  delay(210);
  setMotors(-80, 80);
  delay(220);
  setMotors(0,0);
}

void frenos(){
  setMotors(-200,-200);
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
void speed_control(){ //cambio de velocidad al atacar
  update_sensors();
  if (!left_state && left_front_state && right_front_state && !right_state) {
    if(!sensorActive){
      time_start = millis();
      sensorActive = true;
    }
    if (millis() - time_start > timeout){
      MAX_VEL = 220;
    }
  } else{
      MAX_VEL = 150;
      sensorActive = false;
      time_start = 0;
    }
}
void PIDTracking(int MAX_VEL2, int kp2, int kd2){
  //speed_control();
  curr_time = millis();
  double dt = static_cast<double>(curr_time - prev_time) / 1000;

  int w_sum = left_state * -10 + left_front_state * -5 + right_front_state * 5 + right_state * 10;
  int sum_states = left_state + left_front_state + right_front_state + right_state;

  int pos = w_sum / sum_states;

  error = 0 - pos;
  double I = error * dt;
  double D = (error - last_error) / dt;
  
  double output = kp2 * error + ki * I + kd2 * D;

  int u = static_cast<int>(output);

  int PWMI = constrain(MAX_VEL2 - u, -255, 255);
  int PWMD = constrain(MAX_VEL2 + u, -255, 255);
  setMotors(PWMI, PWMD);
  // setMotors(-output, output);

}
void start_fight(){ //sensado linea
  while (!DONE){
    setMotors(30, 30);
    update_sensors();
    digitalWrite(LED1, left_floor_state);
    //digitalWrite(LED2, right_floor_state);
    if (!left_floor_state || !right_floor_state){
      DONE = true;
      break;
    }
  }
}

void setup() {

  pinMode(LED1, OUTPUT);
 // pinMode(LED2, OUTPUT);
  pinMode(clear, INPUT);
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
  //digitalWrite(LED2, LOW);
  flags_s.attach(10);
  c = 1;
  c2 = 1;
  LINE = true;
  DONE = true;
  flags_s.write(0);
}

void loop() {
  status();
  if (digitalRead(GO) == 1) {
    //flags_s.write(79);
    left_floor_state = (analogRead(left_floor_sensor) >= 500) ? 1 : 0;
    right_floor_state = (analogRead(right_floor_sensor) >= 500) ? 1: 0;

    if (!left_floor_state || !right_floor_state) {
      return_to_sender();
      update_sensors();
    } 
    else {
      if (digitalRead(sw1) && !digitalRead(sw2) && !digitalRead(sw3)) { //100 espalda
          LEDS(1);
          if(c==0){
            roll();
            c++;
          }
          else{
            update_sensors();
            PIDTracking(70, 25, 12);//(pwm, kp,kd)
            update_sensors();
            if (left_state == 0 && left_front_sensor == 1 && right_front_sensor == 1 && right_sensor == 0){
                setMotors(200,200);
            }
          }    
      } else if (digitalRead(sw2) && !digitalRead(sw1) && !digitalRead(sw3)) { //010 lado
          LEDS(2);
          if(c==0){
            dodge();
            c++;
          }
          else{
            update_sensors();
            PIDTracking(70, 25, 12);//(pwm, kp,kd)
            update_sensors();
            if (left_state == 0 && left_front_sensor == 1 && right_front_sensor == 1 && right_sensor == 0){
                setMotors(200,200);
            }
          }  
        }
       else if (digitalRead(sw3) && !digitalRead(sw1) && !digitalRead(sw2)) { //001 frontal 1
          LEDS(3);
          if(c==0){
            flag1();
            c++;
          }
          else{
            update_sensors();
            PIDTracking(70, 25, 12);//(pwm, kp,kd)
            update_sensors();
            if (left_state == 0 && left_front_sensor == 1 && right_front_sensor == 1 && right_sensor == 0){
                setMotors(200,200);
            }
          }  
      }
       else if (!digitalRead(sw1) && !digitalRead(sw2) && !digitalRead(sw3)) { //nada
        LEDS(4);
       } 
       else if (digitalRead(sw1) && digitalRead(sw2) && !digitalRead(sw3)){ //frontal 2
          flags_s.write(70);
          update_sensors();
          PIDTracking(230, 35, 18);//(pwm, kp,kd)
      }
      //   else if (digitalRead(sw1) && digitalRead(sw2) && digitalRead(sw3)){ //debug 111
      //     flags_s.write(0);
      //     update_sensors();
      //     // PIDTracking(230, 36, 21);//(pwm, kp,kd)
      //     PIDTracking(150, 32, 17);//(pwm, kp,kd)
      // }
    }
  
    

    // Puedes agregar más lógica aquí para las demás acciones
    /*
    if (!left_floor_state && !right_floor_state) {
      return_to_sender();
    } else {
      if (digitalRead(sw1) && !digitalRead(sw2) && !digitalRead(sw3)) {
        start_fight();
        update_sensors();
        target_position_with_turn();
      } else if (digitalRead(sw2) && !digitalRead(sw1) && !digitalRead(sw3)) {
        update_sensors();
        target_position_charge();
      } else if (digitalRead(sw3) && !digitalRead(sw1) && !digitalRead(sw2)) {
        start_fight();
        update_sensors();
        PIDTracking();
      }
    }
    */
  } else {
    setMotors(0, 0);
    flags_s.write(0);
    if(digitalRead(clear)){ //resetea las variables de control unicas solo cuando rdy del control se prepare, asi no se ejecutan las tareas unicas si se resetea
        c = 0;
        c2 = 0;
        LINE = false;
        DONE = false;
        flags_s.write(0);
    }
  }
}