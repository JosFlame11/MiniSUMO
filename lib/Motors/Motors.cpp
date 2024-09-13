#include "Motors.h"

MotorControl::MotorControl(Motor motor)
    : motor(motor) {}

void MotorControl::begin() {
    // Set the direction pin as output
    pinMode(motor.dirPin, OUTPUT);
    pinMode(motor.pwmPin, OUTPUT);
}

void MotorControl::move(int speed) {
    // Cap the pwm frequency into 255
    speed = constrain(speed, -255, 255);

    // Set motor direction based on speed value
    if (speed >= 0) {
        digitalWrite(motor.dirPin, HIGH);
    } else { 
        digitalWrite(motor.dirPin, LOW);
        speed = -speed; // Make the speed positive for PWM duty cycle
    }

    // Write the PWM duty cycle
    analogWrite(motor.pwmPin, speed);
}