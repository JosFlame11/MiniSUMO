#ifndef MOTORS_H_
#define MOTORS_H_

#include <Arduino.h>

struct Motor {
    uint8_t pwmPin;        // PWM control pin
    uint8_t dirPin;        // Direction control pin
    
    Motor(uint8_t pwmPin, uint8_t dirPin)
        : pwmPin(pwmPin), dirPin(dirPin){}
};

class MotorControl {
public:
    MotorControl(Motor motor);

    void begin();
    void move(int speed);

private:
    Motor motor;
};

#endif