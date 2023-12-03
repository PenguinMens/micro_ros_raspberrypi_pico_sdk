// motor_control.h

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "utils.h"
#include "custom_pwm.h"
typedef struct MotorPins{
    unsigned int PWM_PIN;
    unsigned int CHANNEL; // pwm channel and slice
    unsigned int SLICE; // pico exclusive for pwm 
    unsigned int IN1_PIN;
    unsigned int IN2_PIN;
    float motor_pwm_val;
} MotorPins;

void init_motors(uint PWMB, uint BIN1, uint BIN2, uint PWMA, uint AIN1, uint AIN2);
int controlLeftMotor(float dir, int pwm);
int controlRightMotor(float dir, int pwm);
#endif // MOTOR_CONTROL_H