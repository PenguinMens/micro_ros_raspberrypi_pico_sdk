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

typedef struct MotorStats{
    float velocity;
    float rps;
} MotorStats;
int controlMotors(float linear_velocity, float angular_velocity);
void init_motors();
int controlMotorsPID(float linear_velocity, float angular_velocity, float linear_velocity_target);
#endif // MOTOR_CONTROL_H