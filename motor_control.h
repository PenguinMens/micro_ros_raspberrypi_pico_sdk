/** motor_control.h */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "utils.h"       // Include utility functions
#include "custom_pwm.h"  // Include custom PWM functions

/**
 * Explanation of motor_control.h:
 * This header file defines functions and structures related to motor control.
 * It provides an interface for initializing motors and controlling their speed and direction.
 */

// Structure to hold motor pins and PWM values
typedef struct MotorPins {
    unsigned int PWM_PIN;  // Pin for PWM signal
    unsigned int CHANNEL;  // PWM channel (shared with slice)
    unsigned int SLICE;    // PWM slice (specific to Pico)
    unsigned int IN1_PIN;  // Motor input pin 1
    unsigned int IN2_PIN;  // Motor input pin 2
    float motor_pwm_val;   // Current PWM value for the motor
} MotorPins;

/**
 * Initializes the motor pins and PWM settings.
 * Parameters:
 *   PWMB, BIN1, BIN2: Pins for Motor B PWM, input 1, and input 2.
 *   PWMA, AIN1, AIN2: Pins for Motor A PWM, input 1, and input 2.
 */
void init_motors(uint PWMB, uint BIN1, uint BIN2, uint PWMA, uint AIN1, uint AIN2);

/**
 * Controls the left motor direction and PWM value.
 * Parameters:
 *   dir: Direction of the motor rotation (positive or negative).
 *   pwm: PWM value for controlling motor speed.
 * Returns:
 *   Returns the status of the motor control operation.
 */
int controlLeftMotor(float dir, float pwm);

/**
 * Controls the right motor direction and PWM value.
 * Parameters:
 *   dir: Direction of the motor rotation (positive or negative).
 *   pwm: PWM value for controlling motor speed.
 * Returns:
 *   Returns the status of the motor control operation.
 */
int controlRightMotor(float dir, float pwm);

#endif // MOTOR_CONTROL_H
