#include "motor_control.h"  // Include motor control functions and structures
#include <math.h>           // Include math functions
#include "pico/stdlib.h"    // Include Pico standard library functions

// Constants for GPIO
#define HIGH 1
#define LOW 0
void set_motor_dir(float speed, MotorPins motor); 
// Define Motor Pins structure
MotorPins motorB;  // Instance of MotorPins structure for Motor B
MotorPins motorA;  // Instance of MotorPins structure for Motor A

// PWM range constants
int PWM_MOTOR_MIN = 0;  // Minimum PWM value for motors
int PWM_MOTOR_MAX = 100;  // Maximum PWM value for motors

// Function to initialize motors
void init_motors(uint PWMB, uint BIN1, uint BIN2, uint PWMA, uint AIN1, uint AIN2) {
    // Initialize GPIO pins for both motors
    gpio_init(BIN1);
    gpio_init(BIN2);
    gpio_init(AIN1);
    gpio_init(AIN2);

    // Set GPIO directions for both motors
    gpio_set_dir(BIN1, GPIO_OUT);
    gpio_set_dir(BIN2, GPIO_OUT);
    gpio_set_dir(AIN1, GPIO_OUT);
    gpio_set_dir(AIN2, GPIO_OUT);

    // Initialize Motor B
    motorB.PWM_PIN = PWMB;
    gpio_set_function(motorB.PWM_PIN, GPIO_FUNC_PWM);
    motorB.SLICE = pwm_gpio_to_slice_num(motorB.PWM_PIN);
    motorB.CHANNEL = pwm_gpio_to_channel(motorB.PWM_PIN);
    pwm_set_enabled(motorB.SLICE, true);
    motorB.IN1_PIN = BIN1;
    motorB.IN2_PIN = BIN2;

    // Initialize Motor A
    motorA.PWM_PIN = PWMA;
    gpio_set_function(motorA.PWM_PIN, GPIO_FUNC_PWM);
    motorA.SLICE = pwm_gpio_to_slice_num(motorA.PWM_PIN);
    motorA.CHANNEL = pwm_gpio_to_channel(motorA.PWM_PIN);
    pwm_set_enabled(motorA.SLICE, true);
    motorA.IN1_PIN = AIN1;
    motorA.IN2_PIN = AIN2;

    // Set initial PWM values for both motors
    motorA.motor_pwm_val = 0;
    motorB.motor_pwm_val = 0;
}

// Function to control right motor
int controlRightMotor(float dir, int pwm) {
    set_motor_dir(dir, motorA);  // Set direction of Motor A
    pwm_set_freq_duty(motorA.SLICE, motorA.CHANNEL, 10000, pwm);  // Set PWM duty cycle for Motor A
}

// Function to control left motor
int controlLeftMotor(float dir, int pwm) {
    set_motor_dir(dir, motorB);  // Set direction of Motor B
    pwm_set_freq_duty(motorB.SLICE, motorB.CHANNEL, 10000, pwm);  // Set PWM duty cycle for Motor B
}

// Function to set motor direction based on speed
void set_motor_dir(float speed, MotorPins motor) {
    if (speed < 0.000001f && speed > -0.000001f) {
        gpio_put(motor.IN1_PIN, HIGH);  // Set input 1 of motor to HIGH
        gpio_put(motor.IN2_PIN, HIGH);  // Set input 2 of motor to HIGH
    } else if (speed > 0.000001f) {
        gpio_put(motor.IN1_PIN, HIGH);  // Set input 1 of motor to HIGH
        gpio_put(motor.IN2_PIN, LOW);   // Set input 2 of motor to LOW
    } else {
        gpio_put(motor.IN1_PIN, LOW);   // Set input 1 of motor to LOW
        gpio_put(motor.IN2_PIN, HIGH);  // Set input 2 of motor to HIGH
    }
}
