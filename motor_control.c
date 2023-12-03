#include "motor_control.h"
#include <math.h>
#include "pico/stdlib.h"
#define HIGH 1
#define LOW 0
MotorPins motorB;
MotorPins motorA;

int PWM_MOTOR_MIN = 0;
int PWM_MOTOR_MAX = 100;

void init_motors(uint PWMB, uint BIN1,uint BIN2,uint PWMA, uint AIN1,uint AIN2){
    
    gpio_init(BIN1);
    gpio_init(BIN2);
    gpio_init(AIN1);
    gpio_init(AIN2);

    gpio_set_dir(BIN1, GPIO_OUT);
    gpio_set_dir(BIN2, GPIO_OUT);
    gpio_set_dir(AIN1, GPIO_OUT);
    gpio_set_dir(AIN2, GPIO_OUT);
    motorB.PWM_PIN = PWMB;
    gpio_set_function(motorB.PWM_PIN, GPIO_FUNC_PWM);
    motorB.SLICE = pwm_gpio_to_slice_num(motorB.PWM_PIN);
    motorB.CHANNEL = pwm_gpio_to_channel(motorB.PWM_PIN);
    pwm_set_enabled(motorB.SLICE, true);
    motorB.IN1_PIN = BIN1;
    motorB.IN2_PIN = BIN2;

    motorA.PWM_PIN = PWMA;
    gpio_set_function(motorA.PWM_PIN, GPIO_FUNC_PWM);
    motorA.SLICE = pwm_gpio_to_slice_num(motorA.PWM_PIN);
    motorA.CHANNEL = pwm_gpio_to_channel(motorA.PWM_PIN);
    pwm_set_enabled(motorA.SLICE, true);
    motorA.IN1_PIN = AIN1;
    motorA.IN2_PIN = AIN2;

    motorA.motor_pwm_val = 0;
    motorB.motor_pwm_val = 0;

}

int controlLeftMotor(float dir, int pwm)
{
    set_motor_dir(dir, motorB);
    pwm_set_freq_duty(motorB.SLICE,motorB.CHANNEL, 10000, pwm);

}
int controlRightMotor(float dir, int pwm)
{
    set_motor_dir(dir, motorA);
    pwm_set_freq_duty(motorA.SLICE,motorA.CHANNEL, 10000, pwm);
}
void set_motor_dir(float speed, MotorPins motor) {
    if (speed == 0) {
        gpio_put(motor.IN1_PIN, HIGH);
        gpio_put(motor.IN2_PIN, HIGH);
    } else if (speed > 0) {
        gpio_put(motor.IN1_PIN, HIGH);
        gpio_put(motor.IN2_PIN, LOW);
    } else {
        gpio_put(motor.IN1_PIN, LOW);
        gpio_put(motor.IN2_PIN, HIGH);
    }
}

