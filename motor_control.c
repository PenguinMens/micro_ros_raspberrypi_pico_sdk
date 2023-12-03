#include "motor_control.h"
#include <math.h>
#include "pico/stdlib.h"

MotorPins motorB;
MotorPins motorA;
MotorStats motorStatsA;
MotorStats motorStatsB;
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
int controlMotors(float linear_velocity,  float angular_velocity) {
    // Calculate motor speeds based on linear and angular velocities\
    float left_speed = linear_velocity - angular_velocity;
    //float right_speed = linear_velocity + angular_velocity;
    float linear = constrain(linear_velocity, -1, 1);
    float angular = constrain(angular_velocity, -1, 1);
    // Map the speeds to the range [-1, 1]
    float left_speed = (linear - angular) / 2.0f;
    float right_speed = (linear + angular) / 2.0f;


    float pwmLeft = (uint16_t) fmap(fabs(left_speed), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    float pwmRight = (uint16_t) fmap(fabs(right_speed), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);

    // Perform motor control based on the calculated speeds
    // (Assuming you have already set up the GPIO or other pin control)
    // Example using pico-sdk:
 

    if(left_speed == 0)// stop speed
    {
        gpio_put(motorB.IN1_PIN, 1);
        gpio_put(motorB.IN2_PIN, 1);
        // PORTC |= (1<<PC0);
        // PORTC |= (1<<PC1);
    }
    else if(left_speed>0) //if lm is positive
    {
        gpio_put(motorB.IN1_PIN, 1);
        gpio_put(motorB.IN2_PIN, 0);
        // PORTC |= (1<<PC0);
        // PORTC &= ~(1<<PC1);
    }
    else
    {
        //set direction reverse
        // PORTC &= ~(1<<PC0);
        // PORTC |= (1<<PC1);
        gpio_put(motorB.IN1_PIN, 0);
        gpio_put(motorB.IN2_PIN, 1);
    }

    if(right_speed == 0)
    {

        gpio_put(motorA.IN1_PIN, 1);
        gpio_put(motorA.IN2_PIN,  1);
        // PORTC |= (1<<PC2);
        // PORTC |= (1<<PC3);
    }
    else if(right_speed>0) //if rm is positive
    {
        //set direction forwards
        gpio_put(motorA.IN1_PIN, 1);
        gpio_put(motorA.IN2_PIN,  0);
        // PORTC |= (1<<PC2);
        // PORTC &= ~(1<<PC3);
    }
    else
    {
        //set direction reverse
        gpio_put(motorA.IN1_PIN, 0);
        gpio_put(motorA.IN2_PIN,  1);
        // PORTC &= ~(1<<PC2);
        // PORTC |= (1<<PC3);
        }
    // gpio_put(motorB.IN1_PIN, left_speed > 0);
    // gpio_put(motorB.IN2_PIN, left_speed < 0);
    // gpio_put(motorA.IN1_PIN, right_speed > 0);
    // gpio_put(motorA.IN2_PIN, right_speed < 0);

    // Optionally, you can adjust the PWM duty cycle for finer control
    // (Assuming PWM is available on your motor driver and configured)
    // Example using pico-sdk:
        pwm_set_freq_duty(motorB.SLICE,motorB.CHANNEL, 10000, pwmLeft);
        pwm_set_freq_duty(motorA.SLICE,motorA.CHANNEL, 10000, pwmRight);

    // pwm_set_gpio_level(LEFT_MOTOR_PWM_PIN, (uint16_t)(fabs(left_speed) * PWM_RANGE));
    // pwm_set_gpio_level(RIGHT_MOTOR_PWM_PIN, (uint16_t)(fabs(right_speed) * PWM_RANGE));
    
    return 0;
}
void doPID(float &sp, float &pv, float &err,
		float &p, float &i, float &d)
{

}
void setMotorSpeedRPS(float RPS, int dir)
int controlMotorsPID(float linear_velocity, float angular_velocity, float linear_velocity_target) {
    // Calculate motor speeds based on linear and angular velocities
    float left_speed = (linear_velocity - angular_velocity) / 2.0f;
    float right_speed = (linear_velocity + angular_velocity) / 2.0f;

    // Constrain speeds to the range [-1, 1]
    left_speed = constrain(left_speed, -1.0f, 1.0f);
    right_speed = constrain(right_speed, -1.0f, 1.0f);

    // Calculate the error between the current linear velocity and the target linear velocity
    float linear_velocity_error = linear_velocity_target - linear_velocity;

    // Proportional (P) control: Adjust PWM based on the linear velocity error
    float Kp_linear = 0.5;
    float pwm_adjustment = Kp_linear * linear_velocity_error;
    if(linear_velocity_error < 0.01 && linear_velocity_error > -0.01 ){

    }
    else if( fabs(linear_velocity_target) <  fabs (linear_velocity)){
        motorA.motor_pwm_val -= fabs(pwm_adjustment);
        motorB.motor_pwm_val -= fabs(pwm_adjustment);
    }
    else
    {
        motorA.motor_pwm_val += fabs(pwm_adjustment);
        motorB.motor_pwm_val += fabs(pwm_adjustment);
    }
    // Apply the PWM adjustment to both left and right speeds

    // Perform motor control based on the calculated speeds
    // (Assuming you have already set up the GPIO or other pin control)
    // Example using pico-sdk:

    if(linear_velocity_target == 0)// stop speed
    {
        gpio_put(motorB.IN1_PIN, 1);
        gpio_put(motorB.IN2_PIN, 1);
        // PORTC |= (1<<PC0);
        // PORTC |= (1<<PC1);
    }
    else if(linear_velocity_target>0) //if lm is positive
    {
        gpio_put(motorB.IN1_PIN, 1);
        gpio_put(motorB.IN2_PIN, 0);
        // PORTC |= (1<<PC0);
        // PORTC &= ~(1<<PC1);
    }
    else
    {
        //set direction reverse
        // PORTC &= ~(1<<PC0);
        // PORTC |= (1<<PC1);
        gpio_put(motorB.IN1_PIN, 0);
        gpio_put(motorB.IN2_PIN, 1);
    }

    if(linear_velocity_target == 0)
    {

        gpio_put(motorA.IN1_PIN, 1);
        gpio_put(motorA.IN2_PIN,  1);
        // PORTC |= (1<<PC2);
        // PORTC |= (1<<PC3);
    }
    else if(linear_velocity_target>0) //if rm is positive
    {
        //set direction forwards
        gpio_put(motorA.IN1_PIN, 1);
        gpio_put(motorA.IN2_PIN,  0);
        // PORTC |= (1<<PC2);
        // PORTC &= ~(1<<PC3);
    }
    else
    {
        //set direction reverse
        gpio_put(motorA.IN1_PIN, 0);
        gpio_put(motorA.IN2_PIN,  1);
        // PORTC &= ~(1<<PC2);
        // PORTC |= (1<<PC3);
        }
    // gpio_put(motorB.IN1_PIN, left_speed > 0);
    // gpio_put(motorB.IN2_PIN, left_speed < 0);
    // gpio_put(motorA.IN1_PIN, right_speed > 0);
    // gpio_put(motorA.IN2_PIN, right_speed < 0);

    // Optionally, you can adjust the PWM duty cycle for finer control
    // (Assuming PWM is available on your motor driver and configured)
    // Example using pico-sdk:
        pwm_set_freq_duty(motorB.SLICE,motorB.CHANNEL, 10000, motorB.motor_pwm_val);
        pwm_set_freq_duty(motorA.SLICE,motorA.CHANNEL, 10000, motorA.motor_pwm_val);

    // pwm_set_gpio_level(LEFT_MOTOR_PWM_PIN, (uint16_t)(fabs(left_speed) * PWM_RANGE));
    // pwm_set_gpio_level(RIGHT_MOTOR_PWM_PIN, (uint16_t)(fabs(right_speed) * PWM_RANGE));
    
    return 0;
}