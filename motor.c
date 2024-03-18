#include <math.h>           // Include math functions
#include "motor.h"  // Include motor control functions and structures
#include "pico/stdlib.h"    // Include Pico standard library functions


void init_motor(Motor *motor, uint PWM, uint IN1, uint IN2, uint ENCODER_PINS, float kp, float ki, float kd, float setpoint, char* name) {



    // Initialize GPIO pins for motor
    gpio_init(IN1);
    gpio_init(IN2);


    // Set GPIO directions for motor
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_set_dir(IN2, GPIO_OUT);
    

    // Initialize Motor B
    for (int i = 0; i < max_motor_name_size; i++) {
        if(i == 9){
            motor->name[i] = '\0';
            break;
        }
        motor->name[i] = name[i];
        if(name[i] == '\0') break;
    }
    motor->PWM_PIN = PWM;
    gpio_set_function(motor->PWM_PIN, GPIO_FUNC_PWM);
    motor->SLICE = pwm_gpio_to_slice_num(motor->PWM_PIN);
    motor->CHANNEL = pwm_gpio_to_channel(motor->PWM_PIN);
    pwm_set_enabled(motor->SLICE, true);
    motor->IN1_PIN = IN1;
    motor->IN2_PIN = IN2;
    motor->ENCODER_PINS = ENCODER_PINS;
    

    pid_init(&(motor->motorStats.pid), kp, ki, kd, setpoint);


}
