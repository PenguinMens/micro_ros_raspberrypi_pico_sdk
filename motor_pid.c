#include <stdio.h>
#include "motor_pid.h"

// Initialize PID controller with given parameters
void pid_init(PIDController* pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd; pid->setpoint = setpoint;
    pid->error = 0.0; pid->integral = 0.0; pid->derivative = 0.0;
}

// Update PID controller and compute control output
float pid_update(PIDController* pid, float input, float dt) {
    pid->error = pid->setpoint - input;
    pid->integral += pid->error * dt;
    pid->derivative = (pid->error - pid->derivative) / dt;
    float output = pid->Kp * pid->error + pid->Ki * pid->integral;
    return output;
}

// Set new setpoint for PID controller
void pid_set_setpoint(PIDController* pid, float setpoint) {
    pid->setpoint = setpoint;
}

// Reset PID controller state
void pid_reset(PIDController* pid) {
    pid->error = 0.0; pid->integral = 0.0; pid->derivative = 0.0;
}
