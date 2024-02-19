#include "motor_pid.h"

// Initialize PID controller with given parameters
void pid_init(PIDController* pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->previous_error = 0.0;  // Initialize previous_error
}

// Update PID controller and compute control output
float pid_update(PIDController* pid, float input, float dt) {
    float error = pid->setpoint - input;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // Update previous_error for next iteration
    pid->previous_error = error;

    return output;
}
// Set new setpoint for PID controller
void pid_set_setpoint(PIDController* pid, float setpoint) {
    pid->setpoint = setpoint;
    pid_reset(pid);
}

// Reset PID controller state
void pid_reset(PIDController* pid) {
    pid->error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->previous_error = 0.0;
}