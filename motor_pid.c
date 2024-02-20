#include "motor_pid.h"

// PWM range constants
int PWM_MOTOR_MIN = 0;  // Minimum PWM value for motors
int PWM_MOTOR_MAX = 100;  // Maximum PWM value for motors
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
float pid_update(PIDController* pid, float input, float dt_ms) {
  //  printf("PID: %f,%f,%f,%f,%f,%f\n", pid->Kp, pid->Ki, pid->Kd,pid->error,pid->integral,pid->derivative);
    float dt = dt_ms/1000.0f;
    float error = pid->setpoint - input;
    pid->integral = pid->integral + error * dt;
    float derivative = (error - pid->previous_error) / dt;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
//    printf("%f,%f,%f\n", error,pid->integral, derivative);
    // Update previous_error for next iteration
    pid->previous_error = error;

    return output;
}
// Set new setpoint for PID controller
void pid_set_setpoint(PIDController* pid, float setpoint) {
    pid->setpoint = setpoint;
    //pid_reset(pid);
}

// Reset PID controller state
void pid_reset(PIDController* pid) {
    pid->error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->previous_error = 0.0;
}