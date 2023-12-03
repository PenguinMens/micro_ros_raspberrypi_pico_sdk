#ifndef MOTOR_PID_H
#define MOTOR_PID_H

// Include any necessary libraries or headers here

// Declare any constants or global variables here
typedef struct {
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain

    float setpoint;  // Desired setpoint
    float error;     // Current error
    float integral;  // Integral term
    float derivative;  // Derivative term
} PIDController;
// Declare function prototypes here

void pid_init(PIDController* pid, float Kp, float Ki, float Kd, float setpoint);
float pid_update(PIDController* pid, float input, float dt);
void pid_set_setpoint(PIDController* pid, float setpoint);
void pid_reset(PIDController* pid);

#endif // MOTOR_PID_H

