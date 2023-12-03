#include <stdio.h>
#include "motor_pid.h"


void pid_init(PIDController* pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
}

float pid_update(PIDController* pid, float input, float dt) {
    pid->error = pid->setpoint - input;
    pid->integral += pid->error * dt;
    pid->derivative = (pid->error - pid->derivative) / dt;
    //float output = pid->Kp * pid->error ;
    float output = pid->Kp * pid->error + pid->Ki * pid->integral ;
    //float output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    return output;
}

void pid_set_setpoint(PIDController* pid, float setpoint) {
    pid->setpoint = setpoint;
}

void pid_reset(PIDController* pid) {
    pid->error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
}
