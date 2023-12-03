// encoder_init.h

#ifndef MOTOR_CALCS_H
#define MOTOR_CALCS_H

#include <stdint.h>
#include "motor_pid.h"
// Function declaration


// TODO NEED TO GIVE COPYRIGHT
typedef struct {
  int RESOLUTION;
  int PULSES_PER_REV;
  int PULSES_PER_REV_GEAR;
  int FRAME_TIME_MS;
  float WHEEL_DIAMETER;
  float WHEEL_BASE;
} encoder_setup_t;

typedef struct {
    float x;     // X-coordinate of the car's position (in meters)
    float y;     // Y-coordinate of the car's position (in meters)
    float theta;
    float linear_velocity;  // Linear velocity of the car (in m/s)
    float angular_velocity; // Angular velocity of the car (in rad/s)
} Odemtry_values;

typedef struct MotorStats{
    float velocity;
    float rps;
    PIDController pid;
} MotorStats;

void calc_stats(float time, Odemtry_values *vals, int32_t ENCODER1_TICKS,int32_t ENCODER2_TICKS, MotorStats *motorStatsA,MotorStats *motorStatsB  );

#endif // ENCODER_INIT_H