// encoder_init.h

#ifndef ENCODER_INIT_H
#define ENCODER_INIT_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
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
    double x;     // X-coordinate of the car's position (in meters)
    double y;     // Y-coordinate of the car's position (in meters)
    double theta;
    double linear_velocity;  // Linear velocity of the car (in m/s)
    double angular_velocity; // Angular velocity of the car (in rad/s)
} Odemtry_values;
float calc_stats(float time, Odemtry_values *vals);
int init_PIO_encoder(uint PIN_AB, uint PIN_CD, uint encode1, uint encode2);
int32_t get_encoder_count(uint encoder);
#endif // ENCODER_INIT_H