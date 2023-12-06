#include "motor_calcs.h"
#include <math.h>
#include "motor_control.h"

// Global variables struct for encoder initialization
encoder_setup_t encoder_setup = {
    .PULSES_PER_REV = 12400, // 40 pulses per revolution
    .PULSES_PER_REV_GEAR = 3100, // 40 pulses per revolutison
    .FRAME_TIME_MS = 100, // 50 Hz
    .WHEEL_DIAMETER = 0.2, // diameter of wheel in meters // rough calc 
    .WHEEL_BASE = 0.20 // distance between wheels in meters
};

#define WINDOW_SIZE 10

int INDEX = 0;
float VALUE = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;

int INDEX2 = 0;
float VALUE2 = 0;
float SUM2 = 0;
float READINGS2[WINDOW_SIZE];
float AVERAGED2 = 0;
void calc_stats(float time, Odemtry_values *vals, int32_t ENCODER1_TICKS,int32_t ENCODER2_TICKS, MotorStats *motorStatsA,MotorStats *motorStatsB  ){
  
    int32_t pulse_count_1 = ENCODER1_TICKS;
    int32_t pulse_count_2 = ENCODER2_TICKS;
    float rmp_1 = (pulse_count_1 * 60 *1000 )/ (encoder_setup.PULSES_PER_REV *time );
    float rmp_2 = (pulse_count_2 * 60 *1000) /  (encoder_setup.PULSES_PER_REV *time );
    motorStatsA->rps = rmp_1 ;
    motorStatsB->rps = rmp_2 ;
    float v1 = (rmp_1 * M_PI * encoder_setup.WHEEL_DIAMETER)/60;
    float v2 = (rmp_2 * M_PI * encoder_setup.WHEEL_DIAMETER)/60;
    SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
    VALUE = v1;        // Read the next sensor value
    READINGS[INDEX] = VALUE;           // Add the newest reading to the window
    SUM = SUM + VALUE;                 // Add the newest reading to the sum
    INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
    AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

    SUM2 = SUM2 - READINGS2[INDEX2];       // Remove the oldest entry from the sum
    VALUE2 = v2;        // Read the next sensor value
    READINGS2[INDEX2] = VALUE2;           // Add the newest reading to the window
    SUM2 = SUM2 + VALUE2;                 // Add the newest reading to the sum
    INDEX2 = (INDEX2+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
    AVERAGED2 = SUM2 / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
    motorStatsA->velocity = AVERAGED;
    motorStatsB->velocity = AVERAGED2;
    vals->linear_velocity = (AVERAGED + AVERAGED2)/ 2;
    vals->angular_velocity = ((AVERAGED- AVERAGED2) * encoder_setup.WHEEL_DIAMETER)/(2* encoder_setup.WHEEL_BASE);

    float rpm3 = (pulse_count_1 * 60 *1000 )/ (encoder_setup.PULSES_PER_REV_GEAR *time );
    // x position calculation (m)



    vals->x += vals->linear_velocity * cos(vals->theta) * time/1000.0f;

    // y position calculation (m)
    vals->y += vals->linear_velocity * sin(vals->theta) * time/1000.0f;
    // theta position calculation (rad) frame time in ms
    vals->theta += vals->angular_velocity * time/1000.0f;

 
}
