#include "motor_calcs.h"  // Include motor calculation functions and structures
#include <math.h>          // Include math functions
#include "motor_control.h" // Include motor control functions and structures

// Global variables struct for encoder initialization
encoder_setup_t encoder_setup = {
    .PULSES_PER_REV = 12400,       // Number of encoder pulses per revolution (default: 12400)
    .PULSES_PER_REV_GEAR = 3100,   // Number of encoder pulses per revolution with gear (default: 3100)
    .FRAME_TIME_MS = 100,          // Frame time in milliseconds (default: 100)
    .WHEEL_DIAMETER = 0.2,         // Diameter of the wheel in meters (default: 0.2)
    .WHEEL_BASE = 0.20             // Distance between wheels in meters (default: 0.20)
};

#define WINDOW_SIZE 10  // Size of the moving average window

// Variables for moving average calculation of motor velocities
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

// Function to calculate motor statistics and update odometry values
void calc_stats(float time, Odometry_values *vals, int32_t ENCODER1_TICKS, int32_t ENCODER2_TICKS, MotorStats *motorStatsA, MotorStats *motorStatsB) {
    // Calculate rotations per minute (RPM) for each motor
    int32_t pulse_count_1 = ENCODER1_TICKS;
    int32_t pulse_count_2 = ENCODER2_TICKS;
    float rmp_1 = (pulse_count_1 * 60 * 1000) / (encoder_setup.PULSES_PER_REV * time);
    float rmp_2 = (pulse_count_2 * 60 * 1000) / (encoder_setup.PULSES_PER_REV * time);

    // Update motor statistics with calculated RPM values
    motorStatsA->rps = rmp_1;
    motorStatsB->rps = rmp_2;

    // Calculate linear velocity for each motor
    float v1 = (rmp_1 * M_PI * encoder_setup.WHEEL_DIAMETER) / 60;
    float v2 = (rmp_2 * M_PI * encoder_setup.WHEEL_DIAMETER) / 60;

    // Moving average calculation for motor velocities
    SUM = SUM - READINGS[INDEX];        // Remove the oldest entry from the sum
    VALUE = v1;                         // Read the next sensor value
    READINGS[INDEX] = VALUE;            // Add the newest reading to the window
    SUM = SUM + VALUE;                  // Add the newest reading to the sum
    INDEX = (INDEX + 1) % WINDOW_SIZE;  // Increment the index, and wrap to 0 if it exceeds the window size
    //AVERAGED = SUM / WINDOW_SIZE;       // Calculate the average velocity
    AVERAGED2 = v1;
    SUM2 = SUM2 - READINGS2[INDEX2];       // Remove the oldest entry from the sum
    VALUE2 = v2;                           // Read the next sensor value
    READINGS2[INDEX2] = VALUE2;            // Add the newest reading to the window
    SUM2 = SUM2 + VALUE2;                  // Add the newest reading to the sum
    INDEX2 = (INDEX2 + 1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
    //AVERAGED2 = SUM2 / WINDOW_SIZE;        // Calculate the average velocity
    AVERAGED2 = v2;

    // Update motor statistics with averaged velocity values
    motorStatsA->velocity = AVERAGED;
    motorStatsB->velocity = AVERAGED2;

    // Calculate linear and angular velocities for odometry
    vals->linear_velocity = (AVERAGED + AVERAGED2) / 2;
    vals->angular_velocity = ((AVERAGED - AVERAGED2) * encoder_setup.WHEEL_DIAMETER) / (2 * encoder_setup.WHEEL_BASE);

    // Calculate new position and orientation based on velocities
    vals->x += vals->linear_velocity * cos(vals->theta) * time / 1000.0f;
    vals->y += vals->linear_velocity * sin(vals->theta) * time / 1000.0f;
    vals->theta += vals->angular_velocity * time / 1000.0f;
}
