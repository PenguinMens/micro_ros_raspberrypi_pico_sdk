#include "encoder.h"
#include "quadrature_encoder.pio.h"
#include <math.h>


// NEED TO GIVE COPY RIGHT ############## TODO


// Global variables struct for encoder initialization
encoder_setup_t encoder_setup = {
    .PULSES_PER_REV = 12400, // 40 pulses per revolution
    .PULSES_PER_REV_GEAR = 3100, // 40 pulses per revolution
    .FRAME_TIME_MS = 100, // 50 Hz
    .WHEEL_DIAMETER = 0.2, // diameter of wheel in meters // rough calc 
    .WHEEL_BASE = 0.20 // distance between wheels in meters
};




uint ENCODER1;
uint ENCODER2;
uint PIN_AB;
uint PIN_CD;
PIO pio = pio0;
void reset_encoders()
{
pio_sm_exec(pio, ENCODER1, pio_encode_set(pio_y, 0));
pio_sm_exec(pio, ENCODER2, pio_encode_set(pio_y, 0));
}
int init_PIO_encoder(uint pin_ab, uint pin_cd, uint encoder1, uint encoder2){
    PIN_AB= pin_ab;
    PIN_CD = pin_cd;
    ENCODER1 = encoder1;
    ENCODER2 = encoder2;
    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, encoder1,  PIN_AB, 0);
    quadrature_encoder_program_init(pio, encoder2, PIN_CD, 0);
    reset_encoders();
    return 0;
}

int32_t get_encoder_count(uint encoder){
    return quadrature_encoder_get_count(pio,encoder);

}

float calc_stats(float time, Odemtry_values *vals){
    int32_t pulse_count_1 = get_encoder_count(ENCODER1);
    int32_t pulse_count_2 = get_encoder_count(ENCODER2);
    float rmp_1 = (pulse_count_1 * 60 *1000 )/ (encoder_setup.PULSES_PER_REV *time );
    float rmp_2 = (pulse_count_2 * 60 *1000) /  (encoder_setup.PULSES_PER_REV *time );
    vals->linear_velocity = ((rmp_1  + rmp_2) * (M_PI * encoder_setup.WHEEL_DIAMETER)) / (2 * 60);
    vals->angular_velocity = ((rmp_1 - rmp_2) * (M_PI * encoder_setup.WHEEL_DIAMETER)) / (encoder_setup.WHEEL_BASE * 60);
    float rpm3 = (pulse_count_1 * 60 *1000 )/ (encoder_setup.PULSES_PER_REV_GEAR *time );
    // x position calculation (m)
    vals->x += vals->linear_velocity * cos(vals->theta) * time/1000.0f;
    // y position calculation (m)
    vals->y += vals->linear_velocity * sin(vals->theta) * time/1000.0f;
    // theta position calculation (rad) frame time in ms
    vals->theta += vals->angular_velocity * time/1000.0f;
    reset_encoders();
    return ((rpm3) * (M_PI * 2 * .025)) / ( 60);
}
