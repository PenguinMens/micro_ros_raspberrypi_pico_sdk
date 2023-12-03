#include "encoder.h"
#include "quadrature_encoder.pio.h"
#include <math.h>


// NEED TO GIVE COPY RIGHT ############## TODO



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

int32_t get_encoder_count_A(){
    return quadrature_encoder_get_count(pio,ENCODER1);

}
int32_t get_encoder_count_B(){
    return quadrature_encoder_get_count(pio,ENCODER2);

}
int32_t encoder_read_and_reset(uint encoder){
    int32_t count = quadrature_encoder_get_count(pio,encoder);
    pio_sm_exec(pio, encoder, pio_encode_set(pio_y, 0));
    return count;
}
int32_t encoder_read_and_reset_A(){
    int32_t count = quadrature_encoder_get_count(pio,ENCODER1);
    pio_sm_exec(pio, ENCODER1, pio_encode_set(pio_y, 0));
    return count;
}
int32_t encoder_read_and_reset_B(){
    int32_t count = quadrature_encoder_get_count(pio,ENCODER2);
    pio_sm_exec(pio, ENCODER2, pio_encode_set(pio_y, 0));
    return count;
}