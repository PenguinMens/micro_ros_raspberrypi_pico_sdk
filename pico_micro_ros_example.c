#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point32.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"
#include "haw/MPU6050.h"

#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
typedef struct MotorPins{
    uint PWM_PIN;
    uint IN1_PIN;
    uint IN2_PIN;
} motorPins;


const uint LED_PIN = 25;
const motorPins motorB = {15,16,17};
const motorPins motorA = {14,18,19};
// incrimenting puiblisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t publisher_imu;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t publisher_twist;
geometry_msgs__msg__Twist geo_msg;

//subscriber for input

geometry_msgs__msg__Twist geo_msg;
rcl_subscription_t pong_subscriber;

rcl_timer_t timer; // general timer current used for motor
rcl_timer_t timer2; // general timer used to measure encoder
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

mpu6050_t mpu6050;

void controlMotors(float linear_velocity, float angular_velocity) ;
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void pong_subscription_callback(const void * msgin);
uint32_t pwm_set_freq_duty(uint slice_num,
       uint chan,uint32_t f, int d);
float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int init_mpu6050_vals();
int init_i2c();
void timer2_callback(rcl_timer_t * timer, int64_t last_call_time);
uint16_t pwmRight;
uint16_t pwmLeft;
int data = 0;
uint slice_num_B = 0;
uint chan_B = 0;
uint slice_num_A = 0;
uint chan_A = 0;
//flag for mtoor direction tests
int flag = 1;
int PWM_MOTOR_MIN = 0;
int PWM_MOTOR_MAX = 100;
uint64_t last_trigger_time = 0;
uint64_t last_trigger_time2 = 0;
uint64_t timer_interval_us = 1000000;  // interval in microseconds (1 second)
int main(){
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    init_i2c();

    mpu6050  = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);
    int check = init_mpu6050_vals();
    if(check){

    }
    const uint PIN_AB = 20;
    int new_value, delta, old_value = 0;
    // should be in init function
    for(int i = 16; i < 20; i++){
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUT);
    }
    
    gpio_set_function(motorB.PWM_PIN, GPIO_FUNC_PWM);
    slice_num_B = pwm_gpio_to_slice_num(motorB.PWM_PIN);
    chan_B = pwm_gpio_to_channel(motorB.PWM_PIN);
    pwm_set_enabled(slice_num_B, true);

    gpio_set_function(motorA.PWM_PIN, GPIO_FUNC_PWM);
    slice_num_A = pwm_gpio_to_slice_num(motorA.PWM_PIN);
    chan_A = pwm_gpio_to_channel(motorA.PWM_PIN);
    pwm_set_enabled(slice_num_A, true);



    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK){
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    // std msg currently used for debugging : motor speeds
    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");


    rclc_publisher_init_default(
        &publisher_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),

        "pico_imu");

    rclc_publisher_init_default(
        &publisher_twist,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg, Twist),
        "pico_twist");
    

    // subscriber for cmdvel
	rclc_subscription_init_best_effort(&pong_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg, Twist),
        "/cmd_vel");


	rclc_executor_init(&executor, &support.context,
        3,
        &allocator);

    // timer for mtoor controll
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1),
        timer_callback);

    rcl_ret_t rc = rclc_timer_init_default(
        &timer2,
        &support,
        RCL_MS_TO_NS(1000),
        timer2_callback);

	rclc_executor_add_timer(&executor, &timer);
    rc = rclc_executor_add_timer(&executor, &timer2);
    if (rc != RCL_RET_OK) {

    
    }
    rclc_executor_add_subscription(&executor,
        &pong_subscriber,
        &geo_msg,
        &pong_subscription_callback,
        ON_NEW_DATA);

    gpio_put(LED_PIN, 1);
    gpio_put(motorB.IN1_PIN, 0);
    gpio_put(motorB.IN2_PIN, 1);
    msg.data = 0;

    PIO pio = pio0;
    const uint sm = 0;

    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm, offset, PIN_AB, 0);
    
    mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);

    while (true)
    {

        // need to put this in a timer 
        uint64_t current_time = time_us_64();
        if (current_time - last_trigger_time2 >= timer_interval_us) {
            new_value = quadrature_encoder_get_count(pio, sm);
            delta = new_value - old_value;
            old_value = new_value;
            last_trigger_time2 = current_time;

            // printf("position %8d, delta %6d\n", new_value, delta);
        }

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    }
    return 0;
}

int init_i2c(){
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Don't forget the pull ups! | Or use external ones
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    return 0;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){	

    if (timer == NULL) {
        return;
    }
    // if(current_state)
    // {
    //     gpio_pin_set(led, PIN, 0);
    //     current_state = !current_state;
    // }
    // else if (!current_state)
    // {
    //     gpio_pin_set(led, PIN, 1);
    //     current_state = !current_state;
    // }
    // msg.data = data;
    
    uint64_t current_time = time_us_64();

    
    if (current_time - last_trigger_time >= timer_interval_us) {
        // msg.data = pwmRight;
        // rcl_publish(&publisher, &msg, NULL);
        controlMotors(geo_msg.linear.x, geo_msg.angular.z);
        
        // pwm_set_freq_duty(slice_num_B,chan_B, 10000, data);
        // pwm_set_enabled(slice_num_B, true);
        last_trigger_time = current_time;
        // if (flag) {
        //     data++;
        //     if (data == 100) {
        //         gpio_put(motorB.IN1_PIN, 1);
        //         gpio_put(motorB.IN2_PIN, 0);
        //         flag = 0;  // Switch to decrement mode when data reaches 100
        //     }
        // } else {
        //     data--;
        //     if (data == 0) {
        //         gpio_put(motorB.IN1_PIN, 0);
        //         gpio_put(motorB.IN2_PIN, 1);  
        //         flag = 1;  // Switch back to increment mode when data reaches 0
        //     }
        // }
        
    }

        
    // if (pwm_dev == NULL) {
    // // Handle error, device not found
    //     msg.data = 266;
    //     return;
    // }   
    // if(pwm_pin_set_usec(pwm_dev, 42, 255, data, 0) < 0)
    // {
    //     msg.data = 266;
    // }
    
    // float linear = constrain(geo_msg.linear.x, -1, 1);
    // float angular = constrain(geo_msg.angular.z, -1, 1);

}

void timer2_callback(rcl_timer_t * timer, int64_t last_call_time){	

    if (timer == NULL) {
        return;
    }
    mpu6050_event(&mpu6050);

    // Pointers to float vectors with all the results
    // mpu6050_vectorf_t *accel = mpu6050_get_accelerometer(&mpu6050);
    // mpu6050_vectorf_t *gyro = mpu6050_get_gyroscope(&mpu6050);
    publish_imu_raw();
}

void controlMotors(float linear_velocity, float angular_velocity) {
    // Calculate motor speeds based on linear and angular velocities\
    float left_speed = linear_velocity - angular_velocity;
    //float right_speed = linear_velocity + angular_velocity;
    float linear = constrain(linear_velocity, -1, 1);
    float angular = constrain(angular_velocity, -1, 1);
    // Map the speeds to the range [-1, 1]
    float left_speed = (linear - angular) / 2.0f;
    float right_speed = (linear + angular) / 2.0f;

    pwmLeft = (uint16_t) fmap(fabs(left_speed), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    pwmRight = (uint16_t) fmap(fabs(right_speed), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    // Perform motor control based on the calculated speeds
    // (Assuming you have already set up the GPIO or other pin control)
    // Example using pico-sdk:
    msg.data = (int)(right_speed* 1000000);
    rcl_publish(&publisher, &msg, NULL);
    if(left_speed == 0)// stop speed
    {
        gpio_put(motorB.IN1_PIN, 1);
        gpio_put(motorB.IN2_PIN, 1);
        // PORTC |= (1<<PC0);
        // PORTC |= (1<<PC1);
    }
    else if(left_speed>0) //if lm is positive
    {
        gpio_put(motorB.IN1_PIN, 1);
        gpio_put(motorB.IN2_PIN, 0);
        // PORTC |= (1<<PC0);
        // PORTC &= ~(1<<PC1);
    }
    else
    {
        //set direction reverse
        // PORTC &= ~(1<<PC0);
        // PORTC |= (1<<PC1);
        gpio_put(motorB.IN1_PIN, 0);
        gpio_put(motorB.IN2_PIN, 1);
    }

    if(right_speed == 0)
    {

        gpio_put(motorA.IN1_PIN, 1);
        gpio_put(motorA.IN2_PIN,  1);
        // PORTC |= (1<<PC2);
        // PORTC |= (1<<PC3);
    }
    else if(right_speed>0) //if rm is positive
    {
        //set direction forwards
        gpio_put(motorA.IN1_PIN, 1);
        gpio_put(motorA.IN2_PIN,  0);
        // PORTC |= (1<<PC2);
        // PORTC &= ~(1<<PC3);
    }
    else
    {
        //set direction reverse
        gpio_put(motorA.IN1_PIN, 0);
        gpio_put(motorA.IN2_PIN,  1);
        // PORTC &= ~(1<<PC2);
        // PORTC |= (1<<PC3);
        }
    // gpio_put(motorB.IN1_PIN, left_speed > 0);
    // gpio_put(motorB.IN2_PIN, left_speed < 0);
    // gpio_put(motorA.IN1_PIN, right_speed > 0);
    // gpio_put(motorA.IN2_PIN, right_speed < 0);

    // Optionally, you can adjust the PWM duty cycle for finer control
    // (Assuming PWM is available on your motor driver and configured)
    // Example using pico-sdk:
        pwm_set_freq_duty(slice_num_B,chan_B, 10000, pwmLeft);
        pwm_set_freq_duty(slice_num_A,chan_A, 10000, pwmRight);

    // pwm_set_gpio_level(LEFT_MOTOR_PWM_PIN, (uint16_t)(fabs(left_speed) * PWM_RANGE));
    // pwm_set_gpio_level(RIGHT_MOTOR_PWM_PIN, (uint16_t)(fabs(right_speed) * PWM_RANGE));
}

void pong_subscription_callback(const void * msgin){
   
    rcl_publish(&publisher_twist, &geo_msg, NULL);
    printf("turtle %f\n", geo_msg.linear.x);
}

uint32_t pwm_set_freq_duty(uint slice_num,
       uint chan,uint32_t f, int d)
{
 uint32_t clock = 125000000;
 uint32_t divider16 = clock / f / 4096 + 
                           (clock % (f * 4096) != 0);
 if (divider16 / 16 == 0)
 divider16 = 16;
 uint32_t wrap = clock * 16 / divider16 / f - 1;
 pwm_set_clkdiv_int_frac(slice_num, divider16/16,
                                     divider16 & 0xF);
 pwm_set_wrap(slice_num, wrap);
 pwm_set_chan_level(slice_num, chan, wrap * d / 100);
 return wrap;
}

int init_mpu6050_vals(){
    if(mpu6050_begin(&mpu6050)){
        // Set scale of gyroscope
        mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        // Set range of accelerometer
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

        // Enable temperature, gyroscope and accelerometer readings
        mpu6050_set_temperature_measuring(&mpu6050, true);
        mpu6050_set_gyroscope_measuring(&mpu6050, true);
        mpu6050_set_accelerometer_measuring(&mpu6050, true);

        // Enable free fall, motion and zero motion interrupt flags
        mpu6050_set_int_free_fall(&mpu6050, false);
        mpu6050_set_int_motion(&mpu6050, false);
        mpu6050_set_int_zero_motion(&mpu6050, false);

        // Set motion detection threshold and duration
        mpu6050_set_motion_detection_threshold(&mpu6050, 2);
        mpu6050_set_motion_detection_duration(&mpu6050, 5);

        // Set zero motion detection threshold and duration
        mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
        mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
        return 0;
    }
    else{
        return 1;
    }
    
        
}

void publish_imu_raw() {
    uint64_t current_time = time_us_64();
    mpu6050_vectorf_t *accel = mpu6050_get_accelerometer(&mpu6050);
    mpu6050_vectorf_t *gyro = mpu6050_get_gyroscope(&mpu6050);
    imu_msg.header.frame_id.data = ("imu_link");
    imu_msg.header.stamp.sec = current_time  / 1000000;

    imu_msg.angular_velocity.x =  gyro->x;
    imu_msg.angular_velocity.y = gyro->y;
    imu_msg.angular_velocity.z = gyro->z;
    imu_msg.linear_acceleration.x = accel->x;
    imu_msg.linear_acceleration.y = accel->y;
    imu_msg.linear_acceleration.z = accel->z;
    // publish imu data
    rcl_publish(&publisher_imu, &imu_msg, NULL);

}