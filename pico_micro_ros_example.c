#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point32.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H   1
#include "custom_pwm.h"
#include "motor_control.h"
#include "motor_calcs.h"
#include "motor_pid.h"
#include "quadrature_encoder.pio.h"
#include "encoder.h"
#include "haw/MPU6050.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pio.h"
#include <inttypes.h>
#define ROS_MODE 0

float time_test = 0.0f;
float left_speed_target = 0;
float right_speed_target =0;
MotorStats motorStatsA;
MotorStats motorStatsB;
int32_t test_A = 0;
int32_t test_B = 0;
const uint LED_PIN = 25;
Odometry_values odo_vals;
// incrimenting puiblisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t publisher_imu;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t publisher_twist;
geometry_msgs__msg__Twist geo_msg;

rcl_publisher_t publisher_odomoter;
nav_msgs__msg__Odometry odo_msg;
//subscriber for input

geometry_msgs__msg__Twist geo_msg;
rcl_subscription_t pong_subscriber;

rcl_timer_t timer; // general timer current used for motor
rcl_timer_t timer2; // general timer used to measure encoder
rcl_timer_t timer3; // general timer used to measure encoder
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

mpu6050_t mpu6050;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void pong_subscription_callback(const void * msgin);
int controlMotorsPID(float dt);
void update_setpoint(double x, double z);
void cacluate_motor_stats(uint64_t last_call_time);
int init_mpu6050_vals();
int init_i2c();
void publish_imu_raw();
void timer2_callback(rcl_timer_t * timer, int64_t last_call_time);
void timer3_callback(rcl_timer_t * timer, int64_t last_call_time);
void publish_odo();
int data = 0;
//flag for mtoor direction tests
int flag = 1;

uint64_t last_trigger_time = 0;
uint64_t last_trigger_time2 = 0;
uint64_t last_trigger_time3 = 0;
uint64_t timer_interval_us = 20000;  // interval in microseconds (1 second)
const uint ENCODERA =0;
const uint ENCODERB = 1;
int main(){

    gpio_init(6);
    gpio_set_dir(6, GPIO_OUT);
    gpio_put(6, 1);
    init_i2c();
    float kp =  100;
    float ki =20;  
    float kd = 1;
    init_motors(15,16,17,14,18,19);
    pid_init(&motorStatsA.pid, kp, ki, kd, 0);
    pid_init(&motorStatsB.pid, kp, ki, kd, 0);
    const uint PIN_AB = 20;
    const uint PIN_CD = 12;

    init_PIO_encoder(PIN_AB, PIN_CD, ENCODERA,ENCODERB);
    mpu6050  = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);
    int check = init_mpu6050_vals();
    msg.data = check;
    int new_value, delta, old_value = 0;

    #if ROS_MODE
        rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
        );
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
        
        rclc_publisher_init_default(
            &publisher_odomoter,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg, Odometry),
            "pico_odometry");

        // subscriber for cmdvel
        rclc_subscription_init_best_effort(&pong_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg, Twist),
            "/cmd_vel");


        rclc_executor_init(&executor, &support.context,
            5,
            &allocator);

        // timer for mtoor controll
        rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(20),
            timer_callback);

        rcl_ret_t rc = rclc_timer_init_default(
            &timer2,
            &support,
            RCL_MS_TO_NS(1000),
            timer2_callback);

        rclc_timer_init_default(
            &timer3,
            &support,
            RCL_MS_TO_NS(20),
            timer3_callback);
        rclc_executor_add_timer(&executor, &timer);
        rclc_executor_add_timer(&executor, &timer2);
        rclc_executor_add_timer(&executor, &timer3);
        rclc_executor_add_subscription(&executor,
            &pong_subscriber,
            &geo_msg,
            &pong_subscription_callback,
            ON_NEW_DATA);

    #else    
     float setpoint = 0.05f;
     int i =0;
    stdio_init_all();

    

    #endif
   
    while (true)
    {

        #if ROS_MODE
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        #else
        uint64_t current_time = time_us_64();
        if (current_time - last_trigger_time >= timer_interval_us) {
            cacluate_motor_stats(current_time - last_trigger_time );
            last_trigger_time = current_time;
            printf("test");
            i++;
            if(i > 200)
            {
                update_setpoint(setpoint,0);
                // update_setpoint(setpoint,0);
                // setpoint = setpoint + 0.01;
                i = 0;
            }
        }

        
        #endif
        
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

    // if (timer == NULL) {
    //     return;
    // }
    // uint64_t current_time = time_us_64();
    
    // if (current_time - last_trigger_time >= timer_interval_us) {
    //     //msg.data =controlMotors(geo_msg.linear.x, geo_msg.angular.z);
    //     //rcl_publish(&publisher, &msg, NULL);
    //     last_trigger_time = current_time;

    // }


}

void timer2_callback(rcl_timer_t * timer, int64_t last_call_time){	

    // if (timer == NULL) {
    //     return;
    // }
    // mpu6050_event(&mpu6050);
    // publish_imu_raw(); 
}

void cacluate_motor_stats( uint64_t last_call_time)
{
    
    test_A = get_encoder_count_A();
    test_B = get_encoder_count_B();
    reset_encoders();
    time_test =  last_call_time/1000.0f;

    //printf("%" PRIu64 "\n", last_call_time);
    calc_stats(time_test, &odo_vals,test_A,test_B, &motorStatsA, &motorStatsB );
    controlMotorsPID(time_test);
}
void timer3_callback(rcl_timer_t * timer, int64_t last_call_time){


    cacluate_motor_stats(last_call_time/1000.0f);
    msg.data = last_call_time;
    rcl_publish(&publisher,&msg,NULL);
    odo_msg.pose.pose.orientation.w = time_test;
    
    publish_odo(last_call_time);

    
}

void update_setpoint(double x, double z)
{
    left_speed_target =  x - 0.5f*z*0.2;
    right_speed_target = x + 0.5f*z*0.2;
    pid_set_setpoint(&motorStatsA.pid, left_speed_target);
    pid_set_setpoint(&motorStatsB.pid, right_speed_target);

}
void pong_subscription_callback(const void * msgin){
    
   rcl_publish(&publisher_twist, &geo_msg, NULL);
    // update_setpoint(0.1 ,0.1);
   update_setpoint(geo_msg.linear.x,geo_msg.angular.z);
    
    // printf("turtle %f\n", geo_msg.linear.x);
}


int init_mpu6050_vals(){
    uint8_t temp = mpu6050_begin(&mpu6050);
    if(temp == 1){
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
        return (int)temp;
    }
    
        
}
void publish_odo(uint64_t current_time)
{
    odo_msg.header.frame_id.data = "odom";
    odo_msg.child_frame_id.data = "base_link";
    odo_msg.header.stamp.sec = (int32_t) (current_time/1000000);
    odo_msg.header.stamp.nanosec = (int32_t) (current_time%1000000)*1000;

    odo_msg.pose.pose.position.x = odo_vals.x; // x position
    odo_msg.pose.pose.position.y = odo_vals.y; // y position
    odo_msg.pose.pose.position.z = 0.0; // z position
    // set the orientation
    odo_msg.pose.pose.orientation.x = 0.0; // x orientation 
    odo_msg.pose.pose.orientation.y = 0.0; // y orientation
    odo_msg.pose.pose.orientation.z = odo_vals.theta;
    // odo_msg.pose.pose.orientation.w = 0.0; // w orientation
    // set the linear velocity 
    odo_msg.twist.twist.linear.x = odo_vals.linear_velocity;
    odo_msg.twist.twist.linear.y = left_speed_target;
    odo_msg.twist.twist.linear.z = right_speed_target;
    // set the angular velocity 
    odo_msg.twist.twist.angular.x = motorStatsA.rps;
    odo_msg.twist.twist.angular.y = motorStatsB.rps;
    odo_msg.twist.twist.angular.z = odo_vals.angular_velocity;
    rcl_publish(&publisher_odomoter, &odo_msg, NULL);
}
void publish_imu_raw() {
    uint64_t current_time = time_us_64();
    mpu6050_vectorf_t *accel = mpu6050_get_accelerometer(&mpu6050);
    mpu6050_vectorf_t *gyro = mpu6050_get_gyroscope(&mpu6050);
    imu_msg.header.frame_id.data = ("imu_link");
    imu_msg.header.stamp.sec = (int32_t) (current_time/1000000);

    imu_msg.angular_velocity.x =  gyro->x;
     imu_msg.angular_velocity.y = gyro->y;
     imu_msg.angular_velocity.z = gyro->z;
     imu_msg.linear_acceleration.x = accel->x;
     imu_msg.linear_acceleration.y = accel->y;
     imu_msg.linear_acceleration.z = accel->z;
    // publish imu data
    rcl_publish(&publisher_imu, &imu_msg, NULL);

}

int controlMotorsPID(float dt)
{
    static bool headersPrinted = false;
    // if (!headersPrinted) {
    //     printf("Time, Motor, PID Output, PWM, Velocity, KP, KI, KD\n");
    //     headersPrinted = true;
    // }

    float outputA = pid_update(&motorStatsA.pid, motorStatsA.velocity, dt);
    float pwmA =  fabs(outputA);
    if (pwmA > 100.0f) pwmA = 100.0f;
    
    float outputB = pid_update(&motorStatsB.pid, motorStatsB.velocity, dt);
    float pwmB = fabs(outputB);
    if (pwmB > 100.0f) pwmB = 100.0f;
    

    // Assuming `getCurrentTime()` is a function that returns the current time in milliseconds or another unit

    printf("%f, A, %f, %f, %f, %f, %f ,%d\n", dt, outputA, pwmA, motorStatsA.velocity,left_speed_target,  odo_vals.linear_velocity, test_A);
    // printf("%f, B, %f, %f, %f, %f, %f\n", dt, outputB, pwmB, motorStatsB.velocity, right_speed_target,  odo_vals.linear_velocity);
    printf("%f, B, %f, %f, %f, %f, %f, %d\n", dt, outputB, pwmB, motorStatsB.velocity,  right_speed_target,  odo_vals.linear_velocity, test_B);
    controlLeftMotor(outputA, pwmA);
    controlRightMotor(outputB, pwmB);

    return 0;
}