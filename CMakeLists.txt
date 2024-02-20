cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
add_subdirectory(mpu6050)
link_directories(libmicroros)
add_executable(pico_micro_ros_example
    pico_micro_ros_example.c
    pico_uart_transport.c
    custom_pwm.c
    motor_control.c
    utils.c
    encoder.c
    motor_pid.c
    motor_calcs.c
    motor.c
)

pico_generate_pio_header(pico_micro_ros_example ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
target_link_libraries(pico_micro_ros_example
    pico_stdlib
    microros
    hardware_gpio hardware_pwm
    hardware_pio
    hardware_i2c
    haw_mpu6050
)

target_include_directories(pico_micro_ros_example PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_micro_ros_example 1)
pico_enable_stdio_uart(pico_micro_ros_example 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_micro_ros_example)
