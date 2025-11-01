cmake_minimum_required(VERSION 3.30)
project(CAVeBoard-Mini)

################################################################################
# BSP
################################################################################
set(BSP_ENCODER ON CACHE BOOL "Enable BSP encoders")
set(BSP_GPIO ON CACHE BOOL "Enable BSP GPIO")
set(BSP_MOTOR ON CACHE BOOL "Enable BSP motors")
set(BSP_PWM ON CACHE BOOL "Enable BSP PWM")
set(BSP_SERVO ON CACHE BOOL "Enable BSP servos")
set(BSP_TIMER ON CACHE BOOL "Enable BSP timers")
set(BSP_UART ON CACHE BOOL "Enable BSP UART")

################################################################################
# Devices
################################################################################
set(ACCELEROMETER ON CACHE BOOL "Enable accelerometer")
set(GYROSCOPE ON CACHE BOOL "Enable gyroscope")

################################################################################
# Drivers
################################################################################
set(DRIVER_A4988 ON CACHE BOOL "Enable A4988 driver")
set(DRIVER_LSM6DSV16X ON CACHE BOOL "Enable Lsm6dsv16x driver")

################################################################################
# Bots
################################################################################
set(ROVER_4WD ON CACHE BOOL "Enable Rover 4WD bot configuration")