cmake_minimum_required(VERSION 3.30)
project(CAVeBoard-Mini)

################################################################################
# Devices
################################################################################
set(ACCELEROMETER ON CACHE BOOL "Enable accelerometer")
set(GYROSCOPE ON CACHE BOOL "Enable gyroscope")

################################################################################
# Drivers
################################################################################
set(LSM6DSV16X ON CACHE BOOL "Enable Lsm6dsv16x")

################################################################################
# Bots
################################################################################
set(ROVER_4WD ON CACHE BOOL "Enable Rover 4WD bot configuration")