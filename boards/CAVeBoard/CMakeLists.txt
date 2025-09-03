cmake_minimum_required(VERSION 3.30)
project(CAVeBoard)

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
set(ROVER_4WS ON CACHE BOOL "Enable Rover 4WS bot configuration")