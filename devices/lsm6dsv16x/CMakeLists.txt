cmake_minimum_required(VERSION 3.30)
project(Lsm6dsv16x)

set(ACCELEROMETER ON CACHE BOOL "Enable accelerometer")
set(GYROSCOPE ON CACHE BOOL "Enable gyroscope")

################################################################################
# LSM6DSV16X driver
################################################################################
set(LSM6DSV16X_DIR ${CMAKE_SOURCE_DIR}/external/lsm6dsv16x-pid)
add_library(lsm6dsv16x-driver INTERFACE)
target_sources(lsm6dsv16x-driver
    INTERFACE
        ${LSM6DSV16X_DIR}/lsm6dsv16x_reg.c
)
target_include_directories(lsm6dsv16x-driver
    INTERFACE
        ${LSM6DSV16X_DIR}
)

################################################################################
# LSM6DSV16X
################################################################################
set(LSM6DSV16X_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(LSM6DSV16X_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(LSM6DSV16X_SRCS
    ${LSM6DSV16X_SRC_DIR}/accelerometer_user.c
    ${LSM6DSV16X_SRC_DIR}/gyroscope_user.c
    ${LSM6DSV16X_SRC_DIR}/lsm6dsv16x.c
)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${LSM6DSV16X_INC_DIR}
)

target_sources(${PROJECT_NAME}
    INTERFACE
        ${LSM6DSV16X_SRCS}
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
        lsm6dsv16x-driver
)