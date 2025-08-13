cmake_minimum_required(VERSION 3.30)
project(Sensors)

set(SENSORS_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(SENSORS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(SENSORS_SRCS
    ${SENSORS_SRC_DIR}/accelerometer.c
    ${SENSORS_SRC_DIR}/gyroscope.c
)

if(NOT DEFINED ACCELEROMETER_ENABLED)
    message(STATUS "Accelerometer not enabled, adding user stub.")
    list(APPEND SENSORS_SRCS ${SENSORS_SRC_DIR}/accelerometer_user.c)
endif()
if(NOT DEFINED GYROSCOPE_ENABLED)
    message(STATUS "Gyroscope not enabled, adding user stub.")
    list(APPEND SENSORS_SRCS ${SENSORS_SRC_DIR}/gyroscope_user.c)
endif()

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${SENSORS_INC_DIR}
)

target_sources(${PROJECT_NAME}
    INTERFACE
        ${SENSORS_SRCS}
)