cmake_minimum_required(VERSION 3.30)
project(Devices)

set(DEVICES_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(DEVICES_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(DEVICES_SRCS)

option(ACCELEROMETER "Enable accelerometer" OFF)
option(GYROSCOPE "Enable gyroscope" OFF)

if(ACCELEROMETER)
    message(STATUS "Accelerometer enabled.")
    list(APPEND DEVICES_SRCS ${DEVICES_SRC_DIR}/accelerometer.c)
endif()
if(GYROSCOPE)
    message(STATUS "Gyroscope enabled.")
    list(APPEND DEVICES_SRCS ${DEVICES_SRC_DIR}/gyroscope.c)
endif()

add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${DEVICES_INC_DIR}
)

target_sources(${PROJECT_NAME}
    PRIVATE
        ${DEVICES_SRCS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Bsp
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        "-Werror"
)

if(COMMAND collect_sources)
    collect_sources(${PROJECT_NAME})
endif()
if(COMMAND collect_include_directories)
    collect_include_directories(${PROJECT_NAME})
endif()