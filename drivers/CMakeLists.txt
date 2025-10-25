cmake_minimum_required(VERSION 3.30)
project(Drivers)

option(DRIVER_A4988 "Enable A4988 driver" OFF)
option(DRIVER_LSM6DSV16X "Enable Lsm6dsv16x driver" OFF)

if(DRIVER_A4988)
    message(STATUS "A4988 driver enabled.")
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/a4988)
endif()

if(DRIVER_LSM6DSV16X)
    message(STATUS "Lsm6dsv16x driver enabled.")
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lsm6dsv16x)
endif()

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
        $<$<TARGET_EXISTS:A4988>:A4988>
        $<$<TARGET_EXISTS:Lsm6dsv16x>:Lsm6dsv16x>
)