cmake_minimum_required(VERSION 3.30)
project(Boards)

# Set board directory
set(BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${BOARD})

# STM32 HAL
add_subdirectory(${BOARD_DIR}/generated/cmake/stm32cubemx)

################################################################################
# BSP User
################################################################################
set(BSP_USER_DIR ${BOARD_DIR}/bsp_user)
set(BSP_USER_INC_DIR ${BSP_USER_DIR}/inc)
set(BSP_USER_SRC_DIR ${BSP_USER_DIR}/src)

set(BSP_USER_SRCS
    ${BSP_USER_SRC_DIR}/bsp_adc_user.c
    ${BSP_USER_SRC_DIR}/bsp_encoder_user.c
    ${BSP_USER_SRC_DIR}/bsp_gpio_user.c
    ${BSP_USER_SRC_DIR}/bsp_pwm_user.c
    ${BSP_USER_SRC_DIR}/bsp_tick_user.c
    ${BSP_USER_SRC_DIR}/bsp_uart_user.c
    ${BSP_USER_SRC_DIR}/bsp_user.c
)

add_library(BspUser INTERFACE)

target_include_directories(BspUser
    INTERFACE
        ${BSP_USER_INC_DIR}
)

target_sources(BspUser
    INTERFACE
        ${BSP_USER_SRCS}
)

target_link_libraries(BspUser
    INTERFACE
        stm32cubemx
)

################################################################################
# Rover config
################################################################################
set(ROVER_CONFIG_DIR ${BOARD_DIR}/rover_config)
set(ROVER_CONFIG_INC_DIR ${ROVER_CONFIG_DIR}/inc)
set(ROVER_CONFIG_SRC_DIR ${ROVER_CONFIG_DIR}/src)

set(ROVER_CONFIG_SRCS
    ${ROVER_CONFIG_SRC_DIR}/rover_4ws_config.c
    ${ROVER_CONFIG_SRC_DIR}/rover_imu_config.c
)

add_library(RoverConfig INTERFACE)

target_include_directories(RoverConfig
    INTERFACE
        ${ROVER_CONFIG_INC_DIR}
)

target_sources(RoverConfig
    INTERFACE
        ${ROVER_CONFIG_SRCS}
)

# TODO remove, everything should be accessible through Bsp
target_link_libraries(RoverConfig
    INTERFACE
        stm32cubemx
)