cmake_minimum_required(VERSION 3.30)
project(Bsp)

################################################################################
# BSP library
################################################################################
set(BSP_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(BSP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(BSP_SRCS
    ${BSP_SRC_DIR}/bsp.c
    ${BSP_SRC_DIR}/bsp_adc.c
    ${BSP_SRC_DIR}/bsp_encoder.c
    ${BSP_SRC_DIR}/bsp_gpio.c
    ${BSP_SRC_DIR}/bsp_logger.c
    ${BSP_SRC_DIR}/bsp_motor.c
    ${BSP_SRC_DIR}/bsp_pwm.c
    ${BSP_SRC_DIR}/bsp_servo.c
    ${BSP_SRC_DIR}/bsp_tick.c
    ${BSP_SRC_DIR}/bsp_uart.c
)

add_library(${PROJECT_NAME} INTERFACE)

target_compile_definitions(${PROJECT_NAME}
    INTERFACE
        $<$<CONFIG:Debug>:BSP_LOGGER>
        $<$<CONFIG:Debug>:BSP_LOGGER_USER_CUSTOM_LOGGER>
        $<$<STREQUAL:${BOARD},CAVeBoard>:BOARD_CAVEBOARD>
        $<$<STREQUAL:${BOARD},Nucleo-64>:BOARD_NUCLEO_64>
        $<$<STREQUAL:${BOARD},STM32-E407>:BOARD_STM32_E407>
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${BSP_INC_DIR}
)

target_sources(${PROJECT_NAME}
    INTERFACE
        ${BSP_SRCS}
)
