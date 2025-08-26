cmake_minimum_required(VERSION 3.30)
project(Boards)

# Set board directory
set(BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${BOARD})
add_subdirectory(${BOARD_DIR})

################################################################################
# STM32 HAL
################################################################################
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
    ${BSP_USER_SRC_DIR}/bsp_logger_user.c
    ${BSP_USER_SRC_DIR}/bsp_pwm_user.c
    ${BSP_USER_SRC_DIR}/bsp_tick_user.c
    ${BSP_USER_SRC_DIR}/bsp_uart_user.c
    ${BSP_USER_SRC_DIR}/bsp_user.c
)

add_library(BspUser INTERFACE)

target_compile_definitions(BspUser
    INTERFACE
        $<$<CONFIG:Debug>:BSP_LOGGER>
        $<$<CONFIG:Debug>:BSP_LOGGER_USER_CUSTOM_LOGGER>
        # TODO CVW-55
        $<$<STREQUAL:${BOARD},CAVeBoard>:BOARD_CAVEBOARD>
        $<$<STREQUAL:${BOARD},Nucleo-64>:BOARD_NUCLEO_64>
        $<$<STREQUAL:${BOARD},STM32-E407>:BOARD_STM32_E407>
)

target_include_directories(BspUser
    INTERFACE
        ${BSP_USER_INC_DIR}
)

target_sources(BspUser
    INTERFACE
        ${BSP_USER_SRCS}
)

################################################################################
# BSP
################################################################################
add_subdirectory(${CMAKE_SOURCE_DIR}/bsp ${CMAKE_BINARY_DIR}/bsp)

################################################################################
# Cavebot User
################################################################################
set(CAVEBOT_USER_DIR ${BOARD_DIR}/cavebot_user)
set(CAVEBOT_USER_INC_DIR ${CAVEBOT_USER_DIR}/inc)
set(CAVEBOT_USER_SRC_DIR ${CAVEBOT_USER_DIR}/src)

set(CAVEBOT_USER_SRCS
    ${CAVEBOT_USER_SRC_DIR}/cavebot_user.c
)

add_library(CavebotUser INTERFACE)

target_include_directories(CavebotUser
    INTERFACE
        ${CAVEBOT_USER_INC_DIR}
)

target_sources(CavebotUser
    INTERFACE
        ${CAVEBOT_USER_SRCS}
)