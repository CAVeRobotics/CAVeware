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
    ${BSP_USER_SRC_DIR}/bsp_logger_user.c
    ${BSP_USER_SRC_DIR}/bsp_tick_user.c
    ${BSP_USER_SRC_DIR}/bsp_user.c
)

if(BSP_ADC)
    list(APPEND BSP_USER_SRCS ${BSP_USER_SRC_DIR}/bsp_adc_user.c)
endif()
if(BSP_ENCODER)
    list(APPEND BSP_USER_SRCS ${BSP_USER_SRC_DIR}/bsp_encoder_user.c)
endif()
if(BSP_GPIO)
    list(APPEND BSP_USER_SRCS ${BSP_USER_SRC_DIR}/bsp_gpio_user.c)
endif()
if(BSP_PWM)
    list(APPEND BSP_USER_SRCS ${BSP_USER_SRC_DIR}/bsp_pwm_user.c)
endif()
if(BSP_TIMER)
    list(APPEND BSP_USER_SRCS ${BSP_USER_SRC_DIR}/bsp_timer_user.c)
endif()
if(BSP_UART)
    list(APPEND BSP_USER_SRCS ${BSP_USER_SRC_DIR}/bsp_uart_user.c)
endif()

add_library(BspUser INTERFACE)

target_compile_definitions(BspUser
    INTERFACE
        $<$<CONFIG:Debug>:BSP_LOGGER>
        $<$<CONFIG:Debug>:BSP_LOGGER_USER_CUSTOM_LOGGER>
        # TODO CVW-55
        $<$<STREQUAL:${BOARD},CAVeBoard>:BOARD_CAVEBOARD>
        $<$<STREQUAL:${BOARD},CAVeBoard-Mini>:BOARD_CAVEBOARD_MINI>
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