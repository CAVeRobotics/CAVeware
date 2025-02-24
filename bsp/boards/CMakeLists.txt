cmake_minimum_required(VERSION 3.30)
project(BspUser)

# Set board directory
set(BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${BOARD})

# STM32 HAL
add_subdirectory(${BOARD_DIR}/generated/cmake/stm32cubemx)

################################################################################
# BSP User library
################################################################################
set(BSP_USER_DIR ${BOARD_DIR}/bsp_user)
set(BSP_USER_INC_DIR ${BSP_USER_DIR}/inc)
set(BSP_USER_SRC_DIR ${BSP_USER_DIR}/src)

set(BSP_USER_SRCS
    ${BSP_USER_SRC_DIR}/bsp_encoder_user.c
    ${BSP_USER_SRC_DIR}/bsp_pwm_user.c
    ${BSP_USER_SRC_DIR}/bsp_tick_user.c
    ${BSP_USER_SRC_DIR}/bsp_uart_user.c
    ${BSP_USER_SRC_DIR}/bsp_user.c
)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${BSP_USER_INC_DIR}
)

target_sources(${PROJECT_NAME}
    INTERFACE
        ${BSP_USER_SRCS}
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
        stm32cubemx
)