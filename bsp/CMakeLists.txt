cmake_minimum_required(VERSION 3.30)
project(Bsp)

################################################################################
# BSP library
################################################################################
set(BSP_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(BSP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


# TODO CVW-54
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

set_source_files_properties(
    ${BSP_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

get_target_property(BSP_USER_SRCS BspUser INTERFACE_SOURCES)
if(BSP_USER_SRCS)
    set_source_files_properties(
        ${BSP_USER_SRCS}
        PROPERTIES
        COMPILE_OPTIONS "-Werror"
    )
endif()

add_library(${PROJECT_NAME})

target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>
        $<TARGET_PROPERTY:BspUser,INTERFACE_COMPILE_DEFINITIONS>
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:BspUser,INTERFACE_INCLUDE_DIRECTORIES>
        ${BSP_INC_DIR}
)

target_sources(${PROJECT_NAME}
    PRIVATE
        $<TARGET_PROPERTY:stm32cubemx,INTERFACE_SOURCES>
        $<TARGET_PROPERTY:BspUser,INTERFACE_SOURCES>
        ${BSP_SRCS}
)

if(COMMAND append_sources)
    append_sources("${BSP_SRCS}")
endif()
if(COMMAND collect_sources)
    collect_sources(BspUser)
endif()
if(COMMAND append_include_directories)
    append_include_directories("${BSP_INC_DIR}")
endif()
if(COMMAND collect_include_directories)
    collect_include_directories(BspUser)
endif()