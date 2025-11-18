cmake_minimum_required(VERSION 3.30)
project(Bsp)

################################################################################
# BSP library
################################################################################
set(BSP_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(BSP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(BSP_SRCS
    ${BSP_SRC_DIR}/bsp.c
    ${BSP_SRC_DIR}/bsp_logger.c
    ${BSP_SRC_DIR}/bsp_tick.c
)

option(BSP_ADC "Enable ADC" OFF)
option(BSP_ENCODER "Enable encoders" OFF)
option(BSP_GPIO "Enable GPIO" OFF)
option(BSP_MOTOR "Enable motors" OFF)
option(BSP_PWM "Enable PWM" OFF)
option(BSP_SERVO "Enable servos" OFF)
option(BSP_SPI "Enable SPI" OFF)
option(BSP_TIMER "Enable timers" OFF)
option(BSP_UART "Enable UART" OFF)

if(BSP_ADC)
    message(STATUS "BSP ADC enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_adc.c)
endif()
if(BSP_ENCODER)
    message(STATUS "BSP Encoders enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_encoder.c)
endif()
if(BSP_GPIO)
    message(STATUS "BSP GPIO enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_gpio.c)
endif()
if(BSP_MOTOR)
    message(STATUS "BSP Motors enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_motor.c)
endif()
if(BSP_PWM)
    message(STATUS "BSP PWM enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_pwm.c)
endif()
if(BSP_SPI)
    message(STATUS "BSP SPI enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_spi.c)
endif()
if(BSP_SERVO)
    message(STATUS "BSP Servos enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_servo.c)
endif()
if(BSP_TIMER)
    message(STATUS "BSP Timers enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_timer.c)
endif()
if(BSP_UART)
    message(STATUS "BSP UART enabled.")
    list(APPEND BSP_SRCS ${BSP_SRC_DIR}/bsp_uart.c)
endif()

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

add_library(${PROJECT_NAME} OBJECT)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>
        $<TARGET_PROPERTY:BspUser,INTERFACE_COMPILE_DEFINITIONS>
        $<$<BOOL:${BSP_ADC}>:BSP_ADC>
        $<$<BOOL:${BSP_GPIO}>:BSP_GPIO>
        $<$<BOOL:${BSP_SPI}>:BSP_SPI>
        $<$<BOOL:${BSP_UART}>:BSP_UART>
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