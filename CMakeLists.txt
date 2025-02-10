cmake_minimum_required(VERSION 3.30)
project(CavemanController LANGUAGES ASM C)

################################################################################
# STM32 settings
################################################################################
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}ranlib)

set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g3")
endif()
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -g0")
endif()

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/stm32f411retx_flash.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")

add_compile_definitions(
	USE_HAL_DRIVER 
	STM32F411xE
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Debug>:BSP_LOGGER>
    $<$<CONFIG:Debug>:BSP_LOGGER_USER_CUSTOM_LOGGER>
)

################################################################################
# STM32 HAL library
################################################################################
# TODO just GLOB for HAL and Core sources
# TODO update for STM32F407xx
set(DRIVERS_DIR ${CMAKE_SOURCE_DIR}/Drivers)
set(CMSIS_DIR ${DRIVERS_DIR}/CMSIS)
set(CMSIS_INC_DIR ${CMSIS_DIR}/Include)
set(HAL_DRIVER_DIR ${DRIVERS_DIR}/STM32F4xx_HAL_Driver)
set(HAL_DRIVER_INC_DIR ${HAL_DRIVER_DIR}/Inc)
set(HAL_DRIVER_SRC_DIR ${HAL_DRIVER_DIR}/Src)
set(CORE_DIR ${CMAKE_SOURCE_DIR}/Core)
set(CORE_INC_DIR ${CORE_DIR}/Inc)
set(CORE_SRC_DIR ${CORE_DIR}/Src)

set(HAL_SRCS
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_tim.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_tim_ex.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_uart.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_rcc.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_rcc_ex.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_flash.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_flash_ex.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_flash_ramfunc.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_gpio.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_dma_ex.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_dma.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_pwr.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_pwr_ex.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_cortex.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal.c
    ${HAL_DRIVER_SRC_DIR}/stm32f4xx_hal_exti.c
)

set(HAL_INC_DIRS
    ${CMSIS_INC_DIR}
    ${CMSIS_DIR}/Device/ST/STM32F4xx/Include
    ${HAL_DRIVER_INC_DIR}
    ${HAL_DRIVER_INC_DIR}/Legacy
    ${CORE_INC_DIR}
)

add_library(Hal ${HAL_SRCS})
target_include_directories(Hal PUBLIC ${HAL_INC_DIRS})

################################################################################
# BSP library
################################################################################
set(BSP_DIR ${CMAKE_SOURCE_DIR}/bsp)
set(BSP_INC_DIR ${BSP_DIR}/inc)
set(BSP_SRC_DIR ${BSP_DIR}/src)

set(CORE_SRCS
    ${CORE_SRC_DIR}/main.c
    ${CORE_SRC_DIR}/gpio.c
    ${CORE_SRC_DIR}/tim.c
    ${CORE_SRC_DIR}/usart.c
    ${CORE_SRC_DIR}/stm32f4xx_it.c
    ${CORE_SRC_DIR}/stm32f4xx_hal_msp.c
    ${CORE_SRC_DIR}/system_stm32f4xx.c
    ${CORE_SRC_DIR}/sysmem.c
    ${CORE_SRC_DIR}/syscalls.c
    ${CMAKE_SOURCE_DIR}/startup_stm32f411xe.s
)

set(BSP_SRCS
    ${BSP_SRC_DIR}/bsp.c
    ${BSP_SRC_DIR}/bsp_logger.c
    ${BSP_SRC_DIR}/bsp_logger_user.c
    ${BSP_SRC_DIR}/bsp_pwm.c
    ${BSP_SRC_DIR}/bsp_pwm_user.c
    ${BSP_SRC_DIR}/bsp_servo.c
    ${BSP_SRC_DIR}/bsp_uart.c
    ${BSP_SRC_DIR}/bsp_uart_user.c
)

set(BSP_INC_DIRS
    ${HAL_INC_DIRS}
    ${CORE_INC_DIR}
    ${BSP_INC_DIR}
)

# TODO compile BSP as a separate library
# add_library(Bsp ${BSP_SRCS})
# target_include_directories(Bsp PUBLIC ${BSP_INC_DIRS})
# target_link_libraries(Bsp PRIVATE Hal)
# target_compile_options(Bsp PRIVATE "-Werror")

################################################################################
# CAVEMAN controller firmware
################################################################################
set(CAVEMAN_CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/caveman_controller)
set(CAVEMAN_CONTROLLER_INC_DIR ${CAVEMAN_CONTROLLER_DIR}/inc)
set(CAVEMAN_CONTROLLER_SRC_DIR ${CAVEMAN_CONTROLLER_DIR}/src)

set(CAVEMAN_CONTROLLER_SRCS
    ${CAVEMAN_CONTROLLER_SRC_DIR}/caveman_controller.c
)

add_executable(${PROJECT_NAME} ${CAVEMAN_CONTROLLER_SRCS} ${CORE_SRCS} ${BSP_SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE ${CAVEMAN_CONTROLLER_INC_DIR} ${BSP_INC_DIRS})
# target_link_libraries(${PROJECT_NAME} PRIVATE Bsp)
target_link_libraries(${PROJECT_NAME} PRIVATE Hal)
target_compile_options(${PROJECT_NAME} PRIVATE "-Werror")
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_SIZE} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf"
        COMMAND ${CMAKE_OBJCOPY} -O ihex "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex"
        COMMAND ${CMAKE_OBJCOPY} -O binary -S "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin"
)

################################################################################
# CI and tools
################################################################################
set(TOOLS_DIR ${CMAKE_SOURCE_DIR}/tools)
include(${TOOLS_DIR}/cppcheck/cppcheck.cmake)
include(${TOOLS_DIR}/uncrustify/uncrustify.cmake)