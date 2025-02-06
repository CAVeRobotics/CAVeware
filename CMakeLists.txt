cmake_minimum_required(VERSION 3.30)
# project(CavemanController LANGUAGES ASM C)
project(CavemanController LANGUAGES C)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
# set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
find_program(ARM_NONE_EABI_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(ARM_NONE_EABI_SIZE arm-none-eabi-size REQUIRED)

# TODO generate .d and .lst for debugging?

# TODO update for STM32F407xx (see new generated Makefile)
set(MCU_FLAGS "-mcpu=cortex-m4")
set(FPU_FLAGS "-mfpu=fpv4-sp-d16")
set(FLOAT_ABI_FLAGS "-mfloat-abi=hard")
set(ARM_FLAGS
    # "-fno-strict-aliasing"
    "-fdata-sections"
    "-ffunction-sections"
    "-fstack-usage"
    "${MCU_FLAGS}"
    "-mthumb"
    "${FPU_FLAGS}"
    "${FLOAT_ABI_FLAGS}"
)
set(ARM_ASM_FLAGS
    "-x"
    "assembler-with-cpp"
)
set(ARM_LD_FLAGS
    "-specs=nano.specs"
    "-specs=nosys.specs"
    "-lc"
    "-lm"
    "LINKER:--gc-sections"
)
set(LDSCRIPT "${CMAKE_SOURCE_DIR}/STM32F411RETx_FLASH.ld")
set(SRC_COMPILE_OPTIONS
    "-Wextra"
    "-Werror"
    "-pedantic"
)

add_compile_options(
    "$<$<COMPILE_LANGUAGE:ASM>:${ARM_ASM_FLAGS}>"
    ${ARM_FLAGS}
    "-Wall"
    "-fdiagnostics-color=always"
  )
add_link_options(
    ${ARM_FLAGS}
    "-T${LDSCRIPT}"
    ${ARM_LD_FLAGS}
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# TODO may need to update debugging info format
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Building in Debug mode")
    add_compile_options(
        # "-O0"
        # "-g3"
        "-Og"
        "-g"
        "-gdwarf-2"
    )
else()
    message("Building in Release mode")
    add_compile_options("-Os") # optimize Release builds for size for now
endif()

# TODO may want to enable logging at a less verbose level in Release builds
add_compile_definitions(
    $<$<CONFIG:Debug>:LOGGER> # Enable logging in debug mode
    USE_HAL_DRIVER
    STM32F411xE
)

# TODO just GLOB for HAL and Core sources
# TODO update for STM32F407xx
set(DRIVERS_DIR ${CMAKE_SOURCE_DIR}/Drivers)
set(CMSIS_DIR ${DRIVERS_DIR}/CMSIS)
set(CMSIS_INC_DIR ${CMSIS_DIR}/Include)
set(HAL_DRIVER_DIR ${DRIVERS_DIR}/STM32F4xx_HAL_Driver)
set(HAL_DRIVER_INC_DIR ${HAL_DRIVER_DIR}/Inc)
set(HAL_DRIVER_SRC_DIR ${HAL_DRIVER_DIR}/Src)
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

# TODO move main to caveman_controller
# TODO update for STM32F407xx
set(CORE_DIR ${CMAKE_SOURCE_DIR}/Core)
set(CORE_INC_DIR ${CORE_DIR}/Inc)
set(CORE_SRC_DIR ${CORE_DIR}/Src)
set(CORE_SRCS
    # ${CORE_SRC_DIR}/main.c
    ${CORE_SRC_DIR}/stm32f4xx_it.c
    ${CORE_SRC_DIR}/stm32f4xx_hal_msp.c
    ${CORE_SRC_DIR}/system_stm32f4xx.c
    ${CORE_SRC_DIR}/gpio.c
    ${CORE_SRC_DIR}/tim.c
    ${CORE_SRC_DIR}/usart.c
)

set(BSP_DIR ${CMAKE_SOURCE_DIR}/bsp)
set(BSP_INC_DIR ${BSP_DIR}/inc)
set(BSP_SRC_DIR ${BSP_DIR}/src)
set(BSP_SRCS
    ${BSP_SRC_DIR}/bsp_pwm.c
    ${BSP_SRC_DIR}/bsp_pwm_user.c
    ${BSP_SRC_DIR}/bsp_servo.c
)

set(CAVEMAN_CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/caveman_controller)
set(CAVEMAN_CONTROLLER_INC_DIR ${CAVEMAN_CONTROLLER_DIR}/inc)
set(CAVEMAN_CONTROLLER_SRC_DIR ${CAVEMAN_CONTROLLER_DIR}/src)
set(CAVEMAN_CONTROLLER_SRCS
    ${CAVEMAN_CONTROLLER_SRC_DIR}/caveman_controller.c
)

# TODO update for STM32F407xx
set(ASM_SRCS
    ${CMAKE_SOURCE_DIR}/startup_stm32f411xe.s
)

set(INC_DIRS
    ${CMSIS_INC_DIR}
    ${CMSIS_DIR}/Device/ST/STM32F4xx/Include
    ${HAL_DRIVER_INC_DIR}
    ${HAL_DRIVER_INC_DIR}/Legacy
    ${CORE_INC_DIR}
    ${BSP_INC_DIR}
    ${CAVEMAN_CONTROLLER_INC_DIR}
)

################################################################################
# STM32 generated code library
################################################################################
add_library(HalCore ${HAL_SRCS} ${CORE_SRCS})
target_include_directories(HalCore PRIVATE ${INC_DIRS})

################################################################################
# BSP
################################################################################
add_library(Bsp ${BSP_SRCS})
target_include_directories(Bsp PRIVATE ${INC_DIRS})
target_link_libraries(Bsp PRIVATE HalCore)
target_compile_options(Bsp PRIVATE ${SRC_COMPILE_OPTIONS})

################################################################################
# Caveman
################################################################################
set_property(SOURCE ${ASM_SRCS} PROPERTY LANGUAGE C) # Some trickery to get CMake to deal with our assembler code?
add_executable(caveman ${CORE_SRC_DIR}/main.c ${CAVEMAN_CONTROLLER_SRCS} ${ASM_SRCS})
target_include_directories(caveman PRIVATE ${INC_DIRS})
target_link_libraries(caveman PRIVATE Bsp)
target_compile_options(caveman PRIVATE ${SRC_COMPILE_OPTIONS})
set_target_properties(caveman PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET caveman POST_BUILD
        COMMAND ${ARM_NONE_EABI_SIZE} "${CMAKE_CURRENT_BINARY_DIR}/caveman.elf"
        COMMAND ${ARM_NONE_EABI_OBJCOPY} -O ihex "${CMAKE_CURRENT_BINARY_DIR}/caveman.elf" "${CMAKE_CURRENT_BINARY_DIR}/caveman.hex"
        COMMAND ${ARM_NONE_EABI_OBJCOPY} -O binary "${CMAKE_CURRENT_BINARY_DIR}/caveman.elf" "${CMAKE_CURRENT_BINARY_DIR}/caveman.bin"
)