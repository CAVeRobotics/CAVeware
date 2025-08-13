cmake_minimum_required(VERSION 3.30)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

# Set board
if(NOT BOARD)
    set(BOARD "CAVeBoard")
endif()
message("Board: " ${BOARD})

# # Include toolchain file
# include("${CMAKE_SOURCE_DIR}/toolchain.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Project with support for ASM and C languages
project(CAVeware LANGUAGES C ASM)

# CAVeTalk library
include(${CMAKE_SOURCE_DIR}/CAVeTalk.cmake)

################################################################################
# Boards
################################################################################
add_subdirectory(${CMAKE_SOURCE_DIR}/boards)
get_target_property(BSP_USER_SRCS BspUser INTERFACE_SOURCES)
set_source_files_properties(
    ${BSP_USER_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)
get_target_property(CAVEBOT_USER_SRCS CavebotUser INTERFACE_SOURCES)
set_source_files_properties(
    ${CAVEBOT_USER_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

################################################################################
# BSP
################################################################################
add_subdirectory(${CMAKE_SOURCE_DIR}/bsp)
get_target_property(BSP_SRCS Bsp INTERFACE_SOURCES)
set_source_files_properties(
    ${BSP_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)
target_link_libraries(Bsp
    INTERFACE
        BspUser
)

################################################################################
# Sensors
################################################################################
add_subdirectory(${CMAKE_SOURCE_DIR}/sensors)
# TODO move to sensors CMakeLists.txt
get_target_property(SENSORS_SRCS Sensors INTERFACE_SOURCES)
set_source_files_properties(
    ${SENSORS_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

################################################################################
# Bots
################################################################################
add_subdirectory(${CMAKE_SOURCE_DIR}/bots)
# TODO move to bots CMakeLists.txt
get_target_property(BOTS_SRCS Bots INTERFACE_SOURCES)
set_source_files_properties(
    ${BOTS_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

################################################################################
# LSM6DSV16X driver
################################################################################
set(LSM6DSV16X_DIR ${CMAKE_SOURCE_DIR}/external/lsm6dsv16x-pid)
add_library(lsm6dsv16x INTERFACE)
target_sources(lsm6dsv16x
    INTERFACE
        ${LSM6DSV16X_DIR}/lsm6dsv16x_reg.c
)
target_include_directories(lsm6dsv16x
    INTERFACE
        ${LSM6DSV16X_DIR}
)

################################################################################
# Versioning
################################################################################
execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND git describe --tags --always
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(GIT_STATUS)
    set(GIT_DIRTY "Dirty")
else()
    set(GIT_DIRTY "Clean")
endif()

################################################################################
# CAVEMAN controller firmware
################################################################################
set(CAVEBOT_DIR ${CMAKE_SOURCE_DIR}/cavebot)
set(CAVEBOT_INC_DIR ${CAVEBOT_DIR}/inc)
set(CAVEBOT_SRC_DIR ${CAVEBOT_DIR}/src)

configure_file(
    ${CAVEBOT_INC_DIR}/cavebot_version.h.in
    ${CAVEBOT_INC_DIR}/cavebot_version.h
)

set(CAVEBOT_SRCS
    ${CAVEBOT_SRC_DIR}/cavebot.c
    ${CAVEBOT_SRC_DIR}/cavebot_cavetalk.c
    ${CAVEBOT_SRC_DIR}/cavebot_pid.c
)

set_source_files_properties(
    ${CAVEBOT_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

add_executable(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CAVEBOT_INC_DIR}
        ${ROVER_INC_DIR}
)

target_sources(${PROJECT_NAME}
    PRIVATE
        ${CAVEBOT_SRCS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Bsp
        Sensors
        Bots
        CavebotUser
        CAVeTalk-c
        lsm6dsv16x
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_SIZE} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf"
        COMMAND ${CMAKE_OBJCOPY} -O ihex "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex"
        COMMAND ${CMAKE_OBJCOPY} -O binary -S "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin"
)

################################################################################
# CI and tools
################################################################################
set(TOOLS_DIR ${CMAKE_SOURCE_DIR}/tools)
set(CPPCHECK_SOURCES ${BSP_SRCS} ${BSP_USER_SRCS} ${ROVER_SRCS} ${ROVER_CONFIG_SRCS} ${CAVEBOT_SRCS})
include(${TOOLS_DIR}/cppcheck/cppcheck.cmake)
get_target_property(BSP_USER_INC_DIRS BspUser INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(BSP_INC_DIRS Bsp INTERFACE_INCLUDE_DIRECTORIES)
set(UNCRUSTIFY_SOURCES ${CPPCHECK_SOURCES})
set(UNCRUSTIFY_INC_DIRS ${BSP_INC_DIRS} ${BSP_USER_INC_DIRS} ${ROVER_INC_DIR} ${ROVER_CONFIG_INC_DIR} ${CAVEBOT_INC_DIR})
include(${TOOLS_DIR}/uncrustify/uncrustify.cmake)