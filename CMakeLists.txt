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

# Collection functions
include(${CMAKE_SOURCE_DIR}/collect.cmake)

# CAVeTalk library
include(${CMAKE_SOURCE_DIR}/CAVeTalk.cmake)

################################################################################
# Subdirectories
################################################################################
add_subdirectory(${CMAKE_SOURCE_DIR}/boards)
add_subdirectory(${CMAKE_SOURCE_DIR}/devices)
add_subdirectory(${CMAKE_SOURCE_DIR}/drivers)
add_subdirectory(${CMAKE_SOURCE_DIR}/bots)

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
    ${CMAKE_SOURCE_DIR}/boards/${BOARD}/cavebot_user/src/cavebot_user.c
)

set_source_files_properties(
    ${CAVEBOT_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

get_target_property(BOTS_SRCS Bots INTERFACE_SOURCES)
if(BOTS_SRCS)
    set_source_files_properties(
        ${BOTS_SRCS}
        PROPERTIES
        COMPILE_OPTIONS "-Werror"
    )
endif()

add_executable(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CAVEBOT_INC_DIR}
        ${CMAKE_SOURCE_DIR}/boards/${BOARD}/cavebot_user/inc
)

target_sources(${PROJECT_NAME}
    PRIVATE
        ${CAVEBOT_SRCS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Bsp
        Drivers
        Bots
        CAVeTalk-c
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_SIZE} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf"
        COMMAND ${CMAKE_OBJCOPY} -O ihex "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex"
        COMMAND ${CMAKE_OBJCOPY} -O binary -S "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin"
)

if(COMMAND collect_sources)
    collect_sources(${PROJECT_NAME})
endif()
if(COMMAND collect_include_directories)
    collect_include_directories(${PROJECT_NAME})
endif()

################################################################################
# CI and tools
################################################################################
set(TOOLS_DIR ${CMAKE_SOURCE_DIR}/tools)
get_property(CPPCHECK_SOURCES GLOBAL PROPERTY SOURCE_COLLECTION)
get_property(UNCRUSTIFY_INC_DIRS GLOBAL PROPERTY INCLUDE_DIRECTORIES_COLLECTION)
if (CPPCHECK_SOURCES)
    include(${TOOLS_DIR}/cppcheck/cppcheck.cmake)
endif()
if (CPPCHECK_SOURCES AND UNCRUSTIFY_INC_DIRS)
    set(UNCRUSTIFY_SOURCES ${CPPCHECK_SOURCES})
    include(${TOOLS_DIR}/uncrustify/uncrustify.cmake)
endif()