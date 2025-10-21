cmake_minimum_required(VERSION 3.30)
project(Bots)

set(BOTS_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(BOTS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

option(ROVER_4WS "Enable Rover 4WS bot configuration" OFF)
option(ROVER_4WD "Enable Rover 4WD bot configuration" OFF)

set(BOTS_SRCS "")

if (ROVER_4WS)
    message(STATUS "Adding support for Rover 4WS.")
    list(APPEND BOTS_SRCS ${BOTS_SRC_DIR}/rover_4ws.c)
endif()

if (ROVER_4WD)
    message(STATUS "Adding support for Rover 4WD.")
    list(APPEND BOTS_SRCS ${BOTS_SRC_DIR}/rover_4wd.c)
endif()

add_library(${PROJECT_NAME} INTERFACE)

target_compile_definitions(${PROJECT_NAME}
    INTERFACE
        $<$<BOOL:${ROVER_4WS}>:ROVER_4WS>
        $<$<BOOL:${ROVER_4WD}>:ROVER_4WD>
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${BOTS_INC_DIR}
)

target_sources(${PROJECT_NAME}
    INTERFACE
        ${BOTS_SRCS}
)

set_source_files_properties(
    ${BOTS_SRCS}
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

if(COMMAND collect_sources)
    collect_sources(${PROJECT_NAME})
endif()
if(COMMAND collect_include_directories)
    collect_include_directories(${PROJECT_NAME})
endif()