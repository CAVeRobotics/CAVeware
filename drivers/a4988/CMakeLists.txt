cmake_minimum_required(VERSION 3.30)
project(A4988)

set(A4988_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(A4988_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${A4988_INC_DIR}
)

target_sources(${PROJECT_NAME}
    PRIVATE
        ${A4988_SRC_DIR}/a4988.c
)

set_source_files_properties(
    ${A4988_SRC_DIR}/a4988.c
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Devices
)

if(COMMAND append_sources)
    append_sources("${A4988_SRC_DIR}/a4988.c")
endif()
if(COMMAND append_include_directories)
    append_include_directories("${A4988_INC_DIR}")
endif()