cmake_minimum_required(VERSION 3.30)
project(RGBW)

set(RGBW_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(RGBW_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${RGBW_INC_DIR}
)

target_sources(${PROJECT_NAME}
    PRIVATE
        ${RGBW_SRC_DIR}/rgbw.c
)

set_source_files_properties(
    ${RGBW_SRC_DIR}/rgbw.c
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Devices
)

if(COMMAND append_sources)
    append_sources("${RGBW_SRC_DIR}/rgbw.c")
endif()
if(COMMAND append_include_directories)
    append_include_directories("${RGBW_INC_DIR}")
endif()