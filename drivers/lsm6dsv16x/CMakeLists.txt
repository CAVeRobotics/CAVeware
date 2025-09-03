cmake_minimum_required(VERSION 3.30)
project(Lsm6dsv16x)

set(LSM6DSV16X_EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/external/lsm6dsv16x-pid)
set(LSM6DSV16X_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(LSM6DSV16X_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${LSM6DSV16X_EXTERNAL_DIR}
        ${LSM6DSV16X_INC_DIR}
)

target_sources(${PROJECT_NAME}
    PRIVATE
        ${LSM6DSV16X_EXTERNAL_DIR}/lsm6dsv16x_reg.c
        ${LSM6DSV16X_SRC_DIR}/lsm6dsv16x.c
)

set_source_files_properties(
    ${LSM6DSV16X_SRC_DIR}/lsm6dsv16x.c
    PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Devices
)

if(COMMAND append_sources)
    append_sources("${LSM6DSV16X_SRC_DIR}/lsm6dsv16x.c")
endif()
if(COMMAND append_include_directories)
    append_include_directories("${LSM6DSV16X_INC_DIR}")
endif()