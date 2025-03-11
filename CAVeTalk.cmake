set(CAVETALK_DIR ${CMAKE_SOURCE_DIR}/external/CAVeTalk)

################################################################################
# Common library
################################################################################
set(COMMON_DIR ${CAVETALK_DIR}/lib/common)
set(COMMON_INC_DIR ${COMMON_DIR}/inc)
set(COMMON_SRC_DIR ${COMMON_DIR}/src)
set(COMMON_SRCS ${COMMON_SRC_DIR}/cave_talk_link.c)
add_library(CAVeTalk-common INTERFACE)
target_sources(CAVeTalk-common
    INTERFACE
        ${COMMON_SRCS}
)
target_include_directories(CAVeTalk-common
    INTERFACE
        ${COMMON_INC_DIR}
)

################################################################################
# C messages
################################################################################
set(C_MESSAGES_OUT_DIR ${CAVETALK_DIR}/build/CAVeTalk-c_protos)
file(GLOB CAVE_TALK_C_MESSAGE_SRCS LIST_DIRECTORIES false CONFIGURE_DEPENDS
    "${C_MESSAGES_OUT_DIR}/*.pb.c"
)
add_library(CAVeTalk-c_messages INTERFACE)
target_sources(CAVeTalk-c_messages
    INTERFACE
        ${CAVE_TALK_C_MESSAGE_SRCS}
        ${C_MESSAGES_OUT_DIR}/pb_common.c
        ${C_MESSAGES_OUT_DIR}/pb_decode.c
        ${C_MESSAGES_OUT_DIR}/pb_encode.c
)
target_include_directories(CAVeTalk-c_messages
    INTERFACE
        ${C_MESSAGES_OUT_DIR}
)

################################################################################
# C library
################################################################################
set(C_DIR ${CAVETALK_DIR}/lib/c)
set(C_INC_DIR ${C_DIR}/inc)
set(C_SRC_DIR ${C_DIR}/src)
set(C_SRCS ${C_SRC_DIR}/cave_talk.c)
add_library(CAVeTalk-c INTERFACE)
target_sources(CAVeTalk-c
    INTERFACE
        ${C_SRCS}
)
target_include_directories(CAVeTalk-c
    INTERFACE
        ${C_INC_DIR}
)
target_link_libraries(CAVeTalk-c
    INTERFACE
        CAVeTalk-common
        CAVeTalk-c_messages
)