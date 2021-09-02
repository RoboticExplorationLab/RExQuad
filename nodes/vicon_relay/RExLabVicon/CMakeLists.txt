CMAKE_MINIMUM_REQUIRED (VERSION 2.8.12)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
PROJECT(vicon)

set(CMAKE_CXX_STANDARD 11)

find_package(cppzmq REQUIRED)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

add_subdirectory(extern/simple_vicon)

set(PROTO_MESSAGE_DIR msgs)
set(HEADERS ${PROTO_MESSAGE_DIR}/vicon_msg.pb.h ${PROTO_MESSAGE_DIR}/vicon_msg.pb.cc)

add_library(vicon_driver_zmq src/ViconDriverZMQ.cpp include/ViconDriverZMQ.h ${HEADERS})
target_include_directories(vicon_driver_zmq PUBLIC include ${PROTO_MESSAGE_DIR})
target_link_libraries(vicon_driver_zmq PUBLIC vicon_driver)

add_executable(vicon_pub src/server.cpp ${HEADERS})
target_include_directories(vicon_pub PUBLIC ${PROTOBUF_INCLUDE_DIRS} ${PROTO_MESSAGE_DIR})
target_link_libraries(vicon_pub ${PROTOBUF_LIBRARIES} cppzmq vicon_driver_zmq)
