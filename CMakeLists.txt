cmake_minimum_required(VERSION 3.0.2)
project(proto_mcu)

add_definitions(-std=c++17)
add_definitions(-O3)
add_definitions(-g)

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

message("-------------Protobuf_FOUND       : " ${Protobuf_FOUND})
message("-------------PROTOBUF_FOUND       : " ${PROTOBUF_FOUND})
message("-------------PROTOBUF_INCLUDE_DIRS: " ${PROTOBUF_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIRS})

add_executable(lsm6dsm_parse_log lsm6dsm_parse_log.cpp
                                 proto/fake_proto_serial.pb.cc)
target_link_libraries(lsm6dsm_parse_log ${PROTOBUF_LIBRARIES})

add_executable(mpu6050_parse_log mpu6050_parse_log.cpp
                                 proto/fake_proto_serial.pb.cc)
target_link_libraries(mpu6050_parse_log ${PROTOBUF_LIBRARIES})
