#!/bin/bash

# Build RExLabVicon package
if ! test -d "nodes/vicon_relay/RExLabVicon/build"; then
    echo "RExLabVicon package has not been built!"
fi

NANOPB_PLUGIN="Arduino/libraries/nanopb/generator/protoc-gen-nanopb"
JL_PLUGIN="$HOME/.julia/packages/ProtoBuf/TYEdo/plugin/protoc-gen-julia"
JL_BUILD_DIR="msgs"
INO_BUILD_DIR="Arduino/libraries/msgs"

# VICON MESSAGE
SRC_DIR="nodes/vicon_relay/RExLabVicon/proto"
MSG_NAME="vicon_msg.proto"
protoc -I=. --plugin=$NANOPB_PLUGIN --proto_path=$SRC_DIR --nanopb_out=$INO_BUILD_DIR  $MSG_NAME
protoc -I=. --plugin=$JL_PLUGIN     --proto_path=$SRC_DIR --julia_out=$JL_BUILD_DIR    $MSG_NAME

# IMU MESSAGE
SRC_DIR1="proto"
SRC_DIR2="nodes/vicon_relay/RExLabVicon/proto"
MSG_NAME="imu_msg.proto"
protoc -I=. --plugin=$NANOPB_PLUGIN --proto_path=$SRC_DIR1 --proto_path=$SRC_DIR2 --nanopb_out=$INO_BUILD_DIR  $MSG_NAME
protoc -I=. --plugin=$JL_PLUGIN     --proto_path=$SRC_DIR1 --proto_path=$SRC_DIR2 --julia_out=$JL_BUILD_DIR    $MSG_NAME

# FILTERED STATE MESSAGE
SRC_DIR="proto"
MSG_NAME="filtered_state_msg.proto"
protoc -I=. --plugin=$NANOPB_PLUGIN --proto_path=$SRC_DIR --nanopb_out=$INO_BUILD_DIR  $MSG_NAME
protoc -I=. --plugin=$JL_PLUGIN     --proto_path=$SRC_DIR --julia_out=$JL_BUILD_DIR    $MSG_NAME

# QUAD INFO MESSAGE
SRC_DIR1="proto"
SRC_DIR2="nodes/vicon_relay/RExLabVicon/proto"
MSG_NAME="quad_info_msg.proto"
protoc -I=. --plugin=$NANOPB_PLUGIN --proto_path=$SRC_DIR1 --proto_path=$SRC_DIR2 --nanopb_out=$INO_BUILD_DIR  $MSG_NAME
protoc -I=. --plugin=$JL_PLUGIN     --proto_path=$SRC_DIR1 --proto_path=$SRC_DIR2 --julia_out=$JL_BUILD_DIR    $MSG_NAME

# # CHIYEN COMMUNICATION PROTO
# SRC_DIR="proto"
# MSG_NAME="message.proto"
# protoc -I=. --plugin=$JL_PLUGIN --julia_out=$JL_BUILD_DIR  $SRC_DIR/$MSG_NAME
