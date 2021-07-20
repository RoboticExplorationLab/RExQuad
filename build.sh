#!/bin/bash

# Build RExLabVicon package
if ! test -d "nodes/vicon_node/RExLabVicon/build"; then
    echo "RExLabVicon package has not been built!"    
fi

# Build proto buf messages for Arduino
SRC_DIR="nodes/vicon_node/RExLabVicon/proto"
INO_BUILD_DIR="Arduino/libraries/msgs"
MSG_NAME="vicon_msg.proto"

protoc -I=$SRC_DIR --nanopb_out=$INO_BUILD_DIR $MSG_NAME

SRC_DIR="proto"
INO_BUILD_DIR="Arduino/libraries/msgs"
MSG_NAME="imu_msg.proto"
protoc -I=$SRC_DIR --nanopb_out=$INO_BUILD_DIR $MSG_NAME

# Build proto buf messages for julia
JL_PLUGIN="$HOME/.julia/packages/ProtoBuf/TYEdo/plugin/protoc-gen-julia"
SRC_DIR="proto"
JL_BUILD_DIR="msgs"
MSG_NAME="imu_msg.proto"
protoc -I=. --plugin=$JL_PLUGIN --proto_path=$SRC_DIR --julia_out=$JL_BUILD_DIR $MSG_NAME