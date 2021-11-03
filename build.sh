#!/bin/bash
JL_PLUGIN="$HOME/.julia/packages/ProtoBuf/TYEdo/plugin/protoc-gen-julia"
JL_BUILD_DIR="msgs"

# QUAD INFO MESSAGE
SRC_DIR="proto"
MSG_NAME="motors_msg.proto"
# protoc -I=. --plugin=$NANOPB_PLUGIN --proto_path=$SRC_DIR --nanopb_out=$INO_BUILD_DIR  $MSG_NAME
protoc -I=. --plugin=$JL_PLUGIN     --proto_path=$SRC_DIR --julia_out=$JL_BUILD_DIR    $MSG_NAME

# FILTERED STATE MESSAGE
SRC_DIR="proto"
MSG_NAME="filtered_state_msg.proto"
protoc -I=. --plugin=$JL_PLUGIN     --proto_path=$SRC_DIR --julia_out=$JL_BUILD_DIR    $MSG_NAME

# QUAD INFO MESSAGE
SRC_DIR="proto"
MSG_NAME="quad_info_msg.proto"
protoc -I=. --plugin=$JL_PLUGIN     --proto_path=$SRC_DIR --julia_out=$JL_BUILD_DIR    $MSG_NAME

# QUAD INFO MESSAGE
SRC_DIR="proto"
MSG_NAME="ground_info_msg.proto"
protoc -I=. --plugin=$JL_PLUGIN     --proto_path=$SRC_DIR --julia_out=$JL_BUILD_DIR    $MSG_NAME