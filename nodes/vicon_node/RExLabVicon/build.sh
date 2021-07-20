#!/bin/bash    
MSG_NAME="vicon_msg.proto"

protoc -I=. --proto_path=proto --cpp_out=msgs $MSG_NAME

mkdir -p build
cd build

rm CMakeCache.txt
cmake ..
make clean
make 
