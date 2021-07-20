cmake_minimum_required(VERSION 3.5)
project(vicon-datastream-sdk)

set (CMAKE_CXX_STANDARD 14)

find_package(Boost 1.5 COMPONENTS system thread timer REQUIRED)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
endif()

# Declare a cpp library
add_library(ViconDataStreamSDK_CPP SHARED
  # ViconCGStreamClient
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/CGStreamReaderWriter.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/ViconCGStreamBayer.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/ViconCGStreamClient.cpp
  # ViconCGStreamClientSDK
  Vicon/CrossMarket/DataStream/ViconCGStreamClientSDK/CGClient.cpp
  # ViconDataStreamSDK_CPP
  Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP/DataStreamClient.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP/DataStreamRetimingClient.cpp
  # ViconDataStreamSDKCore
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/AxisMapping.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/ClientUtils.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/CoreClient.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/RetimerUtils.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/RetimingClient.cpp
  # StreamCommon
  Vicon/CrossMarket/StreamCommon/CGStreamAsyncReaderWriter.cpp
)

target_include_directories(ViconDataStreamSDK_CPP PUBLIC
  Vicon/CrossMarket/DataStream
  Vicon/CrossMarket
)
target_link_libraries(ViconDataStreamSDK_CPP PUBLIC
  Boost::system
  Boost::thread
  Boost::timer
  pthread
)
# All users of this library will need at least C++14
target_compile_features(ViconDataStreamSDK_CPP PUBLIC cxx_std_14)

target_compile_options(ViconDataStreamSDK_CPP PRIVATE -Wno-unknown-pragmas -Wno-unused-function -Wno-unused-but-set-variable)

# Declare an executable

include_directories(
  Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP
)

add_executable(ViconDataStreamSDK_CPPTest
  Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPPTest/ViconDataStreamSDK_CPPTest.cpp
)

add_executable(ViconDataStreamSDK_CPPRetimerTest
  Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPPRetimerTest/ViconDataStreamSDK_CPPRetimerTest.cpp
  )

target_link_libraries(ViconDataStreamSDK_CPPTest
  ViconDataStreamSDK_CPP
)

target_link_libraries(ViconDataStreamSDK_CPPRetimerTest
  ViconDataStreamSDK_CPP
)
