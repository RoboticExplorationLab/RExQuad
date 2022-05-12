cmake_minimum_required(VERSION 3.18.0)
list(APPEND CMAKE_MESSAGE_CONTEXT mocap)
project(MOCAP VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(CMakePrintHelpers)

# Add compile options
add_compile_options(-Wall -Wextra -pedantic -Werror -Wno-error=unknown-pragmas)
add_compile_options(-mavx2 -mfma)

#############################################
# Options
#############################################

# Handle default build type
set(MOCAP_DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified. Setting CMAKE_BUILD_TYPE to ${MOCAP_DEFAULT_BUILD_TYPE}")
  set(CMAKE_BUILD_TYPE ${ULQR_DEFAULT_BUILD_TYPE} CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

#############################################
# Dependencies
#############################################

# NatNet SDK
set(DEP_DIR "${PROJECT_SOURCE_DIR}/../../deps")
set(NATNET_DIR "${DEP_DIR}/NatNet_SDK")
cmake_print_variables(NATNET_DIR)

add_library(NatNetSDK SHARED IMPORTED)
set_target_properties(NatNetSDK PROPERTIES
  IMPORTED_LOCATION "${NATNET_DIR}/lib/libNatNet.so"
)
target_include_directories(NatNetSDK INTERFACE
  "${NATNET_DIR}/include"
)

# libserialport
find_package(PkgConfig)
pkg_check_modules(LIBSP libserialport)
add_library(libserialport INTERFACE)
target_link_libraries(libserialport
  INTERFACE
  ${LIBSP_LINK_LIBRARIES}
)
target_include_directories(libserialport
  INTERFACE
  ${LIBSP_INCLUDE_DIRS}
)

# fmtlib
include(FetchContent)
FetchContent_Declare(fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
  GIT_TAG d141cdbeb0fb422a3fb7173b285fd38e0d1772dc # version 8.0.1
)
FetchContent_MakeAvailable(fmt)

# GoogleTest
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()
include(GoogleTest)
include(CTest)

#############################################
# Compilation 
#############################################

# quadcore
add_subdirectory("../common" "common")
add_subdirectory("../utils" "utils")

add_library(mocap_lib
  Mocap.hpp
  Mocap.cpp

  callbacks.hpp
  callbacks.cpp

  utils.hpp
  utils.cpp
)
target_link_libraries(mocap_lib
  PUBLIC
  NatNetSDK
  fmt::fmt
  libserialport
  quadcommon
  quadutils
)
target_include_directories(mocap_lib
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}
)

add_executable(mocap
  mocap.cpp
)
target_link_libraries(mocap
  PRIVATE
  mocap_lib
)

add_executable(sample
  "${NATNET_DIR}/samples/SampleClient/SampleClient.cpp"
)
target_link_libraries(sample
  PRIVATE
  NatNetSDK
)

add_subdirectory(test)
