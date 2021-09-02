cmake_minimum_required(VERSION 2.8.12)
project(simple_vicon)
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/vicon-datastream-sdk/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()
add_subdirectory(extern/vicon-datastream-sdk)
add_library(vicon_driver
  src/ViconDriver.cpp
)
target_include_directories(vicon_driver PUBLIC include)
target_link_libraries(vicon_driver PUBLIC
  ViconDataStreamSDK_CPP
)
target_compile_features(vicon_driver PRIVATE cxx_std_17)
add_executable(vicon_test src/vicon_test.cpp)
target_link_libraries(vicon_test PUBLIC
  vicon_driver
)
install(TARGETS vicon_driver DESTINATION lib)
