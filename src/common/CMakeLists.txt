add_library(quadcommon
    pose.hpp
    pose.cpp

    utils.hpp
    utils.cpp
)
target_include_directories(quadcommon
    INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/.."
)