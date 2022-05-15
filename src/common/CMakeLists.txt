add_library(quadcommon
    pose.hpp
    pose.cpp

    messages.hpp
    messages.cpp

    utils.hpp
    utils.cpp
)
target_include_directories(quadcommon
    INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/.."
)