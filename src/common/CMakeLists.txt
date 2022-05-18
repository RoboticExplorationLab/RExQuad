add_library(quadcommon
    pose.hpp
    pose.cpp

    messages.hpp
    messages.cpp

    utils.hpp
    utils.cpp 

    quaternion.hpp
    quaternion.cpp

    estimator.hpp
    estimator.cpp

    control.hpp
    control.cpp

    constants.hpp
)
target_link_libraries(quadcommon
    PUBLIC
    ArduinoEigen
)
target_include_directories(quadcommon
    INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/.."
)

add_subdirectory(test)