add_library(quadutils
    serial.hpp
    serial.cpp
)
target_link_libraries(quadutils
    PUBLIC
    libserialport

    PRIVATE
    fmt::fmt
)
target_include_directories(quadutils
    INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/.."
)