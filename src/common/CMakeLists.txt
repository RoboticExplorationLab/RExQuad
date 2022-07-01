add_compile_options(-Wno-error=missing-field-initializers)
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

    workspace.h
    workspace.c

    problem_data.h
    problem_data.c

    osqpsolver.hpp
    osqpsolver.cpp

    problem.hpp
    problem.cpp

    mpc_types.hpp

    riccati.hpp
    riccati.cpp

    rotations.h
    rotations.c

    delayed_mekf.h
    delayed_mekf.c

    linear_algebra.h
    linear_algebra.c
)
target_link_libraries(quadcommon
    PUBLIC
    ArduinoEigen
    empc
    slap
)
target_include_directories(quadcommon
    INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/.."
)

add_subdirectory(test)