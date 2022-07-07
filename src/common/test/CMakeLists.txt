function (common_test testname)
  add_executable(${testname}_test
    ${testname}_test.cpp

    filter_data.h
    filter_data.c
  )
  target_link_libraries(${testname}_test
    PRIVATE
    quadcommon
    gtest_main
    fmt::fmt
  )
endfunction()

common_test(control)
common_test(osqpsolver)
common_test(problem)
common_test(riccati)
common_test(delayed_mekf)
common_test(vector_stack)