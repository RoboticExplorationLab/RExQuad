function (common_test testname)
  add_executable(${testname}_test
    ${testname}_test.cpp
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