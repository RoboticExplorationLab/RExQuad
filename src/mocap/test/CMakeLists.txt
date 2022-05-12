function(mocaptest testname)
    add_executable(${testname}
        ${testname}.cpp
    )
    target_link_libraries(${testname}
        PRIVATE
        mocap_lib
        gtest_main
    )
endfunction()

mocaptest(send_to_feather)
mocaptest(lora_latency)
mocaptest(serial_latency)
mocaptest(serial_chain_latency)

mocaptest(serial_test)
gtest_discover_tests(serial_test) 

mocaptest(pose_test)
gtest_discover_tests(pose_test) 
