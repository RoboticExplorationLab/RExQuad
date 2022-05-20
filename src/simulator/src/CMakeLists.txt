function (quad_node targetname)
  add_executable(${targetname}
    ${targetname}.cpp

    sim_utils.hpp
    sim_utils.cpp
  )
  target_link_libraries(${targetname}
    PRIVATE
    libzmq
    fmt::fmt
    libserialport
    quadutils
    quadcommon
    ArduinoEigen
  )
endfunction()

quad_node(measurement_node)
quad_node(mocap_relay)
# quad_node(fake_mocap)
quad_node(onboard_relay)

add_executable(control_node
  control_node.cpp
)
target_link_libraries(control_node
  PRIVATE
  libzmq
  fmt::fmt
  libserialport
  quadutils
  quadcommon
)

add_executable(server
  server.c
)
target_link_libraries(server
  PRIVATE
  libzmq
  fmt::fmt
)

add_executable(client
  client.c
)
target_link_libraries(client
  PRIVATE
  libzmq
  fmt::fmt
  libserialport
  quadutils
)