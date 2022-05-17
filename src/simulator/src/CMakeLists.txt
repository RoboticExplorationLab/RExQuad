add_executable(measurement_node
  measurement_node.cpp

  sim_utils.hpp
  sim_utils.cpp
)
target_link_libraries(measurement_node
  PRIVATE
  libzmq
  fmt::fmt
  libserialport
  quadutils
  quadcommon
  ArduinoEigen
)

add_executable(mocap_relay
  mocap_relay.cpp

  sim_utils.hpp
  sim_utils.cpp
)
target_link_libraries(mocap_relay
  PRIVATE
  libzmq
  fmt::fmt
  libserialport
  quadutils
  quadcommon
  ArduinoEigen
)

add_executable(fake_mocap
  fake_mocap.cpp
)
target_link_libraries(fake_mocap
  PRIVATE
  libzmq
  fmt::fmt
  libserialport
  quadutils
  quadcommon
  ArduinoEigen
)

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