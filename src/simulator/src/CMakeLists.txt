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
)