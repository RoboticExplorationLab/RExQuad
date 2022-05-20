#include "Mocap.hpp"

#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetTypes.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cmath>
#include <string>
#include <vector>

#include "callbacks.hpp"
#include "fmt/core.h"
#include "libserialport.h"

int main(int argc, char* argv[]) {
  std::string port;
  if (argc > 1) {
    port = argv[1];
  } else {
    port = "/dev/ttyACM0";
  }

  fmt::print("Connecting to Feather Transmitter at port {}\n", port);
  int baudrate = 256000;
  rexquad::SerialCallback radio(port.c_str(), baudrate);
  radio.Open();

  using SecondsDouble = std::chrono::duration<double>;
  sRigidBodyData rbdata;
  rbdata.ID = 0;
  rbdata.x = 0.0f;
  rbdata.y = 0.0f;
  rbdata.z = 0.0f;
  rbdata.qx = 0.0f;
  rbdata.qy = 0.0f;
  rbdata.qz = 0.0f;
  rbdata.qw = 1.0f;
  rbdata.MeanError = 0.0f;
  rbdata.params = 0;

  auto tstart = std::chrono::high_resolution_clock::now();
  fmt::print("Sending MOCAP data...\n");
  while (1) {
    auto t_elapsed = std::chrono::high_resolution_clock::now() - tstart;
    SecondsDouble t = std::chrono::duration_cast<SecondsDouble>(t_elapsed);
    rbdata.z = std::sin(t.count() / 5);
    radio(rbdata);
    usleep(1000*10);
  }

  return 0;
}
