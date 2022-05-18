#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <fmt/core.h>
#include <fmt/chrono.h>
#include <fmt/ostream.h>
#include <libserialport.h>
#include <zmq.h>
#include <chrono>
#include <cmath>
#include <string>

#include "common/lqr_constants.hpp"
#include "common/messages.hpp"
#include "common/pose.hpp"
#include "common/utils.hpp"
#include "utils/serial.hpp"

using namespace rexquad;

int main() {

  // Open Serial port to transmitter
  std::string tx_name = "/dev/ttyACM0";
  int baudrate = 256000;
  struct sp_port* tx = rexquad::InitializeSerialPort(tx_name, baudrate);
  fmt::print("Connected to Transmitter\n");

  // Buffer for sending pose messages
  char buf_pose[sizeof(rexquad::PoseMsg)+1];
  int posemsg_len = sizeof(buf_pose);
  const int send_timeout_ms = 1000;  // ms
  rexquad::PoseMsg posemsg = {};
  posemsg.qw = 1.0;

  auto tstart = std::chrono::high_resolution_clock::now();
  fmt::print("Sending MOCAP data...\n");
  using SecondsDouble = std::chrono::duration<double>;
  while (1) {
    auto t_elapsed = std::chrono::high_resolution_clock::now() - tstart;
    SecondsDouble t = std::chrono::duration_cast<SecondsDouble>(t_elapsed);
    posemsg.z = std::sin(t.count() / 5);
    rexquad::PoseToBytes(buf_pose, posemsg);
    sp_blocking_write(tx, buf_pose, posemsg_len, send_timeout_ms);
    fmt::print("t = {}, z= {}\n", t, posemsg.z);
    usleep(1000*10);  // 100 Hz
  }

  return 0;
}