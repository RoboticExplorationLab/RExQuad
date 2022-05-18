#include <assert.h>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <libserialport.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <zmq.h>

#include <cstdlib>
#include <string>

#include "common/lqr_constants.hpp"
#include "common/messages.hpp"
#include "common/pose.hpp"
#include "common/utils.hpp"
#include "sim_utils.hpp"
#include "utils/serial.hpp"

using namespace rexquad;

// Constants
using Pose = rexquad::PoseMsg;
using StateControl = rexquad::StateControlMsg;
constexpr int kMaxBufferSize = 200;
constexpr int kStateControlSize = sizeof(StateControl) + 1;

int main(int argc, char** argv) {
  std::string pubport = "5557";
  if (argc > 1) {
    pubport = argv[1];
  }
  bool verbose = false;
  if (argc > 2) {
    std::string flag = std::string(argv[2]);
    verbose = (flag == "-v") || (flag == "--verbose");
  }
  fmt::print("Verbose output? {}\n", verbose);

  // Open Serial port to onboard Feather
  std::string rx_name = "/dev/ttyACM1";
  int baudrate = 256000;
  struct sp_port* onboard = rexquad::InitializeSerialPort(rx_name, baudrate);
  fmt::print("Connected to Onboard Feather\n");

  // Set up ZMQ publisher
  void* context = zmq_ctx_new();
  void* pub = zmq_socket(context, ZMQ_PUB);
  std::string tcpaddress_pub = "tcp://127.0.0.1:" + pubport;
  int rc = zmq_connect(pub, tcpaddress_pub.c_str());
  if (rc != 0) {
    printf("Failed to connect publisher.\n");
    return 1;
  } else {
    fmt::print("Connected publisher at {}\n", tcpaddress_pub);
  }

  // Receiving StateControl over Serial
  uint8_t buf_serialrecv[kMaxBufferSize];
  const int recv_timeout_ms = 1000;
  StateControlMsg statecontrol_onboard = {};

  // Sending StateControl over ZMQ
  constexpr int len_zmqsend = kStateControlSize;
  uint8_t buf_zmqsend[len_zmqsend];

  printf("Waiting for messages from onboard computer...\n");
  while (1) {
    // Receive the state and control from onboard computer
    int bytes_received =
        sp_blocking_read(onboard, buf_serialrecv, kStateControlSize, recv_timeout_ms);
    bool received_serial_packet = bytes_received >= kStateControlSize;
    if (received_serial_packet) {

      // Broadcast state and control over ZMQ
      memcpy(buf_zmqsend, buf_serialrecv, kStateControlSize);
      zmq_send(pub, buf_zmqsend, len_zmqsend, 0);

      // Print summary
      if (verbose) {
        fmt::print("Receieved {} / {} bytes:\n", (int)bytes_received, kStateControlSize);
        rexquad::StateControlMsgFromBytes(statecontrol_onboard, buf_serialrecv);
        print_msg(statecontrol_onboard);
        fmt::print("  ZMQ Message Sent\n");
      }
    }
  }

  zmq_close(pub);
  zmq_ctx_destroy(context);
}