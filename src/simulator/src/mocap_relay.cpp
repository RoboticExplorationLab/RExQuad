#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <libserialport.h>
#include <zmq.h>
#include <cstdlib>
#include <string>

#include "common/lqr_constants.hpp"
#include "common/messages.hpp"
#include "common/pose.hpp"
#include "common/utils.hpp"
#include "utils/serial.hpp"
#include "sim_utils.hpp"

using namespace rexquad;

int main(int argc, char** argv) {
  std::string subport = "5556";
  if (argc > 1) {
    subport = argv[1];
  }

  // Open Serial port to transmitter
  std::string tx_name = "/dev/ttyACM0";
  int baudrate = 57600;
  struct sp_port* tx = rexquad::InitializeSerialPort(tx_name, baudrate);
  fmt::print("Connected to Transmitter\n");

  // Set up ZMQ Subscriber
  void* context = zmq_ctx_new();
  void* sub = zmq_socket(context, ZMQ_SUB);
  int rc;

  int conflate = 1;
  rc = zmq_setsockopt(sub, ZMQ_CONFLATE, &conflate, sizeof(conflate));
  if (rc != 0) {
    printf("Failed to set conflate.\n");
    return 1;
  }
  std::string tcpaddress_sub = "tcp://127.0.0.1:" + subport;
  rc = zmq_connect(sub, tcpaddress_sub.c_str());
  if (rc != 0) {
    printf("Failed to connect subscriber.\n");
    return 1;
  } else {
    fmt::print("Connected subscriber at {}\n", tcpaddress_sub);
  }
  rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
  if (rc != 0) {
    printf("Failed to set subscriber.\n");
    return 1;
  }

  // Receiving MeasurementMsg over ZMQ
  constexpr int len_zmqrecv = sizeof(MeasurementMsg) + 1;
  uint8_t buf_zmqrecv[len_zmqrecv];
  MeasurementMsg measmsg = {};

  // Sending PoseMsg over Serial
  char buf_pose[sizeof(rexquad::PoseMsg)+1];
  int posemsg_len = sizeof(buf_pose);
  const int send_timeout_ms = 100;  // ms
  rexquad::PoseMsg posemsg = {};


  while (1) {
    // Receive MeasurementMsg over ZMQ
    int zmq_bytes_received = zmq_recv(sub, buf_zmqrecv, len_zmqrecv, 0);
    fmt::print("\nReceived {} bytes over ZMQ\n", zmq_bytes_received);
    bool good_conversion = MeasurementMsgFromBytes(measmsg, buf_zmqrecv);
    fmt::print("  Successful conversion to MeasurementMsg: {}\n", good_conversion);
    fmt::print("  Ang Velocity = [{:.3f}, {:.3f}, {:.3f}]\n", measmsg.wx, measmsg.wy, measmsg.wz);

    // Extract Pose information
    posemsg.x = measmsg.x;
    posemsg.y = measmsg.y;
    posemsg.z = measmsg.z;
    posemsg.qw = measmsg.qw;
    posemsg.qx = measmsg.qx;
    posemsg.qy = measmsg.qy;
    posemsg.qz = measmsg.qz;
    rexquad::PoseToBytes(buf_pose, posemsg);

    // Send pose to transmitter 
    sp_blocking_write(tx, buf_pose, posemsg_len, send_timeout_ms);
    fmt::print("Message sent:\n");
    print_msg(measmsg);
  }

  return EXIT_SUCCESS;
}