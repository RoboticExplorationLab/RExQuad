#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <queue>
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
  size_t delay = 0;
  if (argc > 1) {
    delay = atoi(argv[1]);
  }
  if (argc > 2) {
    subport = argv[2];
  }
  bool verbose = false;
  if (argc > 3) {
    std::string flag = std::string(argv[3]);
    verbose = (flag == "-v") || (flag == "--verbose");
  }
  fmt::print("Using delay of {}\n", delay);
  fmt::print("Verbose output? {}\n", verbose);

  // Open Serial port to transmitter
  std::string tx_name = "/dev/ttyACM0";
  int baudrate = 256000;
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
  
  // Queue for simulating a delay on the MOCAP
  std::queue<PoseMsg> mocap_queue;

  // Receiving MeasurementMsg over ZMQ
  constexpr int len_zmqrecv = sizeof(MeasurementMsg) + 1;
  uint8_t buf_zmqrecv[len_zmqrecv];
  MeasurementMsg measmsg = {};

  // Sending PoseMsg over Serial
  char buf_pose[sizeof(rexquad::PoseMsg)+1];
  int posemsg_len = sizeof(buf_pose);
  const int send_timeout_ms = 100;  // ms
  rexquad::PoseMsg posemsg = {};
  rexquad::PoseMsg pose_delayed = {};

  while (1) {
    // Receive MeasurementMsg over ZMQ
    int zmq_bytes_received = zmq_recv(sub, buf_zmqrecv, len_zmqrecv, 0);
    bool good_conversion = MeasurementMsgFromBytes(measmsg, buf_zmqrecv);

    // Extract Pose information
    posemsg.x = measmsg.x;
    posemsg.y = measmsg.y;
    posemsg.z = measmsg.z;
    posemsg.qw = measmsg.qw;
    posemsg.qx = measmsg.qx;
    posemsg.qy = measmsg.qy;
    posemsg.qz = measmsg.qz;
    // rexquad::PoseToBytes(buf_pose, posemsg);

    // Send pose to transmitter 
    mocap_queue.push(posemsg);
    int bytes_sent = 0;
    if (mocap_queue.size() > delay) {
      pose_delayed = std::move(mocap_queue.front());
      mocap_queue.pop();
      rexquad::PoseToBytes(buf_pose, pose_delayed);
      bytes_sent = sp_blocking_write(tx, buf_pose, posemsg_len, send_timeout_ms);
    }
    if (verbose) {
      fmt::print("\nReceived {} bytes over ZMQ\n", zmq_bytes_received);
      fmt::print("  Successful conversion to MeasurementMsg: {}\n", good_conversion);
      print_msg(posemsg);
      fmt::print("  Queue size = {}\n", mocap_queue.size());
      if (bytes_sent) {
        fmt::print("Message sent ({} bytes):\n", bytes_sent);
        print_msg(pose_delayed);
      }
    }
  }

  return EXIT_SUCCESS;
}