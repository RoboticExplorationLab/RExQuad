#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <libserialport.h>
#include <zmq.h>
#include <string>

#include "common/lqr_constants.hpp"
#include "common/messages.hpp"
#include "common/pose.hpp"
#include "common/utils.hpp"
#include "utils/serial.hpp"

extern "C" {
  #include "slap/matrix.h"
}

using namespace rexquad;

// float bytestofloat(uint8_t* buf, int off) {
//   uint32_t b0 = static_cast<uint32_t>(buf[0 + off]);
//   uint32_t b1 = static_cast<uint32_t>(buf[1 + off]) << 8;
//   uint32_t b2 = static_cast<uint32_t>(buf[2 + off]) << 16;
//   uint32_t b3 = static_cast<uint32_t>(buf[3 + off]) << 24;
//   uint32_t b = b0 | b1 | b2 | b3;
//   return reinterpret_cast<float&>(b);
// }

// struct MeasurementMsg {
//   float x;
//   float y;
//   float z;
//   float qw;
//   float qx;
//   float qy;
//   float qz;
//   float ax;
//   float ay;
//   float az;
//   float wx;
//   float wy;
//   float wz;
// };

// void MeasurementMsgFromBytes(MeasurementMsg& msg, uint8_t* buf) {
//   msg.x = bytestofloat(buf, 0 * 4);
//   msg.y = bytestofloat(buf, 1 * 4);
//   msg.z = bytestofloat(buf, 2 * 4);
//   msg.qw = bytestofloat(buf, 3 * 4);
//   msg.qx = bytestofloat(buf, 4 * 4);
//   msg.qy = bytestofloat(buf, 5 * 4);
//   msg.qz = bytestofloat(buf, 6 * 4);
//   msg.ax = bytestofloat(buf, 7 * 4);
//   msg.ay = bytestofloat(buf, 8 * 4);
//   msg.az = bytestofloat(buf, 9 * 4);
//   msg.wx = bytestofloat(buf, 10 * 4);
//   msg.wy = bytestofloat(buf, 11 * 4);
//   msg.wz = bytestofloat(buf, 12 * 4);
// }

void print_msg(const MeasurementMsg& msg) {
  fmt::print("  Translation:  [{:.3f}, {:.3f}, {:.3f}]\n", msg.x, msg.y, msg.z);
  fmt::print("  Orientation:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.qw, msg.qx, msg.qy, msg.qz);
  fmt::print("  Lin Accel:    [{:.3f}, {:.3f}, {:.3f}]\n", msg.ax, msg.ay, msg.az);
  fmt::print("  Ang Velocity: [{:.3f}, {:.3f}, {:.3f}]\n", msg.wx, msg.wy, msg.wz);
}

void print_msg(const PoseMsg& msg) {
  fmt::print("  Translation:  [{:.3f}, {:.3f}, {:.3f}]\n", msg.x, msg.y, msg.z);
  fmt::print("  Orientation:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.qw, msg.qx, msg.qy, msg.qz);
}

void print_msg(const ControlMsg& msg) {
  fmt::print("  Controls:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
}

void print_msg(const StateControlMsg& msg) {
  fmt::print("  Translation:  [{:.3f}, {:.3f}, {:.3f}]\n", msg.x, msg.y, msg.z);
  fmt::print("  Orientation:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.qw, msg.qx, msg.qy, msg.qz);
  fmt::print("  Lin Velocity: [{:.3f}, {:.3f}, {:.3f}]\n", msg.vx, msg.vy, msg.vz);
  fmt::print("  Ang Velocity: [{:.3f}, {:.3f}, {:.3f}]\n", msg.wx, msg.wy, msg.wz);
  fmt::print("  Controls:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.u[0], msg.u[1], msg.u[2], msg.u[3]);
}

int main(int argc, char** argv) {
  Matrix K = slap_NewMatrix(4, 12);
  slap_MatrixCopyFromArray(&K, rexquad::kFeedbackGain);
  slap_PrintMatrix(&K);
  slap_FreeMatrix(&K);
  std::string pubport = "5555";
  std::string subport = "5556";
  if (argc > 1) {
    subport = argv[1];
  }
  if (argc > 2) {
    pubport = argv[2];
  }
  const bool poseonly = false;

  // Open Serial port to transmitter
  std::string tx_name = "/dev/ttyACM0";
  int baudrate = 57600;
  struct sp_port* tx = rexquad::InitializeSerialPort(tx_name, baudrate);
  fmt::print("Connected to Transmitter\n");

  // Open Serial port to onboard Feather 
  std::string rx_name = "/dev/ttyACM1";
  struct sp_port* onboard = rexquad::InitializeSerialPort(rx_name, baudrate);
  fmt::print("Connected to Onboard Feather\n");

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

  // Set up ZMQ publisher
  void* pub = zmq_socket(context, ZMQ_PUB);
  std::string tcpaddress_pub = "tcp://127.0.0.1:" + pubport;
  rc = zmq_connect(pub, tcpaddress_pub.c_str());
  if (rc != 0) {
    printf("Failed to connect publisher.\n");
    return 1;
  } else {
    fmt::print("Connected publisher at {}\n", tcpaddress_pub);
  }

  // Receiving ZMQ message from simulator
  constexpr int len_zmqrecv = sizeof(MeasurementMsg) + 1;
  uint8_t buf_zmqrecv[len_zmqrecv];

  // Sending
  char buf_pose[sizeof(rexquad::PoseMsg)+1];
  int posemsg_len = sizeof(buf_pose);
  const int send_timeout_ms = 100;  // ms
  MeasurementMsg measmsg = {};
  rexquad::PoseMsg posemsg = {};

  // Receiving StateControl over Serial
  constexpr int len_serialrecv = sizeof(StateControlMsg) + 1;
  uint8_t buf_serialrecv[len_serialrecv];
  const int recv_timeout_ms = 1000; 
  StateControlMsg statecontrol_onboard = {};

  // Sending StateControl over ZMQ
  constexpr int len_zmqsend = sizeof(StateControlMsg) + 1;
  uint8_t buf_zmqsend[len_zmqsend];

  fmt::print("Size of control: {}\n", sizeof(rexquad::ControlMsg));
  printf("Waiting for messages...\n");
  while (1) {
    int zmq_bytes_received = zmq_recv(sub, buf_zmqrecv, len_zmqrecv, 0);
    fmt::print("\nReceived {} bytes over ZMQ\n", zmq_bytes_received);
    bool good_conversion = MeasurementMsgFromBytes(measmsg, buf_zmqrecv);
    fmt::print("  Successful conversion to MeasurementMsg: {}\n", good_conversion);
    fmt::print("  Ang Velocity = [{:.3f}, {:.3f}, {:.3f}]\n", measmsg.wx, measmsg.wy, measmsg.wz);

    // Send entire message to onboard feather over serial
    if (!poseonly) {
      sp_blocking_write(onboard, buf_zmqrecv, len_zmqrecv, send_timeout_ms);
    }
    (void) onboard;

    // Extract Pose information
    posemsg.x = measmsg.x;
    posemsg.y = measmsg.y;
    posemsg.z = measmsg.z;
    posemsg.qw = measmsg.qw;
    posemsg.qx = measmsg.qx;
    posemsg.qy = measmsg.qy;
    posemsg.qz = measmsg.qz;
    rexquad::PoseToBytes(buf_pose, posemsg);

    // Send pose to LoRa tx
    sp_blocking_write(tx, buf_pose, posemsg_len, send_timeout_ms);
    fmt::print("Message sent:\n");
    print_msg(measmsg);

    // Receive the state and control message back
    sp_return bytes_received = sp_blocking_read(onboard, buf_serialrecv, len_serialrecv, recv_timeout_ms);
    rexquad::StateControlMsgFromBytes(statecontrol_onboard, buf_serialrecv);
    // rexquad::PoseFromBytes(pose_recv, (char*) bufrecv);  // first byte should be msgid
    // rexquad::ControlMsgFromBytes(control_recv, bufrecv, posemsg_len + 1);
    fmt::print("Receieved {} / {} bytes:\n", (int) bytes_received, len_serialrecv);
    print_msg(statecontrol_onboard);

    // Send state and control back to simulator
    memcpy(buf_zmqsend, buf_serialrecv, len_serialrecv);
    zmq_send(pub, buf_zmqsend, len_zmqsend, 0);
    fmt::print("  ZMQ Message Sent\n");

  }
  zmq_close(sub);
  zmq_ctx_destroy(context);
  return 0;
}
