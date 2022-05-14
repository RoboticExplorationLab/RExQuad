#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <libserialport.h>
#include <zmq.h>

#include "common/pose.hpp"
#include "utils/serial.hpp"

float bytestofloat(uint8_t* buf, int off) {
  uint32_t b0 = static_cast<uint32_t>(buf[0 + off]);
  uint32_t b1 = static_cast<uint32_t>(buf[1 + off]) << 8;
  uint32_t b2 = static_cast<uint32_t>(buf[2 + off]) << 16;
  uint32_t b3 = static_cast<uint32_t>(buf[3 + off]) << 24;
  uint32_t b = b0 | b1 | b2 | b3;
  return reinterpret_cast<float&>(b);
}

struct MeasurementMsg {
  float x;
  float y;
  float z;
  float qw;
  float qx;
  float qy;
  float qz;
  float ax;
  float ay;
  float az;
  float wx;
  float wy;
  float wz;
};

void MeasurementMsgFromBytes(MeasurementMsg& msg, uint8_t* buf) {
  msg.x = bytestofloat(buf, 0 * 4);
  msg.y = bytestofloat(buf, 1 * 4);
  msg.z = bytestofloat(buf, 2 * 4);
  msg.qw = bytestofloat(buf, 3 * 4);
  msg.qx = bytestofloat(buf, 4 * 4);
  msg.qy = bytestofloat(buf, 5 * 4);
  msg.qz = bytestofloat(buf, 6 * 4);
  msg.ax = bytestofloat(buf, 7 * 4);
  msg.ay = bytestofloat(buf, 8 * 4);
  msg.az = bytestofloat(buf, 9 * 4);
  msg.wx = bytestofloat(buf, 10 * 4);
  msg.wy = bytestofloat(buf, 11 * 4);
  msg.wz = bytestofloat(buf, 12 * 4);
}

void print_msg(const MeasurementMsg& msg) {
  fmt::print("Translation:  [{:.3f}, {:.3f}, {:.3f}]\n", msg.x, msg.y, msg.z);
  fmt::print("Orientation:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.qw, msg.qx, msg.qy, msg.qz);
  fmt::print("Lin Accel:    [{:.3f}, {:.3f}, {:.3f}]\n", msg.ax, msg.ay, msg.az);
  fmt::print("Ang Velocity: [{:.3f}, {:.3f}, {:.3f}]\n", msg.wx, msg.wy, msg.wz);
}

int main(void) {
  // Open Serial port
  std::string rx_name = "/dev/ttyACM0";
  int baudrate = 57600;
  struct sp_port* tx = rexquad::InitializeSerialPort(rx_name, baudrate);
  fmt::print("Connected to Receiver\n");
  (void)tx;

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
  rc = zmq_connect(sub, "tcp://127.0.0.1:5557");
  if (rc != 0) {
    printf("Failed to connect.\n");
    return 1;
  }
  rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
  if (rc != 0) {
    printf("Failed to set subscriber.\n");
    return 1;
  }

  printf("Waiting for messages...\n");
  int len = sizeof(MeasurementMsg);
  uint8_t buffer[sizeof(MeasurementMsg)];
  char buf_pose[sizeof(rexquad::PoseMsg)+1];
  int posemsg_len = sizeof(buf_pose);
  const int send_timeout = 100;  // ms

  MeasurementMsg measmsg = {};
  rexquad::PoseMsg posemsg = {};
  while (1) {
    zmq_recv(sub, buffer, len, 0);
    MeasurementMsgFromBytes(measmsg, buffer);

    // Extract Pose information
    posemsg.x = measmsg.x;
    posemsg.y = measmsg.y;
    posemsg.z = measmsg.z;
    posemsg.qw = measmsg.qw;
    posemsg.qx = measmsg.qx;
    posemsg.qy = measmsg.qy;
    posemsg.qz = measmsg.qz;
    rexquad::PoseToBytes(buf_pose, posemsg);
    enum sp_return bytes_sent = sp_blocking_write(tx, buf_pose, posemsg_len, send_timeout);
    (void) bytes_sent;

    print_msg(measmsg);
    fmt::print("\n");
  }
  zmq_close(sub);
  zmq_ctx_destroy(context);
  return 0;
}
