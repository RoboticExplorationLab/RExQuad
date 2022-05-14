#include <fmt/core.h>
#include <fmt/ostream.h>
#include <libserialport.h>
#include <zmq.h>

#include <common/pose.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

#include "common/pose.hpp"
#include "utils/serial.hpp"

int main(int argc, char** argv) {
  std::string port = "5556";
  if (argc == 2) {
    port = argv[1];
  }

  // Open Serial port (to LoRa rx)
  std::string rx_name = "/dev/ttyACM1";
  int baudrate = 57600;
  struct sp_port* rx = rexquad::InitializeSerialPort(rx_name, baudrate);
  fmt::print("Connected to Receiver\n");

  // Set up ZMQ publisher
  void* context = zmq_ctx_new();
  void* pub = zmq_socket(context, ZMQ_PUB);
  std::string tcpaddress = "tcp://127.0.0.1:" + port; 
  int rc = zmq_bind(pub, tcpaddress.c_str());
  if (rc != 0) {
    printf("Failed to connect.\n");
    return 1;
  }

  constexpr int len = sizeof(rexquad::PoseMsg) + 1;
  char buf_in[len];
  char buf_out[len];
  const int timeout_ms = 100;
  rexquad::PoseMsg posemsg = {};
  while (true) {
    int bytes_received = sp_blocking_read(rx, buf_in, len, timeout_ms);
    if (bytes_received >= len) {
      rexquad::PoseFromBytes(posemsg, buf_in);
      fmt::print("\nGot Pose:\n");
      fmt::print("  x = [{:.3f}, {:.3f}, {:.3f}]\n", posemsg.x, posemsg.y, posemsg.z);
      fmt::print("  q = [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", posemsg.qw, posemsg.qx,
                 posemsg.qy, posemsg.qz);
      memcpy(buf_out, buf_in, len);
      zmq_send(pub, buf_out, len, 0);
      fmt::print("  ZMQ Message sent.\n");
    }
  }
}