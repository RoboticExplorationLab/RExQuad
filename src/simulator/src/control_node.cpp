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
  std::string port = "5555";
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

  char buf[sizeof(rexquad::PoseMsg) + 1];
  const int len = sizeof(buf);
  const int timeout_ms = 100;
  rexquad::PoseMsg posemsg = {};
  while (true) {
    int bytes_received = sp_blocking_read(rx, buf, len, timeout_ms);
    if (bytes_received >= len) {
      rexquad::PoseFromBytes(posemsg, buf);
      fmt::print("\nGot Pose:\n");
      fmt::print("  x = [{:.3f}, {:.3f}, {:.3f}]\n", posemsg.x, posemsg.y, posemsg.z);
      fmt::print("  q = [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", posemsg.qw, posemsg.qx,
                 posemsg.qy, posemsg.qz);
    }
  }
}