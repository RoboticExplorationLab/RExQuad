#include "callbacks.hpp"

#include <string.h>
#include <chrono>

#include "common/pose.hpp"
#include "common/utils.hpp"
#include "fmt/core.h"
#include "fmt/chrono.h"
#include "utils/serial.hpp"

namespace rexquad {

void RigidBodyToPose(PoseMsg& pose, const sRigidBodyData& data) {
  pose.x = data.x;
  pose.y = data.y;
  pose.z = data.z;
  pose.qw = data.qw;
  pose.qx = data.qx;
  pose.qy = data.qy;
  pose.qz = data.qz;
}

SerialCallback::SerialCallback(const std::string& port_name, int baud_rate)
    : port_name_(port_name), baud_rate_(baud_rate) {}

SerialCallback::~SerialCallback() { this->Close(); }

bool SerialCallback::Open() {
  try {
    struct sp_port* port;
    std::string port_name = port_name_;
    int baud_rate = baud_rate_;
    fmt::print("Looking for port {}\n", port_name);
    LibSerialCheck(sp_get_port_by_name(port_name.c_str(), &port));

    fmt::print("Port name: {}\n", sp_get_port_name(port));
    fmt::print("Port description: {}\n", sp_get_port_description(port));

    fmt::print("Opening Port.\n");
    enum sp_return result = sp_open(port, SP_MODE_READ_WRITE);
    if (result != SP_OK) {
      fmt::print("Couldn't open serial port\n");
      char* err_msg = sp_last_error_message();
      fmt::print("Got error: {}\n", err_msg);
      sp_free_error_message(err_msg);
    }

    fmt::print("Setting port to baudrate {}\n", baud_rate);
    LibSerialCheck(sp_set_baudrate(port, baud_rate));
    LibSerialCheck(sp_set_bits(port, 8));
    LibSerialCheck(sp_set_parity(port, SP_PARITY_NONE));
    LibSerialCheck(sp_set_stopbits(port, 1));
    LibSerialCheck(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

    port_ = port;
    is_open_ = true;
    return true;
  } catch (const LibSerialPortError& e) {
    fmt::print("Error Calling libserialport\n");
    char* err_msg = sp_last_error_message();
    fmt::print("Got error: {}\n", err_msg);
    sp_free_error_message(err_msg);
  }
  return false;
}

void SerialCallback::Close() {
  if (IsOpen()) {
    fmt::print("Closing port...\n");
    sp_close(port_);
    fmt::print("Port closed!\n");
    sp_free_port(port_);
    is_open_ = false;
  }
}

void SerialCallback::SetTimeout(int time_ms) {
  timeout_ = std::chrono::milliseconds(time_ms);
}

void SerialCallback::operator()(const sRigidBodyData& data) {
  RigidBodyToPose(pose_, data);
  PoseToBytes(buf_, pose_);
  WriteBytes(buf_, sizeof(PoseMsg) + 1);
}

int SerialCallback::WriteBytes(const char* data, size_t size) {
  // fmt::print("Writing to serial...\n");
  int skip = 1;
  int batch = 10;
  static int sendcount = 1;
  if (IsOpen()) {
    // Check for input from the serial port and print to stdout
    if (check_for_input_ && (sp_input_waiting(port_) > 0)) {
      int input_bytes = sp_input_waiting(port_);
      fmt::print("Got message of length {} from Feather: ", input_bytes);
      char* tempbuf = (char*)malloc(input_bytes);
      sp_blocking_read(port_, tempbuf, std::min(1000, input_bytes), 100);

      // Convert char array to valid string
      std::string message = "";
      for (int i = 0; i < input_bytes; ++i) {
        message += tempbuf[i];
      }
      fmt::print("{}", message);
    }

    int bytes = 0;
    if (sendcount % skip == 0) {
      // fmt::print("Sending Pose Message {} over Serial: ", sendcount);
      // fmt::print("position = [{:.3f}, {:.3f}, {:.3f}]\n", pose_.x, pose_.y, pose_.z);

      // Write the pose data to the serial port
      // fmt::print("Writing data\n");
      bytes = sp_blocking_write(port_, data, size, timeout_.count());
      (void) data;
      (void) size;
      // fmt::print("Finished writing data. Wrote {} bytes\n", bytes);
      // sp_drain(port_);  // make sure the transmission is complete before continuing
      // fmt::print("Finished drain\n");

    }
    if (sendcount % batch == 0) {
      auto tcur = std::chrono::high_resolution_clock::now();
      auto t_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(tcur - tstart_);
      double rate = static_cast<double>(batch) / static_cast<double>(t_elapsed_us.count()) * 1e6;
      fmt::print("Average rate = {} Hz\n", rate);
      // fmt::print("Time elapsed = {}\n", t_elapsed_us);
      tstart_ = tcur;
    }
    if (sendcount == 0) {
      tstart_ = std::chrono::high_resolution_clock::now(); 
    }
    ++sendcount;
    return bytes;
  } else {
    fmt::print("WARNING: Trying to write to a closed port!\n");
  }
  return 0;
}

// void ZMQCallback::WriteBytes(const char* data, size_t size) {
//   // fmt::print("Sending ZMQ message...\n");
//   socket_.send(zmq::message_t(data, size), zmq::send_flags::none);
// }

}  // namespace rexquad