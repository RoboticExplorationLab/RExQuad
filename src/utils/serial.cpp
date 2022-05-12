#include "serial.hpp"

#include <fmt/core.h>

namespace rexquad {

std::vector<std::string> GetPortList() {
  std::vector<std::string> ports;

  struct sp_port** port_list;
  enum sp_return result = sp_list_ports(&port_list);
  if (result != SP_OK) {
    fmt::print("Couldn't get the list of ports\n");
  }

  int i;
  for (i = 0; port_list[i] != NULL; i++) {
    struct sp_port* port = port_list[i];
    ports.emplace_back(sp_get_port_name(port));
  }
  return ports;
}

struct sp_port* InitializeSerialPort(std::string& portpath, int baudrate) {
  struct sp_port* port;

  rexquad::HandleLibSerialError(sp_get_port_by_name(portpath.c_str(), &port));
  fmt::print("Port name: {}\n", sp_get_port_name(port));
  fmt::print("Port description: {}\n", sp_get_port_description(port));
  rexquad::HandleLibSerialError(sp_open(port, SP_MODE_READ_WRITE));
  rexquad::HandleLibSerialError(sp_set_baudrate(port, baudrate));
  rexquad::HandleLibSerialError(sp_set_bits(port, 8));
  rexquad::HandleLibSerialError(sp_set_parity(port, SP_PARITY_NONE));
  rexquad::HandleLibSerialError(sp_set_stopbits(port, 1));
  rexquad::HandleLibSerialError(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));
  return port;
}


/* Helper function for error handling. */
bool LibSerialCheck(enum sp_return result) {
  char* err_msg;
  switch (result)
  {
  case SP_ERR_ARG:
    throw(LibSerialPortError("Error: Invalid argument."));
    break;

  case SP_ERR_FAIL:
    err_msg = sp_last_error_message();
    throw(LibSerialPortError("Error: Failed: " + std::string(err_msg)));
    sp_free_error_message(err_msg);
    break;

  case SP_ERR_SUPP:
    throw(LibSerialPortError("Error: Not Supported."));
    break;

  case SP_ERR_MEM:
    throw(LibSerialPortError("Error: Couldn't allocate memory."));
    break;

  case SP_OK:
  default:
    return true;
  }
}

void HandleLibSerialError(enum sp_return result) {
  try {
    LibSerialCheck(result);
  } catch (LibSerialPortError& e) {
    fmt::print("Error Calling libserialport\n");
    char* err_msg = sp_last_error_message();
    fmt::print("Got error: {}\n", err_msg);
    sp_free_error_message(err_msg);
  }
}

void RatePrinter::Init() {
  time_start_ = std::chrono::high_resolution_clock::now();
  count_ = 0;
}

void RatePrinter::Enable() { is_enabled_ = true; }
void RatePrinter::Disable() { is_enabled_ = false; }
bool RatePrinter::IsEnabled() { return is_enabled_; } 
void RatePrinter::SetFrequency(float frequency) { 
  period_ = std::chrono::duration<double, std::ratio<1>>(1.0 / frequency);
}
void RatePrinter::Print() {
  if (IsEnabled()) {
    std::chrono::duration<double, std::ratio<1>> t_elapsed = std::chrono::high_resolution_clock::now() - time_start_;
    if (t_elapsed > period_) {
      double average_rate = count_ / t_elapsed.count();

      fmt::print("Average rate: {}\n", average_rate);
      count_ = 0;
      time_start_ = std::chrono::high_resolution_clock::now();
    }
    ++count_;
  }
}

}