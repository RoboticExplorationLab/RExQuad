#pragma once

#include <chrono>
#include <inttypes.h>
#include <string>
#include <stdexcept>
#include <vector>

#include <libserialport.h>

namespace rexquad {

std::vector<std::string> GetPortList();

struct sp_port* InitializeSerialPort(std::string& port, int baudrate);

class LibSerialPortError : public std::runtime_error {
 public:
  LibSerialPortError(std::string arg) : std::runtime_error(std::move(arg)) {}
};

/**
 * @brief Check return codes for libserial and throw errors as necessary
 * 
 * @param result true if no errors are round
 * @throw LibSerialPortError if an error code is read 
 */
bool LibSerialCheck(enum sp_return result);

void HandleLibSerialError(enum sp_return result);

class RatePrinter {
 public:
  void Init();
  void SetFrequency(float frequency);
  bool IsEnabled();
  void Enable();
  void Disable();
  void Print();
 private:
  std::chrono::system_clock::time_point time_start_;
  std::chrono::duration<double, std::ratio<1>> period_ = std::chrono::seconds(1);
  bool is_enabled_;
  int count_ = 0;
};

}  // namespace rexquad