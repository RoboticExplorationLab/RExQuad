#include <chrono>
#include <cstring>
#include <fstream>
#include <ratio>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>
#include <ctime>

#include <fmt/chrono.h>
#include <gtest/gtest.h>
#include <libserialport.h>

#include "NatNetTypes.h"
#include "callbacks.hpp"
#include "common/pose.hpp"
#include "fmt/core.h"
#include "common/utils.hpp"
#include "utils/serial.hpp"

using namespace std::chrono_literals;

constexpr int MSG_SIZE = sizeof(rexquad::PoseMsg);

int main() {
  std::vector<std::string> ports = rexquad::GetPortList();
  fmt::print("Available Ports:\n");
  for (auto &port : ports) {
    fmt::print("  {}\n", port);
  }

  struct sp_port *rx;
  std::string rx_name = "/dev/ttyACM1";
  int baudrate = 57600;
  rexquad::LibSerialCheck(sp_get_port_by_name(rx_name.c_str(), &rx));
  rexquad::LibSerialCheck(sp_open(rx, SP_MODE_READ));
  rexquad::LibSerialCheck(sp_set_baudrate(rx, baudrate));
  rexquad::LibSerialCheck(sp_set_bits(rx, 8));
  rexquad::LibSerialCheck(sp_set_parity(rx, SP_PARITY_NONE));
  rexquad::LibSerialCheck(sp_set_stopbits(rx, 1));
  rexquad::LibSerialCheck(sp_set_flowcontrol(rx, SP_FLOWCONTROL_NONE));
  fmt::print("Connected to Receiver\n");

  const char *port = "/dev/ttyACM0";
  rexquad::SerialCallback lora(port, baudrate);
  lora.SetTimeout(200);

  sRigidBodyData rbdata;
  lora.Open();
  fmt::print("Connect to Transmitter\n");

  fmt::print("Starting Loop...\n");
  
  auto tstart = std::chrono::system_clock::now();
  std::chrono::milliseconds runtime = 3s;
  char buf[MSG_SIZE];
  memset(buf, 0, MSG_SIZE);
  rexquad::PoseMsg pose;
  std::vector<std::pair<double, float>> datain;
  std::vector<std::pair<double, float>> dataout;
  dataout.reserve(runtime.count() * 500);
  datain.reserve(runtime.count() * 500);
  using fmillisecond = std::chrono::duration<float, std::milli>;
  sp_flush(rx, SP_BUF_BOTH);
  while (true) {
    auto tcur = std::chrono::system_clock::now();
    auto t_elapsed = std::chrono::duration_cast<fmillisecond>(tcur - tstart);
    // fmt::print("t_elapsed: {}\n", t_elapsed);
    if (t_elapsed > runtime || t_elapsed.count() > 1e6) {
      break;
    }

    int input_bytes = sp_input_waiting(rx);
    if (input_bytes > 0) {
      sp_blocking_read(rx, buf, input_bytes, 0);
      bool successful_parse = rexquad::PoseFromBytes(pose, buf);
      if (successful_parse) {
        datain.emplace_back(std::make_pair(t_elapsed.count(), pose.x));
      } else {
        fmt::print("Parse to PoseMsg wasn't successful.\n");
      }
      // fmt::print("Received {} bytes. x = {}\n", input_bytes, pose.x);
    }
    rbdata.x += 0.001;
    dataout.emplace_back(std::make_pair(t_elapsed.count(), rbdata.x));
    lora(rbdata);
    usleep(1000);
  }

  sp_close(rx);

  FILE* outfile = fopen("src/mocap/test/lora_latency_test_out.txt", "w");
  FILE* infile = fopen("src/mocap/test/lora_latency_test_in.txt", "w");
  for (auto& pair : dataout) {
    fmt::print(outfile, "{:0.4f}, {:0.5f}\n", pair.first, pair.second); 
  }
  for (auto& pair : datain) {
    fmt::print(infile, "{:0.4f}, {:0.5f}\n", pair.first, pair.second); 
  }
  fclose(outfile);
  fclose(infile);


  return 0;
}