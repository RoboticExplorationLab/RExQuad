#include <chrono>
#include <cstdint>
#include <vector>
#include <unistd.h>

#include <libserialport.h>
#include <fmt/core.h>
#include <fmt/chrono.h>

#include "common/pose.hpp"
#include "common/utils.hpp"
#include "utils/serial.hpp"
#include "mocap/callbacks.hpp"

struct MyMsg {
  static constexpr uint8_t MsgID() { return 120; }
  float x;
};
using Pose = rexquad::PoseMsg;
constexpr int MSG_SIZE = sizeof(Pose);

int main() { 
  // Open Serial ports
  std::string tx_name = "/dev/ttyACM0";
  int baudrate = 57600;
  rexquad::SerialCallback tx(tx_name, baudrate);
  tx.Open();
  fmt::print("Connected to Transmitter\n");


  std::string rx_name = "/dev/ttyACM1";
  struct sp_port* rx = rexquad::InitializeSerialPort(rx_name, baudrate);
  fmt::print("Connected to Receiver\n");

  // Send and receive floats
  const int nsamples = 400;
  char buf[MSG_SIZE+1];
  char recv[100];
  Pose msg;
  Pose msg_recv;
  msg.x = 0.0;
  msg_recv.x = -1.0;
  std::vector<std::pair<double, float>> datasent;
  std::vector<std::pair<double, float>> datarecv;
  datasent.reserve(nsamples);
  datarecv.reserve(nsamples);
  auto tstart = std::chrono::high_resolution_clock::now();
  using fmillisecond = std::chrono::duration<float, std::milli>;

  // Sending info
  fmt::print("\nSending...\n");
  buf[0] = Pose::MsgID();
  for (int i = 0; i < MSG_SIZE; ++i) {
    buf[i+1] = i + 'a';
  }
  memcpy(&msg, buf+1, MSG_SIZE);
  float x = msg.x;
  fmt::print("  Sent x = {}\n", x);
  tx.SetTimeout(100);
  auto t_send = std::chrono::high_resolution_clock::now();
  int bytes_sent = tx.WriteBytes(buf, MSG_SIZE+1);

  // Receiving
  fmt::print("Receiving...\n");
  int bytes_received = sp_blocking_read(rx, recv, 100, 100);
  auto t_recv = std::chrono::high_resolution_clock::now();
  auto latency = std::chrono::duration_cast<fmillisecond>(t_recv - t_send);

  fmt::print("Summary:\n");
  fmt::print("  Sent {} bytes\n", (int)bytes_sent);
  fmt::print("  Message: [ ");
  for (int i = 0; i < (int)bytes_sent; ++i) {
    fmt::print("{} ", (int)buf[i]);
  }
  fmt::print("]\n");
  fmt::print("  {} bytes received\n", bytes_received);
  fmt::print("  Message from Rx: [ ");
  for (int i = 0; i < bytes_received; ++i) {
    fmt::print("{} ", (int)recv[i]);
  }
  fmt::print("]\n");
  memcpy(&msg_recv, recv+1, MSG_SIZE);
  float x_recv = msg_recv.x;
  fmt::print("  Got x = {}\n", x_recv);
  fmt::print("Latency {}\n\n", latency);

  // return 0;
  tx.SetTimeout(2);
  for (int i = 0; i < nsamples; ++i) {
    float x_sent = i * 0.001;
    msg_recv.x = -1.0;
    msg.x = x_sent; 

    // Send data
    memcpy(buf+1, &msg, MSG_SIZE);
    buf[0] = Pose::MsgID();
    auto tsend = std::chrono::duration_cast<fmillisecond>(
        std::chrono::high_resolution_clock::now() - tstart);
    // enum sp_return bytes_sent = sp_blocking_write(tx, buf, MSG_SIZE+1, 2);
    int bytes_sent = tx.WriteBytes(buf, MSG_SIZE+1);

    // Receive
    int bytes_received = sp_blocking_read(rx, recv, MSG_SIZE+1, 20);
    auto trecv = std::chrono::duration_cast<fmillisecond>(
        std::chrono::high_resolution_clock::now() - tstart);

    (void) bytes_sent;
    (void) bytes_received;

    // Record info 
    fmt::print("Sample {}\n", i);
    fmt::print("  Sent {} bytes, x = {}\n", (int)bytes_sent, msg.x);
    memcpy(&msg_recv, recv+1, MSG_SIZE);
    fmt::print("  Received {} bytes, x = {}, ID = {}\n", bytes_received, msg_recv.x, (int)recv[0]);
    fmt::print("  Latency {} ms\n", trecv - tsend);
    int id_index = 0;
    for (; id_index < MSG_SIZE + 1; ++id_index) {
      if (recv[id_index] == Pose::MsgID()) {
        break;
      }
    }
    fmt::print("  ID index: {}\n", id_index);

    datasent.emplace_back(std::make_pair(tsend.count(), x_sent));
    datarecv.emplace_back(std::make_pair(trecv.count(), msg.x));
    // usleep(1 * 1000);
  }
  
  // Write data to file
  FILE* outfile = fopen("src/mocap/test/serial_chain_latency_test_out.txt", "w");
  FILE* infile = fopen("src/mocap/test/serial_chain_latency_test_in.txt", "w");
  for (auto& pair : datasent) {
    fmt::print(outfile, "{:0.4f}, {:0.5f}\n", pair.first, pair.second); 
  }
  for (auto& pair : datarecv) {
    fmt::print(infile, "{:0.4f}, {:0.5f}\n", pair.first, pair.second); 
  }
  fclose(outfile);
  fclose(infile);
  sp_free_port(rx);
  return 0;
}