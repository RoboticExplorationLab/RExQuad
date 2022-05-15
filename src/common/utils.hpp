#pragma once
#include <string>

namespace rexquad {

std::string MakeTcpAddress(const std::string& addr);

class TcpAddress {
public:
  TcpAddress();
  TcpAddress(std::string addr_, int port);
  TcpAddress(std::string addr_);
  TcpAddress(TcpAddress&& other);
  std::string ToString();
  static constexpr int kAnyPort = -1; 

 private:
  std::string addr_;
  int port_ = kAnyPort;
};

float bytestofloat(uint8_t* buf, int off);
void floattobytes(uint8_t* buf, float x, int off);

}