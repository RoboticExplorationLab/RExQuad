#include "utils.hpp"
#include <string>
#include <cstring>

#include <stdlib.h>


namespace rexquad {

 
std::string MakeTcpAddress(const std::string& addr) {
  std::string tcp_addr;
  if (addr.rfind("tcp://") == 0) {
    tcp_addr = addr;
  } else {
    tcp_addr = "tcp://" + addr;
  }
  return tcp_addr;
}

TcpAddress::TcpAddress() : addr_(MakeTcpAddress("localhost")) {}
TcpAddress::TcpAddress(std::string addr) : addr_(MakeTcpAddress(std::move(addr))) {}
TcpAddress::TcpAddress(std::string addr, int port) 
    : addr_(MakeTcpAddress(addr)), port_(std::move(port)) {}
TcpAddress::TcpAddress(TcpAddress&& other) : addr_(other.addr_), port_(other.port_) {}

std::string TcpAddress::ToString() {
  std::string port_string;
  if (port_ == kAnyPort) {
    port_string = ":*";
  } else {
    char buf[20];
    itoa(port_, buf, 20);
    port_string = ":" + std::string(buf);
  }
  return addr_ + port_string;
}

float bytestofloat(uint8_t* buf, int off) {
  uint32_t b0 = static_cast<uint32_t>(buf[0 + off]);
  uint32_t b1 = static_cast<uint32_t>(buf[1 + off]) << 8;
  uint32_t b2 = static_cast<uint32_t>(buf[2 + off]) << 16;
  uint32_t b3 = static_cast<uint32_t>(buf[3 + off]) << 24;
  uint32_t b = b0 | b1 | b2 | b3;
  float f = 0;
  memcpy(&f, &b, sizeof(f));
  return f;
}

void floattobytes(uint8_t* buf, float x, int off) {
  uint32_t mask = 0xff;
  uint32_t xint = 0;
  memcpy(&xint, &x, sizeof(xint));
  // uint32_t xint = reinterpret_cast<uint32_t&>(x);
  buf[0 + off] = static_cast<uint8_t>(xint & mask);
  buf[1 + off] = static_cast<uint8_t>((xint >> 8) & mask);
  buf[2 + off] = static_cast<uint8_t>((xint >> 16) & mask);
  buf[3 + off] = static_cast<uint8_t>((xint >> 24) & mask);
}

}  // namespace rexquad