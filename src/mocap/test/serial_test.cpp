#include <string>

#include <gtest/gtest.h>
#include <NatNetTypes.h>
#include <fmt/core.h>

#include "callbacks.hpp"
#include "common/utils.hpp"
#include "utils/serial.hpp"


TEST(LORATEST, Connection) {
    const char* port = "/dev/ttyACM0";
    int baudrate = 57600;
    rexquad::SerialCallback lora(port, baudrate);

    sRigidBodyData rbdata; 
    std::vector<std::string> ports = rexquad::GetPortList();
    for (auto& port : ports) {
        fmt::print("{}\n", port);
    }
    lora.Open();
    EXPECT_TRUE(lora.IsOpen());
}