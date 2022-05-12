#include "callbacks.hpp"
#include "NatNetTypes.h"
#include "fmt/core.h"
#include "common/utils.hpp"
#include "utils/serial.hpp"
#include <string>

#include <gtest/gtest.h>
#include <unistd.h>

int main() {

    const char* port = "/dev/ttyACM0";
    int baudrate = 57600;
    rexquad::SerialCallback lora(port, baudrate);
    lora.SetTimeout(200);

    sRigidBodyData rbdata; 
    std::vector<std::string> ports = rexquad::GetPortList();
    for (auto& port : ports) {
        fmt::print("{}\n", port);
    }
    lora.Open();
    for (int i = 0; i < 1000; ++i) {
        fmt::print("Sending message {}\n", i);
        lora(rbdata);
        // usleep(1000);
    }
    lora.Close();

    return 0;
}
