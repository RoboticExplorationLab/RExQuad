#include "Mocap.hpp"

#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetTypes.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <vector>

#include "callbacks.hpp"
#include "fmt/core.h"
#include "libserialport.h"

// Globals
static const char* SERVER_ADDR = "192.168.1.66";
static const char* LOCAL_ADDR = "192.168.1.68";

int main(int argc, char* argv[]) {
  rexquad::Mocap mocap = {};
  mocap.SetVerbose(false);

  // Add callbacks
  rexquad::PrintCallback printcallback = {};

  const char* port = "/dev/ttyACM3";
  int baudrate = 256000;
  rexquad::SerialCallback radio(port, baudrate);
  radio.Open();

  float framerate = mocap.GetFramerate();
  fmt::print("Framerate = {} Hz\n", framerate);

//   mocap.AddCallback(std::ref(printcallback));
  mocap.AddCallback(std::ref(radio));

  // Connect to Server
  mocap.SetLocalAddress(LOCAL_ADDR);
  mocap.SetServerAddress(SERVER_ADDR);
  if (argc == 1) {
  } else {
    if (argc >= 2) {
      mocap.SetLocalAddress(argv[2]);
    }

    if (argc >= 3) {
      mocap.SetServerAddress(argv[3]);
    }
  }

  // Connect to Motive
  int iResult;
  iResult = mocap.ConnectClient();
  if (iResult != ErrorCode_OK) {
    printf("Error initializing client. See log for details. Exiting.\n");
    return 1;
  } else {
    printf("Client initialized and ready.\n");
  }

  // Send/receive test request
  mocap.SendTestRequest();

  // Run main loop
  printf("\nClient is connected to server and listening for data...\n");
  mocap.Run();

  return ErrorCode_OK;
}
