
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <vector>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include "callbacks.hpp"
#include "libserialport.h"
#include "fmt/core.h"

#include "Mocap.hpp"

// Globals
static const char* SERVER_ADDR = "192.168.1.5";
static const char* LOCAL_ADDR = "192.168.1.63";

int main( int argc, char* argv[] )
{
    rexquad::Mocap mocap = {};

    // Add callbacks
    rexquad::PrintCallback printcallback = {};

    const char* port = "ttyACM0";
    int baudrate = 57600;
    rexquad::SerialCallback lora(port, baudrate);

    mocap.AddCallback(std::ref(printcallback));
    mocap.AddCallback(std::ref(lora));

    // Connect to Server
    mocap.SetLocalAddress(LOCAL_ADDR);
    mocap.SetServerAddress(SERVER_ADDR);
    if ( argc == 1 ) {
    } else {
        if ( argc >= 2 ) {
            mocap.SetLocalAddress(argv[2]);
        }

        if ( argc >= 3 ) {
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

