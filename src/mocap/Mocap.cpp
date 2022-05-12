#include "Mocap.hpp"
#include <NatNetClient.h>
#include <NatNetTypes.h>

namespace rexquad {

void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server

Mocap::Mocap() {
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );

    // Install logging callback
    NatNet_SetLogCallback( MessageHandler );

    // create NatNet client
    _pClient = new NatNetClient();
}

Mocap::~Mocap() {
    if (_pClient) {
        _pClient->Disconnect();
        delete _pClient;
        _pClient = NULL;
    }
}

void Mocap::SetLocalAddress(const std::string &addr) {
    _local_address = addr;
}

void Mocap::SetServerAddress(const std::string &addr) {
    _server_address = addr;
}

const sServerDescription& Mocap::GetServerDescription() const {
    return _serverDescription;
}

void Mocap::AddCallback(const CallbackFunction &callback) {
  _callbacks.emplace_back(callback);
}

void Mocap::RunCallbacks(const sRigidBodyData& rigidbody) {
    for (auto& callback : _callbacks) {
        callback(rigidbody);
    }
}

int Mocap::ConnectClient() {
    _connectParams.serverAddress = _server_address.c_str(); 
    _connectParams.localAddress = _local_address.c_str();

    // Release previous server
    _pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = _pClient->Connect( _connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &_serverDescription, 0, sizeof( _serverDescription ) );
        ret = _pClient->GetServerDescription( &_serverDescription );
        if ( ret != ErrorCode_OK || ! _serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", _serverDescription.szHostApp, _serverDescription.HostAppVersion[0],
            _serverDescription.HostAppVersion[1], _serverDescription.HostAppVersion[2], _serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", _serverDescription.NatNetVersion[0], _serverDescription.NatNetVersion[1],
            _serverDescription.NatNetVersion[2], _serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", _connectParams.localAddress );
        printf("Server IP:%s\n", _connectParams.serverAddress );
        printf("Server Name:%s\n", _serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = _pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = _pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            _analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", _analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

void Mocap::ResetClient()
{
    int iSuccess;

    printf("\n\nre-setting Client\n\n.");

    iSuccess = _pClient->Disconnect();
    if(iSuccess != 0)
        printf("error un-initting Client\n");

    iSuccess = _pClient->Connect( _connectParams );
    if(iSuccess != 0)
        printf("error re-initting Client\n");
}

void Mocap::SendTestRequest() {
    void* response;
    int nBytes;
    printf("[SampleClient] Sending Test Request\n");
    int iResult = _pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
    if (iResult == ErrorCode_OK)
    {
        printf("[SampleClient] Received: %s\n", (char*)response);
    }
}

int Mocap::Run() {
    // set the frame callback handler
    _pClient->SetFrameReceivedCallback(DataHandler, this);	// this function will receive data from the server

    bool bExit = false;
    int iResult;
    void* response;
    int nBytes;
    while ( const int c = getchar() )
    {
        switch(c)
        {
            case 'q':
                bExit = true;		
                break;	
            case 'r':
                ResetClient();
                break;	
            case 'p':
                sServerDescription ServerDescription;
                memset(&ServerDescription, 0, sizeof(ServerDescription));
                _pClient->GetServerDescription(&ServerDescription);
                if(!ServerDescription.HostPresent)
                {
                    printf("Unable to connect to server. Host not present. Exiting.");
                    return 1;
                }
                break;
            case 's':
                {
                printf("\n\n[SampleClient] Requesting Data Descriptions...");
                sDataDescriptions* pDataDefs = NULL;
                iResult = _pClient->GetDataDescriptionList(&pDataDefs);
                if (iResult != ErrorCode_OK || pDataDefs == NULL)
                {
                    printf("[SampleClient] Unable to retrieve Data Descriptions.");
                }
                else
                {
                    printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
                }
                }
                break;
            case 'm':	                        // change to multicast
                _connectParams.connectionType = ConnectionType_Multicast;
                iResult = ConnectClient();
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Multicast.\n\n");
                else
                    printf("Error changing client connection type to Multicast.\n\n");
                break;
            case 'u':	                        // change to unicast
                _connectParams.connectionType = ConnectionType_Unicast;
                iResult = ConnectClient();
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Unicast.\n\n");
                else
                    printf("Error changing client connection type to Unicast.\n\n");
                break;
            case 'c' :                          // connect
                iResult = ConnectClient();
                break;
            case 'd' :                          // disconnect
                // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
                iResult = _pClient->SendMessageAndWait("Disconnect", &response, &nBytes);
                if (iResult == ErrorCode_OK)
                    printf("[SampleClient] Disconnected");
                break;
            default:
                break;
        }
        if(bExit)
            break;
    }
    return ErrorCode_OK;
}

void NATNET_CALLCONV MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s\n", msg );
}


// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    // NatNetClient* pClient = (NatNetClient*) pUserData;
    Mocap* mocap = (Mocap*) pUserData;
    NatNetClient* pClient = mocap->GetClient();
    const sServerDescription& serverDescription = mocap->GetServerDescription();

    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(serverDescription.HighResClockFrequency);

    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;

    int i=0;

    printf("FrameID : %d\n", data->iFrame);
    printf("Timestamp : %3.2lf\n", data->fTimestamp);
    printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);

    // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

    if ( bSystemLatencyAvailable )
    {
        // System latency here is defined as the span of time between:
        //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
        // and
        //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(serverDescription.HighResClockFrequency);

        // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
        // This is the all-inclusive measurement (photons to client processing).
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;

        // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
        //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

        printf( "System latency : %.2lf milliseconds\n", systemLatencyMillisec );
        printf( "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec );
    } else {
        printf( "Transit latency : %.2lf milliseconds\n", transitLatencyMillisec );
    }

    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording) {
        printf("RECORDING\n");
    }
    if(bTrackedModelsChanged) {
        printf("Models Changed.\n");
    }
    
    // Rigid Bodies
    printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
    for(i=0; i < data->nRigidBodies; i++) {
        mocap->RunCallbacks(data->RigidBodies[i]);
    }

    //     // params
    //     // 0x01 : bool, rigid body was successfully tracked in this frame
    //     bool bTrackingValid = data->RigidBodies[i].params & 0x01;

    //     printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
    //     printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
    //     printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
    //         data->RigidBodies[i].x,
    //         data->RigidBodies[i].y,
    //         data->RigidBodies[i].z,
    //         data->RigidBodies[i].qx,
    //         data->RigidBodies[i].qy,
    //         data->RigidBodies[i].qz,
    //         data->RigidBodies[i].qw);
    // }

}

}  // namespace rexquad