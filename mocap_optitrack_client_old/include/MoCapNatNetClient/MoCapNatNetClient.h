#ifndef MOCAPNATNETCLIENT_H
#define MOCAPNATNETCLIENT_H

#include <string>
#include <iostream>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <MoCapPublisher.h>


using namespace std;

// Handler to receive the data from the server
void NATNET_CALLCONV dataFrameHandler(sFrameOfMocapData* data, void* pUserData);


class MoCapNatNetClient:private NatNetClient
{
// Private attributes and methods
private:
    sNatNetClientConnectParams g_connectParams;
    sServerDescription g_serverDescription;
    sDataDescriptions* pDataDefs;
    int g_analogSamplesPerMocapFrame = 0;
    MoCapPublisher* moCapPublisher;
    //const char* server_address;
    //const char* multicast_address;
    
    
    // Method to get the data description from the server
    void getDataDescription();
    void processDataDescription(sDataDescriptions* pDataDefs);
    void processMarkerSet(sMarkerSetDescription* pMS);
    void processRigidBody(sRigidBodyDescription* pRB);
    void processSkeleton(sSkeletonDescription* pSK);
    void processForcePlate(sForcePlateDescription* pFP);
    void processPeripheralDevice(sDeviceDescription* pDevice);
    void processCamera(sCameraDescription* pCamera);


// Public attributes and methods
public:
    // Definition of the construtors
    MoCapNatNetClient(MoCapPublisher* moCapPublisher);
    ~MoCapNatNetClient();


    // Method to connect the client with the server
    int connect();

    // Method to disconnect the client from the server
    void disconnect();

    double SecondsSinceHostTimestamp( uint64_t hostTimestamp );

    // Getters
    sServerDescription getServerDescription();
    int getAnalogSamplesPerMocapFrame();
    MoCapPublisher* getPublisher();//returns the Ros2 publisher
    // Setters

    //Methods to forward messages to the ROS2 system
    void sendRigidBodyMessage(sRigidBodyData* bodies, int nRigidBodies);

    
};
 
#endif