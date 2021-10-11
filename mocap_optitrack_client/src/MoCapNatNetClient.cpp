// This class implements a NatNet client
#include <cstdio>
#include "MoCapNatNetClient.h"
#include <vector>

using namespace std;

// MoCapNatNetClient constructor
MoCapNatNetClient::MoCapNatNetClient(MoCapPublisher* _moCapPublisher)
{
    // Save the ROS2 publisher to send messages to this
    this->moCapPublisher = _moCapPublisher;
    
    // Create the connection parameters, retreiving the data from the ROS2 node
    if (moCapPublisher->getConnectionType() == 0) g_connectParams.connectionType = ConnectionType_Multicast;
    else g_connectParams.connectionType = ConnectionType_Unicast;

    g_connectParams.serverDataPort = moCapPublisher->getServerDataPort();
    g_connectParams.serverCommandPort = moCapPublisher->getServerCommandPort();

    std::string serverAddress_s = moCapPublisher->getServerAddress();
    char* serverAddress = (char*) malloc(serverAddress_s.length()*sizeof(char));
    strcpy(serverAddress, serverAddress_s.c_str());
    g_connectParams.serverAddress = (const char*) serverAddress;

    std::string multiCastAddress_s = moCapPublisher->getMulticastAddress();
    char* multiCastAddress = (char*) malloc(multiCastAddress_s.length()*sizeof(char));
    strcpy(multiCastAddress, multiCastAddress_s.c_str());
    g_connectParams.multicastAddress = (const char*) multiCastAddress;
    
    // Struct where the data description will be saved
    pDataDefs = NULL;

    // Set the callback for the frames received by the server
    this->SetFrameReceivedCallback( dataFrameHandler, this );
}

// Distructor
MoCapNatNetClient::~MoCapNatNetClient()
{
    if ( this->pDataDefs )
    {
        NatNet_FreeDescriptions( this->pDataDefs );
        this->pDataDefs = NULL; 
    }

    this->moCapPublisher = NULL;//this is deleted outside
}

// Method that starts the connection with the server
int MoCapNatNetClient::connect()
{
    int retCode;
    void* pResult;
    int nBytes = 0;
    ErrorCode ret = ErrorCode_OK;
    //
    // Release previous server
    this->Disconnect();
    //
    // Print version info
    unsigned char ver[4];
    NatNet_GetVersion(ver);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);
    //
    // Connect the client to the server
    retCode = this->Connect( g_connectParams );
    //
    memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
    //
    ret = this->GetServerDescription( &g_serverDescription );
    if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
    {
        RCLCPP_ERROR(this->moCapPublisher->get_logger(), "Unable to connect, exiting.\n");
        return 1;
    }
    //Log some information about the server
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Server application info:\n");
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, 
                                                                                            g_serverDescription.HostAppVersion[0],
                                                                                            g_serverDescription.HostAppVersion[1],
                                                                                            g_serverDescription.HostAppVersion[2],
                                                                                            g_serverDescription.HostAppVersion[3]);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "NatNet Version: %d.%d.%d.%d\n",g_serverDescription.NatNetVersion[0], 
                                                                                    g_serverDescription.NatNetVersion[1],
                                                                                    g_serverDescription.NatNetVersion[2], 
                                                                                    g_serverDescription.NatNetVersion[3]);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Server IP:%s\n", g_connectParams.serverAddress);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Server Name:%s\n\n", g_serverDescription.szHostComputerName);
    //
    // Get mocap frame rate
    ret = this->SendMessageAndWait("FrameRate", &pResult, &nBytes);
    if (ret == ErrorCode_OK)
    {
        float fRate = *((float*)pResult);
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "Mocap Framerate : %3.2f\n", fRate);
    }
    else
    {
        RCLCPP_ERROR(this->moCapPublisher->get_logger(), "Can not get the frame rate.\n");
    }
    // Get # of analog samples per mocap frame of data
    ret = this->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
    if (ret == ErrorCode_OK)
    {
        int g_analogSamplesPerMocapFrame = *((int*)pResult);
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
    }
    else
    {
        RCLCPP_ERROR(this->moCapPublisher->get_logger(), "Error getting Analog frame rate.\n");
    }
    //
    // Get all the objects from the server
    getDataDescription();
    //
    // Return the code
    return retCode;
}

/*Disconnect the client*/
void MoCapNatNetClient::disconnect()
{
    // Disconnect the client
    this->Disconnect();
}


sServerDescription MoCapNatNetClient::getServerDescription()
{
    return this->g_serverDescription;
}

int MoCapNatNetClient::getAnalogSamplesPerMocapFrame()
{
    return this->g_analogSamplesPerMocapFrame;
}

double MoCapNatNetClient::SecondsSinceHostTimestamp( uint64_t hostTimestamp )
{
   return NatNetClient::SecondsSinceHostTimestamp(hostTimestamp);
}

MoCapPublisher* MoCapNatNetClient::getPublisher()
{
    return this->moCapPublisher;
}

// Callback for the data frames streamed by the server
void NATNET_CALLCONV dataFrameHandler(sFrameOfMocapData* data, void* pUserData)
{
    MoCapNatNetClient* pClient = (MoCapNatNetClient*) pUserData;
    //
    int g_analogSamplesPerMocapFrame = pClient->getAnalogSamplesPerMocapFrame();
    sServerDescription g_serverDescription = pClient->getServerDescription();
    //
    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);
    //
    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;
    //
    int i=0;
    //
    RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "FrameID : %d\n", data->iFrame);
    RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Timestamp : %3.2lf\n", data->fTimestamp);
    RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);
    //
    // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;
    //
    if ( bSystemLatencyAvailable )
    {
        // System latency here is defined as the span of time between:
        //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
        // and
        //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

        // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
        // This is the all-inclusive measurement (photons to client processing).
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;

        // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
        //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

        RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "System latency : %.2lf milliseconds\n", systemLatencyMillisec );
        RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec );
    }
    else
    {
        RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Transit latency : %.2lf milliseconds\n", transitLatencyMillisec );
    }
    //
    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording)
        RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "RECORDING\n");
    if(bTrackedModelsChanged)
        RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Models Changed.\n");
	//
    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode( data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe );
	// decode to friendly string
	char szTimecode[128] = "";
    NatNet_TimecodeStringify( data->Timecode, data->TimecodeSubframe, szTimecode, 128 );
	RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Timecode : %s\n", szTimecode);
    //
	// Rigid Bodies
    pClient->sendRigidBodyMessage(data->RigidBodies, data->nRigidBodies);
    //
    //NOTE : from below is just logging...
	// Skeletons
	RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Skeletons [Count=%d]\n", data->nSkeletons);
	for(i=0; i < data->nSkeletons; i++)
	{
		sSkeletonData skData = data->Skeletons[i];
		RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
		for(int j=0; j< skData.nRigidBodies; j++)
		{
			sRigidBodyData rbData = skData.RigidBodyData[j];
			RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
				rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw );
		}
	}
    //
	// labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
    //
    //
	RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Markers [Count=%d]\n", data->nLabeledMarkers);
	for(i=0; i < data->nLabeledMarkers; i++)
	{
        //Commented below some additional boolean variables about the markers
        //bool bOccluded = ((data->LabeledMarkers[i].params & 0x01)!=0);// marker was not visible (occluded) in this frame
        //bool bPCSolved = ((data->LabeledMarkers[i].params & 0x02)!=0);// reported position provided by point cloud solve
        //bool bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);// reported position provided by model solve
        //bool bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);// marker has an associated asset in the data stream
        bool bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);// marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
		bool bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);// marker is an actively labeled LED marker
        //
        sMarker marker = data->LabeledMarkers[i];
        //
        // Marker ID Scheme:
        // Active Markers:
        //   ID = ActiveID, correlates to RB ActiveLabels list
        // Passive Markers: 
        //   If Asset with Legacy Labels
        //      AssetID 	(Hi Word)
        //      MemberID	(Lo Word)
        //   Else
        //      PointCloud ID
        int modelID, markerID;
        NatNet_DecodeID( marker.ID, &modelID, &markerID );
		//
        char szMarkerType[512];
        if (bActiveMarker)
            strcpy(szMarkerType, "Active");
        else if(bUnlabeled)
            strcpy(szMarkerType, "Unlabeled");
        else
            strcpy(szMarkerType, "Labeled");
        //
        RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
            szMarkerType, modelID, markerID, marker.size, marker.x, marker.y, marker.z);
	}
    //
    // force plates
    RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Force Plate [Count=%d]\n", data->nForcePlates);
    for(int iPlate=0; iPlate < data->nForcePlates; iPlate++)
    {
        RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Force Plate %d\n", data->ForcePlates[iPlate].ID);
        for(int iChannel=0; iChannel < data->ForcePlates[iPlate].nChannels; iChannel++)
        {
            RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "\tChannel %d:\t", iChannel);
            if(data->ForcePlates[iPlate].ChannelData[iChannel].nFrames == 0)
            {
                RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "\tEmpty Frame\n");
            }
            else if(data->ForcePlates[iPlate].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
            {
                RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->ForcePlates[iPlate].ChannelData[iChannel].nFrames);
            }
            for(int iSample=0; iSample < data->ForcePlates[iPlate].ChannelData[iChannel].nFrames; iSample++)
                RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "%3.2f\t", data->ForcePlates[iPlate].ChannelData[iChannel].Values[iSample]);
            RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "\n");
        }
    }
    //
    // devices
    RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Device [Count=%d]\n", data->nDevices);
    for (int iDevice = 0; iDevice < data->nDevices; iDevice++)
    {
        RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "Device %d\n", data->Devices[iDevice].ID);
        for (int iChannel = 0; iChannel < data->Devices[iDevice].nChannels; iChannel++)
        {
            RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "\tChannel %d:\t", iChannel);
            if (data->Devices[iDevice].ChannelData[iChannel].nFrames == 0)
            {
                RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "\tEmpty Frame\n");
            }
            else if (data->Devices[iDevice].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
            {
                RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->Devices[iDevice].ChannelData[iChannel].nFrames);
            }
            for (int iSample = 0; iSample < data->Devices[iDevice].ChannelData[iChannel].nFrames; iSample++)
                RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "%3.2f\t", data->Devices[iDevice].ChannelData[iChannel].Values[iSample]);
            RCLCPP_DEBUG(pClient->getPublisher()->get_logger(), "\n");
        }
    }
}

// Method responsible of forwarding messages of rigid body data to the ROS2 publisher
void MoCapNatNetClient::sendRigidBodyMessage(sRigidBodyData* bodies, int nRigidBodies)
{
    this->moCapPublisher->sendRigidBodyMessage(bodies, nRigidBodies);
}


/*Method that gets the data description from the server*/
void MoCapNatNetClient::getDataDescription()
{
    // TODO : throw an execption if it was not possible to read the data
    int iResult;
    //
    // Request the data description
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "\n\nRequesting Data Descriptions...\n");
    iResult = this->GetDataDescriptionList(&this->pDataDefs);
	if (iResult != ErrorCode_OK || this->pDataDefs == NULL)
	{
		RCLCPP_ERROR(this->moCapPublisher->get_logger(), "Unable to retrieve Data Descriptions.\n");
	}
	else
	{
        //Print the data description
        processDataDescription(this->pDataDefs);
    }
}

// Method that processes the data description received from the server
void MoCapNatNetClient::processDataDescription(sDataDescriptions* pDataDefs)
{
    // Process the data description
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
    for(int i=0; i < pDataDefs->nDataDescriptions; i++)
    {
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
        switch (pDataDefs->arrDataDescriptions[i].type)
        {
        case Descriptor_MarkerSet:
            // MarkerSet
            processMarkerSet(pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription);
            break;
        case Descriptor_RigidBody:
            // Rigid body
            processRigidBody(pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription);
            break;
        case Descriptor_Skeleton:
            // Skeleton
            processSkeleton(pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription);
            break;
        case Descriptor_ForcePlate:
            // Force Plate
            processForcePlate(pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription);
            break;
        case Descriptor_Device:
            // Peripheral Device
            processPeripheralDevice(pDataDefs->arrDataDescriptions[i].Data.DeviceDescription);
            break;
        case Descriptor_Camera:
            // Camera
            processCamera(pDataDefs->arrDataDescriptions[i].Data.CameraDescription);
            break;
        default:
            // Unkown data type
            RCLCPP_WARN(this->moCapPublisher->get_logger(), "Unknown data type.\n");
            break;
        }
    }
}

// Method that processes a MarkerSet descriptor
void MoCapNatNetClient::processMarkerSet(sMarkerSetDescription* pMS)
{
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "MarkerSet Name : %s\n", pMS->szName);
    for(int i=0; i < pMS->nMarkers; i++)
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "%s\n", pMS->szMarkerNames[i]);
}

// Method that processes a RigidBody descriptor
void MoCapNatNetClient::processRigidBody(sRigidBodyDescription* pRB)
{
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "RigidBody Name : %s\n", pRB->szName);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "RigidBody ID : %d\n", pRB->ID);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "RigidBody Parent ID : %d\n", pRB->parentID);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

    if ( pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL )
    {
        for ( int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx )
        {
            const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
            const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];
            //
            RCLCPP_INFO(this->moCapPublisher->get_logger(), "\tMarker #%d:\n", markerIdx );
            RCLCPP_INFO(this->moCapPublisher->get_logger(), "\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2] );
            //
            if ( markerRequiredLabel != 0 )
            {
                RCLCPP_INFO(this->moCapPublisher->get_logger(), "\t\tRequired active label: %d\n", markerRequiredLabel );
            }
        }
    }
}

// Method that processes a Skeleton descriptor
void MoCapNatNetClient::processSkeleton(sSkeletonDescription* pSK)
{
    // Skeleton
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Skeleton Name : %s\n", pSK->szName);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Skeleton ID : %d\n", pSK->skeletonID);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
    for(int j=0; j < pSK->nRigidBodies; j++)
    {
        sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "  RigidBody Name : %s\n", pRB->szName);
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "  RigidBody ID : %d\n", pRB->ID);
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "  RigidBody Parent ID : %d\n", pRB->parentID);
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
    }
}

//Method that processes a ForcePlate descriptor
void MoCapNatNetClient::processForcePlate(sForcePlateDescription* pFP)
{
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate ID : %d\n", pFP->ID);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate Serial : %s\n", pFP->strSerialNo);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate Width : %3.2f\n", pFP->fWidth);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate Length : %3.2f\n", pFP->fLength);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX,pFP->fOriginY, pFP->fOriginZ);
    for(int iCorner=0; iCorner<4; iCorner++)
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0],pFP->fCorners[iCorner][1],pFP->fCorners[iCorner][2]);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate Type : %d\n", pFP->iPlateType);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate Data Type : %d\n", pFP->iChannelDataType);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Force Plate Channel Count : %d\n", pFP->nChannels);
    for(int iChannel=0; iChannel<pFP->nChannels; iChannel++)
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
}

//Method that processes a Peripheral device descriptor
void MoCapNatNetClient::processPeripheralDevice(sDeviceDescription* pDevice)
{
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Device Name : %s\n", pDevice->strName);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Device Serial : %s\n", pDevice->strSerialNo);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Device ID : %d\n", pDevice->ID);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Device Channel Count : %d\n", pDevice->nChannels);
    for (int iChannel = 0; iChannel < pDevice->nChannels; iChannel++)
        RCLCPP_INFO(this->moCapPublisher->get_logger(), "\tChannel %d : %s\n", iChannel, pDevice->szChannelNames[iChannel]);
}

//Method that processes a Camera device descriptor
void MoCapNatNetClient::processCamera(sCameraDescription* pCamera)
{
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Camera Name : %s\n", pCamera->strName);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Camera Position (%3.2f, %3.2f, %3.2f)\n", pCamera->x, pCamera->y, pCamera->z);
    RCLCPP_INFO(this->moCapPublisher->get_logger(), "Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)\n", pCamera->qx, pCamera->qy, pCamera->qz, pCamera->qw);
}
