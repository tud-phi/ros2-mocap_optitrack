/* Configuration header*/
#ifndef MYCONFIG_H
#define MYCONFIG_H

/*Address of the NatNet server*/
// Set to "host.docker.internal" if the node runs in a docker container on the same machine of the NatNet server.
// If, in addition, the container is hosted on a linux machine be sure to run the docker
// container with the option : --add-host host.docker.internal:host-gateway.
#define SERVER_ADDRESS "10.125.37.2"
/*Multicast address of the server*/
#define MULTI_CAST_ADDRESS "239.255.42.99"
/*Type of the connection to the server*/
// Type of the connection : 0 = multicast, 1 = unicast, see the NatNetSDK doc for more details.
#define SERVER_CONNECTION_TYPE 0
/*PORT WHERE READ DATA*/
#define SERVER_DATA_PORT 1511
/*PORT WHERE SEND COMMANDS*/
#define SERVER_COMMAND_PORT 1510

#endif