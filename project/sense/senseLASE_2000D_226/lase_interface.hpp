//============================================================================
// Name        : LASE_Interface.hpp
// Author      : René Middelberg
// Version     : 0.1
// Copyright   : Your copyright notice
// Description : TCP/IP Verbindung SICK Sensor
//============================================================================

#ifndef _LASE_INTERFACE_HPP_
#define _LASE_INTERFACE_HPP_

#include <iostream>
#include <netdb.h>
#include <errno.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <error.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

//using namespace std;

//#define LASE_SERVER_IP_ADDRESS 	"192.168.100.160"
//#define LASE_SERVER_PORT		1024
//#define CLIENT_IP_ADDRESS		"192.168.100.96"
//#define CLIENT_PORT				1025
#define CLIENT_SEND_PORT			1026
#define	ADAPTER_INDEX			2

const uint32_t MAXMEASPOINTS = 1000;           // Maximale Anzhal der Messpunkte

class lase_Connection
{
private:
	int client_portnumber;
	int server_portnumber;
	struct sockaddr_in client_remote_socket_info;
	struct sockaddr_in server_remote_socket_info;
	struct hostent *client_hPtr;
	struct hostent *server_hPtr;
	int client_sockethandle;
	int server_sockethandle;
public:
    lase_Connection(int client_port, const char *client_ip, int server_port, const char *server_ip);
    ~lase_Connection();
    bool connect();
	const int get_client_sockethandle(){return client_sockethandle;}
	const int get_server_sockethandle(){return server_sockethandle;}
    void disconnect();
};

#endif /* _LASE_INTERFACE_HPP_ */
