//============================================================================
// Name        : Sick_Interface.hpp
// Author      : René Middelberg
// Version     : 0.1
// Copyright   : Your copyright notice
// Description : TCP/IP Verbindung SICK Sensor
//============================================================================

#ifndef SICK_INTERFACE_HPP_
#define SICK_INTERFACE_HPP_

#include <iostream>
#include <netdb.h>
#include <errno.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <error.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

class Sick_Connection
{
private:
	int portnumber;
	struct sockaddr_in remote_socket_info;
	struct hostent *hPtr;
	int sockethandle;
public:
	Sick_Connection(int port, const char *ip);
	~Sick_Connection();
	bool connect_Sick();
	const int get_sockethandle(){return sockethandle;}
	void disconnect_Sick();
};

#endif /* SICK_INTERFACE_HPP_ */
