//============================================================================
// Name        : Sick_Interface.cpp
// Author      : René Middelberg
// Version     : 0.1
// Copyright   : Your copyright notice
// Description : TCP/IP Verbindung SICK Sensor
//============================================================================

#include "sick_interface.hpp"

Sick_Connection::Sick_Connection(int port, const char *ip)
{
	portnumber = port;
	bzero(&remote_socket_info, sizeof(sockaddr_in));  // Clear structure memory
	if((hPtr = gethostbyname(ip)) == NULL)
	{
		std::cerr << "System DNS name resolution not configured properly." << std::endl;
		std::cerr << "Error number: " << ECONNREFUSED << std::endl;
		exit(EXIT_FAILURE);
	}
	sockethandle = 0;
};

Sick_Connection::~Sick_Connection()
{
	Sick_Connection::disconnect_Sick();
}
bool Sick_Connection::connect_Sick()
{
	// create socket
	if((sockethandle = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		close(sockethandle);
		return 0;
	}
	else
		printf("Client-The remote host is: %s\n", hPtr->h_name);


	// Load system information into socket data structures
	memcpy((char *)&remote_socket_info.sin_addr, hPtr->h_addr, hPtr->h_length);
	remote_socket_info.sin_family = AF_INET;
	remote_socket_info.sin_port = htons(portnumber);      // Set port number

	if(connect(sockethandle, (struct sockaddr *)&remote_socket_info, sizeof(sockaddr_in)) < 0)
	{
		printf("Client-The connect() is not OK...\n");
		close(sockethandle);
		return 0;
	}
	else
	{
		printf("Client-The connect() is OK...\n");
		return 1;
	}
}

void Sick_Connection::disconnect_Sick()
{
	printf("Client-Closing sockfd\n");
    close(sockethandle);
}




