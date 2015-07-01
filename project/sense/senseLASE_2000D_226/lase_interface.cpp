//============================================================================
// Name        : Sick_Interface.cpp
// Author      : René Middelberg
// Version     : 0.1
// Copyright   : Your copyright notice
// Description : TCP/IP Verbindung SICK Sensor
//============================================================================

#include "lase_interface.hpp"
#include <iostream>
#include <string>

#include <stdlib.h>
#include <memory>

lase_Connection::lase_Connection(int client_port, const char *client_ip, int server_port, const char *server_ip)
{
	client_portnumber = client_port;
	server_portnumber = server_port;
	bzero(&client_remote_socket_info, sizeof(sockaddr_in));  // Clear structure memory
	if((client_hPtr = gethostbyname(client_ip)) == NULL)
	{
		std::cerr << "System DNS name resolution not configured properly." << std::endl;
		std::cerr << "Error number: " << ECONNREFUSED << std::endl;
		exit(EXIT_FAILURE);
	}
	client_sockethandle = 0;
};

lase_Connection::~lase_Connection()
{
    lase_Connection::disconnect();
}
bool lase_Connection::connect()
{
	// clear terminal
    //std::system("clear");
	// create socket
	if((client_sockethandle = socket(AF_INET, SOCK_DGRAM , 0)) < 0)
	{
		close(client_sockethandle);
		return 0;
	}
	else
		std::cout << "Client-The remote host is: " << client_hPtr->h_name << "\n";


	// Load system information into socket data structures
	memcpy((char *)&client_remote_socket_info.sin_addr, client_hPtr->h_addr, client_hPtr->h_length);
	client_remote_socket_info.sin_family = AF_INET;
	client_remote_socket_info.sin_port = htons(client_portnumber);      // Set port number

	if(bind(client_sockethandle, (struct sockaddr *)&client_remote_socket_info, sizeof(sockaddr_in)) < 0)
	{
		std::cout << "Client-The connect() is not OK...\n";
		close(client_sockethandle);
		return 0;
	}
	else
	{
		std::cout << "Client-The connect() is OK...\n";
		return 1;
	}



	  /*sendinfo = UdpSendTo(gTcpDataSocket,serverIp,serverport,databuffergetversion,framesize12);

	  if (0 == sendinfo)
	  {
	    writelineex(1, 1, "Successfully send to server");
	  }
	  else
	  {
	    writelineex(1, 1, "not completed successfully");
	    IpGetLastSocketErrorAsString( gTcpDataSocket, gIpLastErrStr, elcount( gIpLastErrStr));
	    writeTextBkgColor(1, 255, 0, 0);
	    writelineex( 1, 2, "OnTcpReceive error (%d): %s", IpGetLastSocketError( gTcpDataSocket), gIpLastErrStr);
	  }*/
}

void lase_Connection::disconnect()
{
	std::cout << "Client-Closing sockfd\n";
    close(client_sockethandle);
}




