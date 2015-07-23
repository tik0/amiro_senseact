//============================================================================
// Name        : Sick_Main.cpp
// Author      : René Middelberg
// Version     : 0.1
// Copyright   : Your copyright notice
// Description : TCP/IP Verbindung SICK Sensor
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include "lase_interface.hpp"
#include "lase_data.hpp"
#include <iostream>
#include <memory>
#include <string.h>

#include <unistd.h>

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

// RST"../../includes"
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <lase_2000D_226.pb.h>

#define BUFLEN 8192
#define MAXMEASPOINTS 1000

//LASE_2000D_226
boost::shared_ptr<rst::claas::LASE_2000D_226> lase_2000D_226 = boost::make_shared<rst::claas::LASE_2000D_226>();

uint32_t getparameter(char buffer[], uint16_t offset)
{
  uint32_t parameter = 0;
  parameter = (buffer[offset] & 0x00FF) << 24 | (buffer[offset+1]& 0x00FF) << 16 | (buffer[offset+2]& 0x00FF) << 8 | (buffer[offset+3] & 0x00FF);
  return parameter;
}

void setScanData (char buffer[], int laenge, rsb::Informer<rst::claas::LASE_2000D_226>::Ptr informer)
{
	/****************************** Header auslesen *******************************/
	// Maximale Anzhal der Messpunkte
	int64_t distance[MAXMEASPOINTS];
	int64_t pulsewidth[MAXMEASPOINTS];
	uint16_t offset = 12;
	//std::system("clear"); // clear display
	//std::cout << buffer[0] <<  buffer[1]  <<  buffer[2] <<  buffer[3]<< "	" ;

	uint32_t scannumber = getparameter(buffer, offset);
	lase_2000D_226->set_scannumber(scannumber);
	offset = 16;
	uint32_t timestamp = getparameter(buffer, offset);
	lase_2000D_226->set_timestamp(timestamp);
	offset = 36;
	uint32_t systemtemperature = getparameter(buffer, offset);
	lase_2000D_226->set_systemtemperature(systemtemperature);
	//std::cout << "Scannumber: " <<  scannumber << "	" << "Timestamb: " << timestamp << "	" << "Temperature: " << systemtemperature << "\n";
	offset = 52;
	int64_t tmp;
	for(uint16_t j = 0; j <= MAXMEASPOINTS-1; j++)
	{
		distance[j] = (buffer[offset+(j*8)] & 0x00FF) << 24 | (buffer[offset+1+(j*8)]& 0x00FF) << 16 | (buffer[offset+2+(j*8)]& 0x00FF) << 8 | (buffer[offset+3+(j*8)] & 0x00FF);
		memcpy(&tmp, &distance[j],8);
		lase_2000D_226->add_distance(tmp);
		pulsewidth[j] = (buffer[offset+4+(j*8)] & 0x00FF) << 24 | (buffer[offset+1+4+(j*8)]& 0x00FF) << 16 | (buffer[offset+2+4+(j*8)]& 0x00FF) << 8 | (buffer[offset+3+4+(j*8)] & 0x00FF);
		memcpy(&tmp, &pulsewidth[j],8);
		lase_2000D_226->add_pulsewidth(tmp);
	}
	//std::cout << std::endl;
	informer->publish(lase_2000D_226);
	//Clear the scanned points after a whole data set was transmitted
	lase_2000D_226->clear_distance();
	lase_2000D_226->clear_pulsewidth();
}

int main(int argc, char* argv[])
{
	//Parameters for the lase connection
	std::string clientIP = "192.168.100.96";
	uint16_t clientPort = 1025;
	std::string laseServerIP = "192.168.100.160";
	uint16_t laseServerPort = 1024;

	//Scope for sending the data
	std::string laseOutScope = "/sense/LASE2000D/1";

	std::vector<double> coordinates{-2822.938, 2080.804, 0, 0, 0, 29.742};

	INFO_MSG("")
	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
	("c_CLIENT_IP_ADDRESS,c", po::value <std::string> (&clientIP), "Client ip address, default = 192.168.100.96")
	("p_CLIENT_PORT,p", po::value <uint16_t> (&clientPort), "Client port, default = 1025")
	("s_LASE_SERVER_IP_ADDRESS,s", po::value <std::string> (&laseServerIP), "Lase server ip address, default = 192.168.100.160")
	("t_LASE_SERVER_PORT,t", po::value <uint16_t> (&laseServerPort), "Lase server port, default = 1024")
	("outscope,o", po::value <std::string> (&laseOutScope), "Scope for sending lase data, default = /sense/LASE2000D/1")
	("q_position,q", po::value <std::vector<double>> (&coordinates)->multitoken(), "Sensor Poistion: x y z alpha beta gamma, default = -2822.938, 2080.804, 0, 0, 0, 29.742");

	// allow to give the value as a positional argument
	//po::positional_options_description p;
	//p.add("value", 1);

	po::variables_map vm;
	po::store( po::command_line_parser(argc, argv).options(options).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
	  std::cout << options << "\n";
	  exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	//set uniqueIdentifier for the scanner, which is the IP address and position values for the sensor
	lase_2000D_226->set_uniqueidentifier(laseServerIP);
	lase_2000D_226->mutable_pose()->mutable_translation()->set_x(coordinates.at(0));
	lase_2000D_226->mutable_pose()->mutable_translation()->set_y(coordinates.at(1));
	lase_2000D_226->mutable_pose()->mutable_translation()->set_z(coordinates.at(2));
	lase_2000D_226->mutable_pose()->mutable_rotation()->set_roll(coordinates.at(3));
	lase_2000D_226->mutable_pose()->mutable_rotation()->set_pitch(coordinates.at(4));
	lase_2000D_226->mutable_pose()->mutable_rotation()->set_yaw(coordinates.at(5));

	struct hostent *client_hPtr;
	struct hostent *server_hPtr;
	struct sockaddr_in client_remote_socket_info;
	struct sockaddr_in server_remote_socket_info;
	int client_sockethandle;
	unsigned slen = sizeof(server_remote_socket_info);
	unsigned clen = sizeof(client_remote_socket_info);
	char buf[BUFLEN];
	uint32_t framesize16 = 16;
	uint32_t framesize12 = 12;
	char databuffergetversion[12] = "GVER" ;  // Requesting the Firmware Version
	char databufferstartscan1[16] = "SCAN";
	char databuffergetscan[16] = "GSCN";      // Requesting one scan measurement

	databuffergetversion[8] = (char) 0x1b;    //CRC
	databuffergetversion[9] = (char) 0x39;    //CRC
	databuffergetversion[10] = (char) 0xcf;   //CRC
	databuffergetversion[11] = (char) 0x64;   //CRC

	databufferstartscan1[7] = (char) 0x04;    // Datalength
	databufferstartscan1[11] = (char) 0x01;
	databufferstartscan1[12] = (char) 0x9a;
	databufferstartscan1[13] = (char) 0x35;
	databufferstartscan1[14] = (char) 0x13;
	databufferstartscan1[15] = (char) 0x20;

	databuffergetscan[7] = (char) 0x04;       // Datalength
	databuffergetscan[12] = (char) 0x48;
	databuffergetscan[13] = (char) 0x2f;
	databuffergetscan[14] = (char) 0xe1;
	databuffergetscan[15] = (char) 0xc3;

	//Create an informer that is capable of sending events
	rsb::Informer<rst::claas::LASE_2000D_226>::Ptr informer;

	// Init RSB
	// First get a factory instance that is used to create RSB domain objects
	rsb::Factory& factory = rsb::getFactory();

	// Register new converter
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::LASE_2000D_226>> converter(new rsb::converter::ProtocolBufferConverter<rst::claas::LASE_2000D_226>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	// Prepare informer
	informer = factory.createInformer<rst::claas::LASE_2000D_226>(laseOutScope);

	if ((client_sockethandle = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket");
		return 0;
	}

	/* bind to an arbitrary return address */
	/* because this is the client side, we don't care about the address */
	/* since no application will initiate communication here - it will */
	/* just send responses */
	/* INADDR_ANY is the IP address and 0 is the socket */
	/* htonl converts a long integer (e.g. address) to a network representation */
	/* htons converts a short integer (e.g. port) to a network representation */

	// Client structure
	bzero(&client_remote_socket_info, sizeof(sockaddr_in));
	if((client_hPtr = gethostbyname(clientIP.c_str())) == NULL) {
		std::cerr << "System DNS name resolution not configured properly." << std::endl;
		std::cerr << "Error number: " << ECONNREFUSED << std::endl;
		exit(EXIT_FAILURE);
	}
	memcpy((char *)&client_remote_socket_info.sin_addr, client_hPtr->h_addr, client_hPtr->h_length);
	client_remote_socket_info.sin_family = AF_INET;
	client_remote_socket_info.sin_port = htons(clientPort);

	// bind the clien socket to 1024
	if(bind(client_sockethandle, (struct sockaddr *)&client_remote_socket_info, sizeof(client_remote_socket_info)) < 0) {
		perror("bind failed");
		return 0;
	}
	if (getsockname(client_sockethandle, (struct sockaddr *)&client_remote_socket_info, &clen) < 0) {
		perror("getsockname failed");
		return 0;
	}
	std::cout << "bind complete. Port number = " << ntohs(client_remote_socket_info.sin_port) << "\n";

	// Server structure
	bzero(&server_remote_socket_info, sizeof(sockaddr_in));
	if((server_hPtr = gethostbyname(laseServerIP.c_str())) == NULL) {
		std::cerr << "System DNS name resolution not configured properly." << std::endl;
		std::cerr << "Error number: " << ECONNREFUSED << std::endl;
		exit(EXIT_FAILURE);
	}

	memcpy((char *)&server_remote_socket_info.sin_addr, server_hPtr->h_addr, server_hPtr->h_length);
	server_remote_socket_info.sin_family = AF_INET;
	server_remote_socket_info.sin_port = htons(laseServerPort);

	if (sendto(client_sockethandle, databuffergetversion, framesize12, 0, (struct sockaddr *)&server_remote_socket_info, slen)==-1) {
			perror("sendto");
			exit(1);
	}

	int i = 0;
	for (;;) {
		if(recvfrom(client_sockethandle, buf, BUFLEN, 0, (struct sockaddr *)&client_remote_socket_info, &clen)==-1) {
			perror("recvfrom");
			exit(1);
		}
		// Laenge auswerten
		uint32_t length = getparameter(buf, 4);
		if (length>=1000)
		{
			setScanData(buf, length, informer);
		}
		else {
			//std::cout << buf[0] <<  buf[1]  <<  buf[2] <<  buf[3]<< "\n";
		}

		if (i == 0)
		{
			if(sendto(client_sockethandle, databufferstartscan1, framesize16, 0, (struct sockaddr *)&server_remote_socket_info, slen)==-1) {
				perror("sendto");
				exit(1);
			}
			i++;
		}
		else if(i < 6)
		{
			if(sendto(client_sockethandle, databuffergetversion, framesize12, 0, (struct sockaddr *)&server_remote_socket_info, slen)==-1) {
				perror("sendto");
				exit(1);
			}
			i++;
		}
		else if (i < 8)
		{
			if(sendto(client_sockethandle, databufferstartscan1, framesize16, 0, (struct sockaddr *)&server_remote_socket_info, slen)==-1) {
				perror("sendto");
				exit(1);
		}
			i++;
		}
		else
			sendto(client_sockethandle, databuffergetscan, framesize16, 0, (struct sockaddr *)&server_remote_socket_info, slen);
	}
	close(client_sockethandle);
	return 0;

}
