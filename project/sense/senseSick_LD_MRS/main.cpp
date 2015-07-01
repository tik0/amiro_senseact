//============================================================================
// Name        : Sick_Main.cpp
// Author      : René Middelberg
// Version     : 0.1
// Copyright   : Your copyright notice
// Description : TCP/IP Verbindung SICK Sensor
//============================================================================

#include "sick_data.hpp"
#include "sick_interface.hpp"

#include <MSG.h>
#include <boost/program_options.hpp>

#include <SensorPosition.pb.h>
#include <GenericInformer.hpp>

int main(int argc, char* argv[])
{
	//Parameters for the sick connection
	std::string tcpServerIP = "192.168.100.150";
	uint16_t tcpServerPort = 12002;
	//std::string clientIP = "192.168.100.152";

	//Scope for sending the data
	std::string sickOutScope = "/Sick";

	std::vector<double> coordinates;

	INFO_MSG("")
	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
	("TcpServerIP,s", po::value <std::string> (&tcpServerIP), "Server ip address, default = 192.168.100.150")
	("TcpServerPort,p", po::value <uint16_t> (&tcpServerPort), "Server port, default = 12002")
	//("ClientIP,c", po::value <std::string> (&clientIP), "Client ip address, default = 192.168.100.151")
	("outscope,o", po::value <std::string> (&sickOutScope), "Scope for sending sick data, default = /Sick")
	("position,q", po::value <std::vector<double>> (&coordinates)->multitoken(), "Position values for the sensor: x y z alpha beta gamma");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
	  std::cout << options << "\n";
	  exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	GenericInformer<rst::sensor::SensorPosition> informerLasePosition(sickOutScope);
	informerLasePosition.getMsgPointer()->set_x(coordinates.at(0));
	informerLasePosition.getMsgPointer()->set_y(coordinates.at(1));
	informerLasePosition.getMsgPointer()->set_z(coordinates.at(2));
	informerLasePosition.getMsgPointer()->set_alpha(coordinates.at(3));
	informerLasePosition.getMsgPointer()->set_beta(coordinates.at(4));
	informerLasePosition.getMsgPointer()->set_gamma(coordinates.at(5));
	informerLasePosition.getMsgPointer()->set_uniqueidentifier(tcpServerIP);
	informerLasePosition.publishMsg();

	//set uniqueIdentifier for every sick scanner, which is the IP address
	sickLDMRS4002->set_uniqueidentifier(tcpServerIP);

	//Create an informer that is capable of sending events
	rsb::Informer<SickLdMRS400102>::Ptr informer;

	// Init RSB
	// First get a factory instance that is used to create RSB domain objects
	rsb::Factory& factory = rsb::getFactory();

	// Register new converter
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<SickLdMRS400102>> converter(new rsb::converter::ProtocolBufferConverter<SickLdMRS400102>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	// Prepare informer
	informer = factory.createInformer<SickLdMRS400102>(sickOutScope);

	char buffer[MAXBUFFER];
	int datalength;
	bool Flag;

    Sick_Connection Sick1(tcpServerPort, tcpServerIP.c_str());
    while(Flag == 0)
    Flag = Sick1.connect_Sick();

    while(Flag == 1)
    {
    	datalength = recv(Sick1.get_sockethandle(), buffer, (MAXBUFFER), 0);
    	if(global==0)
		onTcpReceive(buffer, datalength, informer);
    }
    return 0;
}
