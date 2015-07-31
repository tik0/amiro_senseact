//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the distance information of DS_ODSL96B_M_TOF_S12
//               and send it via RSB
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

//#include <boost/thread.hpp>
#include <boost/program_options.hpp>
//#include <boost/shared_ptr.hpp>

#include <thread>
#include <chrono>
#include <iostream>

#include "IbeoScalaDataListener.hpp"

void getScalaValues(ibeosdk::LogFileManager& logFileManager, std::string& ip, std::string& scalaScope)
{
	IbeoScalaDataListener ibeoScalaDataListener(ip, scalaScope);

	const uint16_t port = ibeosdk::getPort(ip, 12004);
	ibeosdk::IbeoScala scala(ip, port);
	scala.setLogFileManager(&logFileManager);

//	scala.registerListener(dynamic_cast<ibeosdk::DataListener<ibeosdk::FrameEndSeparator>*>(&ibeoScalaDataListener));
//	scala.registerListener(dynamic_cast<DataListener<ScanLux>*>(&allScalaListener));
	scala.registerListener(dynamic_cast<ibeosdk::DataListener<ibeosdk::Scan2208>*>(&ibeoScalaDataListener));
//	scala.registerListener(dynamic_cast<DataListener<ObjectListLux>*>(&allScalaListener));
//	scala.registerListener(dynamic_cast<DataListener<ObjectListScala>*>(&allScalaListener));
	scala.registerListener(dynamic_cast<ibeosdk::DataListener<ibeosdk::ObjectListScala2271>*>(&ibeoScalaDataListener));
//	scala.registerListener(dynamic_cast<DataListener<VehicleStateBasicLux>*>(&allScalaListener));
	scala.registerListener(dynamic_cast<ibeosdk::DataListener<ibeosdk::DeviceStatus>*>(&ibeoScalaDataListener));
//	scala.registerListener(dynamic_cast<DataListener<LogMessageError>*>(&allScalaListener));
//	scala.registerListener(dynamic_cast<DataListener<LogMessageDebug>*>(&allScalaListener));
//	scala.registerListener(dynamic_cast<DataListener<LogMessageNote>*>(&allScalaListener));
//	scala.registerListener(dynamic_cast<DataListener<LogMessageWarning>*>(&allScalaListener));

	scala.getConnected();

	// Just to keep the program alive
	while (true) {
		if (!scala.isConnected()){
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}

int main(int argc, char *argv[]) {

	std::vector<double> coordinates{-2645.269, 1868.837, -75, 0, 0, 13.658};
	std::string scalaIP = "192.168.100.180";
	uint16_t tcpServerPort = 12002;
	std::string rsbOutScope = "/sense/IBEOScala/1";

	INFO_MSG("")
	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
	("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending ibeo scala 1403 values")
	("s_Scala_IP,s", po::value <std::string> (&scalaIP), "Scala ip address, default = 192.168.100.180")
	("p_TcpServerPort,p", po::value <uint16_t> (&tcpServerPort), "Server port, default = 12002")
	("q_position,q", po::value <std::vector<double> > (&coordinates)->multitoken(), "Sensor Poistion: x y z alpha beta gamma, default = -2645.269, 1868.837, -75, 0, 0, 13.658");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store( po::command_line_parser(argc, argv).options(options).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
	  std::cout << options << "\n";
	  exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	// Register new converter for std::vector<int>
	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::IbeoScala_1403_Scan_2208> >
		converter(new rsb::converter::ProtocolBufferConverter<rst::claas::IbeoScala_1403_Scan_2208>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::IbeoScala_1403_ObjectData_2271>>
		converter_ObjectData_2271(new rsb::converter::ProtocolBufferConverter<rst::claas::IbeoScala_1403_ObjectData_2271>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_ObjectData_2271);

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::IbeoScala_1403_DeviceStatus>>
		converter_DeviceStatus(new rsb::converter::ProtocolBufferConverter<rst::claas::IbeoScala_1403_DeviceStatus>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_DeviceStatus);

	const off_t maxLogFileSize = 1000000;

	ibeosdk::LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);

	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);

	logFileManager.start();

	getScalaValues(logFileManager, scalaIP, rsbOutScope);

	exit(0);
}
