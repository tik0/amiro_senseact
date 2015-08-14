#include <iostream>

#include "PeakCan.hpp"
#include "ClaasCan_CombineHarvester_Listener.hpp"
#include "S10GpsListener.hpp"

void closingHandler(int signum){
	PeakCan::runningHandle_1 = false;
	PeakCan::runningHandle_2 = false;
	PeakCan::runningHandle_3 = false;
	PeakCan::runningHandle_4 = false;
	PeakCan::runningHandle_5 = false;
	PeakCan::runningHandle_6 = false;
	PeakCan::wrintingHandle_1 = false;
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	std::cout << "Stopping all Threads" << std::endl;
}

int main(int argc, char **argv) {

	rsc::misc::initSignalWaiter();

	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = closingHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);

    PeakCan peakcan;
	/**
	 * Register all converters for CAN interface
	 */
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::CanMessage>> converter(new rsb::converter::ProtocolBufferConverter<rst::claas::CanMessage>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGGA> > converter_GPGGA(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGGA>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPGGA);

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGSA> > converter_GPGSA(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGSA>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPGSA);

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGSV> > converter_GPGSV(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGSV>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPGSV);

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPRMC> > converter_GPRMC(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPRMC>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPRMC);

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPVTG> > converter_GPVTG(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPVTG>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPVTG);

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGLL> > converter_GPGLL(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGLL>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPGLL);

	std::string interface_1 = "can0";
	std::string interface_2 = "can1";
	std::string interface_3 = "can2";
	std::string interface_4 = "can3";
	std::string interface_5 = "can4";
	std::string interface_6 = "can5";

	std::vector<std::thread> vecThreads;
	/**
	 * Open socket connections to the PeakCans
	 * Start for every CAN interface a thread that reads the CAN messages and publishes them to rsb
	 */
	int result = peakcan.setUpSocket(interface_1, peakcan.socketHandle_1);
	if(result < 0){
		std::cerr<< "Socket: " << interface_1 << " SteeringCan could not be created"<< std::endl;
	}
	else
	{
		//can3_Read = std::thread(&PeakCan::readCanFrame_4, peakcan, peakcan.socketHandle_1);
		vecThreads.push_back(std::thread(&PeakCan::readCanFrame_4, peakcan, peakcan.socketHandle_1));
	}

	result = peakcan.setUpSocket(interface_2, peakcan.socketHandle_2);
	if(result < 0){
		std::cerr<< "Socket: " << interface_2 << " FrontCan could not be created"<< std::endl;
	}
	else
	{
		//can2_Read = std::thread(&PeakCan::readCanFrame_3, peakcan, peakcan.socketHandle_2);
		vecThreads.push_back(std::thread(&PeakCan::readCanFrame_3, peakcan, peakcan.socketHandle_2));
	}

	result = peakcan.setUpSocket(interface_3, peakcan.socketHandle_3);
	if(result < 0){
		std::cerr<< "Socket: " << interface_3 << " J1939Can could not be created"<< std::endl;
	}
	else
	{
		//can1_Read = std::thread(&PeakCan::readCanFrame_2, peakcan, peakcan.socketHandle_3);
		vecThreads.push_back(std::thread(&PeakCan::readCanFrame_2, peakcan, peakcan.socketHandle_3));
	}

	result = peakcan.setUpSocket(interface_4, peakcan.socketHandle_4);
	if(result< 0){
		std::cerr<< "Socket: " << interface_4 << " ClaasCan could not be created"<< std::endl;
	}
	else
	{
		//can0_Read = std::thread(&PeakCan::readCanFrame_1, peakcan, peakcan.socketHandle_4);
		//can0_Write = std::thread(&PeakCan::writeCanFrame_ClaasCan, peakcan, peakcan.socketHandle_4);
		vecThreads.push_back(std::thread(&PeakCan::readCanFrame_1, peakcan, peakcan.socketHandle_4));
		vecThreads.push_back(std::thread(&PeakCan::writeCanFrame_ClaasCan, peakcan, peakcan.socketHandle_4));
	}

//	result = peakcan.setUpSocket(interface_1, peakcan.socketHandle_5);
//	if(result < 0){
//		std::cerr<< "Socket: " << interface_1 << " J1939Can could not be created"<< std::endl;
//	}
//	else
//	{
//		vecThreads.push_back(std::thread(&PeakCan::readCanFrame_5, peakcan, peakcan.socketHandle_5));
//	}

	m_socketHandle_1 = peakcan.socketHandle_1;
	m_socketHandle_2 = peakcan.socketHandle_2;
	m_socketHandle_3 = peakcan.socketHandle_3;
	m_socketHandle_4 = peakcan.socketHandle_4;
	m_socketHandle_5 = peakcan.socketHandle_5;
	m_socketHandle_6 = peakcan.socketHandle_6;

	/**
	 *
	 */
//	std::thread can0_Read(&PeakCan::readCanFrame_1, peakcan, peakcan.socketHandle_4);
//	std::thread can1_Read(&PeakCan::readCanFrame_2, peakcan, peakcan.socketHandle_3);
//	std::thread can2_Read(&PeakCan::readCanFrame_3, peakcan, peakcan.socketHandle_2);
//	std::thread can3_Read(&PeakCan::readCanFrame_4, peakcan, peakcan.socketHandle_1);
//	std::thread can4_Read(&PeakCan::readCanFrame_5, peakcan, peakcan.socketHandle_1);
//	std::thread can0_Write(&PeakCan::writeCanFrame_ClaasCan, peakcan, peakcan.socketHandle_4);

	S10Gps_Listener s10Gps_Listener;
	s10Gps_Listener.setUpListener();

	ClaasCan_CombineHarvester_Listener claasCan_CombineHarvester_Listener;
	claasCan_CombineHarvester_Listener.setUpListener();

	for(auto &threads : vecThreads){
		if(threads.joinable()){
			threads.join();
		}
	}

	PeakCan::closeSocket(peakcan.socketHandle_5);
	PeakCan::closeSocket(peakcan.socketHandle_4);
	PeakCan::closeSocket(peakcan.socketHandle_3);
	PeakCan::closeSocket(peakcan.socketHandle_2);
	PeakCan::closeSocket(peakcan.socketHandle_1);
	std::cout<< "Closing PeakCan Connection" << "\n";

//	return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());
}

