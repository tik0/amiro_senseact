#include <iostream>

#include "PeakCan.hpp"
#include "ClaasCan_CombineHarvester_Listener.hpp"
#include "S10GpsListener.hpp"

int main(int argc, char **argv) {

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
	PeakCan peakcan;

	/**
	 * Open socket connections to the PeakCans
	 */
	if(peakcan.setUpSocket(interface_1, peakcan.socketHandle_1) < 0){
		std::cerr<< "Socket: " << interface_1 << " could not be created"<< std::endl;
		return -2;
	}

	if(peakcan.setUpSocket(interface_2, peakcan.socketHandle_2) < 0){
		std::cerr<< "Socket: " << interface_2 << " could not be created"<< std::endl;
		return -2;
	}

	if(peakcan.setUpSocket(interface_3, peakcan.socketHandle_3) < 0){
		std::cerr<< "Socket: " << interface_3 << " could not be created"<< std::endl;
		return -2;
	}

	if(peakcan.setUpSocket(interface_4, peakcan.socketHandle_4) < 0){
		std::cerr<< "Socket: " << interface_4 << " could not be created"<< std::endl;
		return -2;
	}

	/**
	 * Start for every CAN interface a thread that reads the CAN messages and publishes them to rsb
	 */
	std::thread can0_Read(&PeakCan::readCanFrame_1, peakcan, peakcan.socketHandle_4);
	std::thread can1_Read(&PeakCan::readCanFrame_2, peakcan, peakcan.socketHandle_3);
	std::thread can2_Read(&PeakCan::readCanFrame_3, peakcan, peakcan.socketHandle_2);
	std::thread can3_Read(&PeakCan::readCanFrame_4, peakcan, peakcan.socketHandle_1);

	std::thread can0_Write(&PeakCan::writeCanFrame_ClaasCan, peakcan, peakcan.socketHandle_4);

	m_socketHandle_1 = peakcan.socketHandle_1;
	m_socketHandle_2 = peakcan.socketHandle_2;
	m_socketHandle_3 = peakcan.socketHandle_3;
	m_socketHandle_4 = peakcan.socketHandle_4;

	S10Gps_Listener s10Gps_Listener;
	s10Gps_Listener.setUpListener();

	ClaasCan_CombineHarvester_Listener claasCan_CombineHarvester_Listener;
	claasCan_CombineHarvester_Listener.setUpListener();

	can0_Read.join();
	can1_Read.join();
	can2_Read.join();
	can3_Read.join();

	can0_Write.join();

	std::cout<< "Closing PeakCan Connection" << "\n";
}

