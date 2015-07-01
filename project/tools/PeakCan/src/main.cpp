#include <iostream>
#include <thread>

#include "PeakCan.hpp"
#include "ClaasCan_CombineHarvester_Listener.hpp"

#include "ClaasCan_CombineHarvester.hpp"
#include "J1939Can_CombineHarvester.hpp"
#include "FrontAttachmentCan_CombineHarvester.hpp"
#include "SteeringCan.hpp"

#include "Converter_ClaasCan_CombineHarvester.hpp"
#include "Converter_J1939Can_CombineHarvester.hpp"
#include "Converter_FrontAttachmentCan_CombineHarvester.hpp"
#include "Converter_SteeringCan.hpp"


int main(int argc, char **argv) {

	/**
	 * Register all converters for CAN interface
	 */
	Converter_ClaasCan_CombineHarvester claas_converter;
	Converter_J1939Can_CombineHarvester j1939_converter;
	Converter_FrontAttachmentCan_CombineHarvester front_converter;
	Converter_SteeringCan seetring_converter;

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
	std::thread can0(&PeakCan::readCanFrame<ClaasCan_CombineHarvesterDBC>, peakcan, peakcan.socketHandle_4);
	std::thread can1(&PeakCan::readCanFrame<J1939Can_CombineHarvesterDBC>, peakcan, peakcan.socketHandle_3);
	std::thread can2(&PeakCan::readCanFrame<FrontAttachmentCan_CombineHarvesterDBC>, peakcan, peakcan.socketHandle_2);
	std::thread can3(&PeakCan::readCanFrame<SteeringCanDBC>, peakcan, peakcan.socketHandle_1);

	m_socketHandle_1 = peakcan.socketHandle_1;
	m_socketHandle_2 = peakcan.socketHandle_2;
	m_socketHandle_3 = peakcan.socketHandle_3;
	m_socketHandle_4 = peakcan.socketHandle_4;

	ClaasCan_CombineHarvester_Listener claasCan_CombineHarvester_Listener;
	claasCan_CombineHarvester_Listener.setUpListener();

	can0.join();
	can1.join();
	can2.join();
	can3.join();

	std::cout<< "Closing PeakCan Connection" << "\n";
}

