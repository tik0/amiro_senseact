/*
 * PeakCan.cpp
 *
 *  Created on: 03.06.2015
 *      Author: Andreas Skiba
 */

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

#include "PeakCan.hpp"

int m_socketHandle_1 = 0;
int m_socketHandle_2 = 0;
int m_socketHandle_3 = 0;
int m_socketHandle_4 = 0;
int m_socketHandle_5 = 0;
int m_socketHandle_6 = 0;

PeakCan::PeakCan() : socketHandle_1(0), socketHandle_2(0), socketHandle_3(0),socketHandle_4(0),socketHandle_5(0),socketHandle_6(0){

}

/**
 * Creates a socket connection
 * @param interface to be used for the socket communication
 * @param socketHandle the bind function assigns an address to the socketHandle
 * @return 0 on success and -1 if binding failed
 */
int PeakCan::setUpSocket(std::string& interface, int &socketHandle){

	//create the socket
	socketHandle = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(socketHandle < 0){
		return socketHandle;
	}

	struct ifreq ifr;
	//Locate the interface you wish to use
	std::strcpy(ifr.ifr_name, interface.c_str());
	ioctl(socketHandle, SIOCGIFINDEX, &ifr);

	//Select that CAN interface, and bind the socket to it.
	struct sockaddr_can addr;
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int result = ::bind(socketHandle, (struct sockaddr *)&addr, sizeof(addr));

	return result;
}

void PeakCan::readCanFrame_1(int socketHandle){
	struct can_frame frame;

	rsb::Factory& factory = rsb::getFactory();
	rsb::Informer<rst::claas::CanMessage>::Ptr informer_vec = factory.createInformer<rst::claas::CanMessage>("/sense/ClaasCan/rx");
	rsb::Informer<rst::claas::CanMessage>::DataPtr msg(new rst::claas::CanMessage);

	while(true){

		int bytes_read = read( socketHandle, &frame, sizeof(frame) );

		if(bytes_read < 0){
			return;
		}

		uint32_t canId = frame.can_id & 0x7FFFFFFF;

		msg->set_canid(canId);
		msg->set_candlc(frame.can_dlc);
		for(int i = 0; i < frame.can_dlc; ++i){
			msg->add_canmessage(frame.data[i]);
		}

		informer_vec->publish(msg);
		msg->Clear();
	}
}

void PeakCan::readCanFrame_2(int socketHandle){
	struct can_frame frame;

	rsb::Factory& factory = rsb::getFactory();
	rsb::Informer<rst::claas::CanMessage>::Ptr informer_vec = factory.createInformer<rst::claas::CanMessage>("/sense/J1939Can/rx");
	rsb::Informer<rst::claas::CanMessage>::DataPtr msg(new rst::claas::CanMessage);

	while(true){

		int bytes_read = read( socketHandle, &frame, sizeof(frame) );

		if(bytes_read < 0){
			return;
		}

		uint32_t canId = frame.can_id & 0x7FFFFFFF;

		msg->set_canid(canId);
		msg->set_candlc(frame.can_dlc);
		for(int i = 0; i < frame.can_dlc; ++i){
			msg->add_canmessage(frame.data[i]);
		}

		informer_vec->publish(msg);
		msg->Clear();
	}
}

void PeakCan::readCanFrame_3(int socketHandle){
	struct can_frame frame;

	rsb::Factory& factory = rsb::getFactory();
	rsb::Informer<rst::claas::CanMessage>::Ptr informer_vec = factory.createInformer<rst::claas::CanMessage>("/sense/FrontAtt/rx");
	rsb::Informer<rst::claas::CanMessage>::DataPtr msg(new rst::claas::CanMessage);

	while(true){

		int bytes_read = read( socketHandle, &frame, sizeof(frame) );

		if(bytes_read < 0){
			return;
		}
		uint32_t canId = frame.can_id & 0x7FFFFFFF;

		msg->set_canid(canId);
		msg->set_candlc(frame.can_dlc);
		for(int i = 0; i < frame.can_dlc; ++i){
			msg->add_canmessage(frame.data[i]);
		}

		informer_vec->publish(msg);
		msg->Clear();
	}
}

void PeakCan::readCanFrame_4(int socketHandle){
	struct can_frame frame;

	rsb::Factory& factory = rsb::getFactory();
	rsb::Informer<rst::claas::CanMessage>::Ptr informer_vec = factory.createInformer<rst::claas::CanMessage>("/sense/Steering/rx");
	rsb::Informer<rst::claas::CanMessage>::DataPtr msg(new rst::claas::CanMessage);

	while(true){

		int bytes_read = read( socketHandle, &frame, sizeof(frame) );

		if(bytes_read < 0){
			return;
		}

		uint32_t canId = frame.can_id & 0x7FFFFFFF;

		msg->set_canid(canId);
		msg->set_candlc(frame.can_dlc);
		for(int i = 0; i < frame.can_dlc; ++i){
			msg->add_canmessage(frame.data[i]);
		}

		informer_vec->publish(msg);
		msg->Clear();
	}
}

void PeakCan::writeCanFrame_ClaasCan(int socketHandle){

	while(true){
		writeCanFrame_ClaasCan_cCoecCaccDevStat1(socketHandle);
		writeCanFrame_ClaasCan_feedingHeight(socketHandle);
		writeCanFrame_ClaasCan_feedingHeight_Offset(socketHandle);

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}

void PeakCan::writeCanFrame_ClaasCan_cCoecCaccDevStat1(int socketHandle){
	struct can_frame frame;
	frame.can_id  = 0x9CF13FF0;
	frame.can_dlc = 8;
	frame.data[0] = 0x01;
	frame.data[1] = 0x00;
	frame.data[2] = 0x00;
	frame.data[3] = 0x00;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;

	PeakCan::writeCanFrame(frame, socketHandle);
}

void PeakCan::writeCanFrame_ClaasCan_feedingHeight(int socketHandle){
	struct can_frame frame;
	frame.can_id  = 0x9DF10AF0;
	frame.can_dlc = 8;
	frame.data[0] = 0xB8;
	frame.data[1] = 0x00;
	frame.data[2] = 0x10;
	frame.data[3] = 0x01;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;

	PeakCan::writeCanFrame(frame, socketHandle);
}

void PeakCan::writeCanFrame_ClaasCan_feedingHeight_Offset(int socketHandle){
	struct can_frame frame;
	frame.can_id  = 0x9DF10AF0;
	frame.can_dlc = 8;
	frame.data[0] = 0xB8;
	frame.data[1] = 0x02;
	frame.data[2] = 0x10;
	frame.data[3] = 0x01;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;

	PeakCan::writeCanFrame(frame, socketHandle);
}
