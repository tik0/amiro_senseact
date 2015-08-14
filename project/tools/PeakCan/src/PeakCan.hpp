/*
 * PeakCan.hpp
 *
 *  Created on: 03.06.2015
 *      Author: Andreas Skiba
 */

#ifndef PEAKCAN_HPP_
#define PEAKCAN_HPP_

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <unistd.h>
#include <string>
#include <cstring>
#include <byteswap.h>

#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <CanMessage.pb.h>

#include <thread>
#include <chrono>

#include <signal.h>

extern int m_socketHandle_1;
extern int m_socketHandle_2;
extern int m_socketHandle_3;
extern int m_socketHandle_4;
extern int m_socketHandle_5;
extern int m_socketHandle_6;

class PeakCan {
public:
	PeakCan();
	//virtual ~PeakCan();

	/**
	 * Creates a socket connection
	 * @param interface to be used for the socket communication
	 * @param socketHandle the bind function assigns an address to the socketHandle
	 * @return 0 on success and -1 if binding failed
	 */
	int setUpSocket(std::string& interface, int& socketHandle);

//	template< typename R>
//	void readCanFrame(int socketHandle){
//
//		struct can_frame frame;
//		rsb::Factory& factory = rsb::getFactory();
//		//T converter;
//		R canDatabase;
//
//		while(true){
//
//			int bytes_read = read( socketHandle, &frame, sizeof(frame) );
//
//			if(bytes_read < 0){
//				return;
//			}
//			//std::cout << "ID: " << frame.can_id << "  " << bytes_read <<std::endl;
//			uint32_t canId = frame.can_id & 0x7FFFFFFF;
//			try{
//				canDatabase.getCanMessages().at(canId)->getSignals(&frame.data[0], factory);
//			}catch (const std::out_of_range& e) {
//				//std::cerr<< "Id not found  " << std::hex << canId << std::dec <<std::endl;
//			}
//		}
//	}

	void writeCanFrame_ClaasCan(int socketHandle);
	void writeCanFrame_ClaasCan_cCoecCaccDevStat1(int socketHandle);
	void writeCanFrame_ClaasCan_feedingHeight(int socketHandle);
	void writeCanFrame_ClaasCan_feedingHeight_Offset(int socketHandle);

	void readCanFrame_1(int socketHandle);
	void readCanFrame_2(int socketHandle);
	void readCanFrame_3(int socketHandle);
	void readCanFrame_4(int socketHandle);
	void readCanFrame_5(int socketHandle);

	int socketHandle_1;
	int socketHandle_2;
	int socketHandle_3;
	int socketHandle_4;
	int socketHandle_5;
	int socketHandle_6;

	static bool runningHandle_1;
	static bool runningHandle_2;
	static bool runningHandle_3;
	static bool runningHandle_4;
	static bool runningHandle_5;
	static bool runningHandle_6;
	static bool wrintingHandle_1;

	static void writeCanFrame(struct can_frame& frame, int socketHandle){
		int bytes_write = write(socketHandle, &frame, sizeof(struct can_frame));
		if(bytes_write < 1){
			std::cerr<<"Could not write CAN messages: " << std::hex << frame.can_id << std::dec << std::endl;
		}
	}

	static void closeSocket(int socketHandle){
		std::cout << "Closing socket: " << socketHandle << "\n";
		int disconnect = close(socketHandle);
		if(disconnect < 0){
			std::cerr << "Failure closing socket: " << socketHandle << "\n";
		}
	}

};

#endif /* PEAKCAN_HPP_ */
