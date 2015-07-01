/*
 * ClaasCan_CombineHarvester_Listener.cpp
 *
 *  Created on: 10.06.2015
 *      Author: itsowl
 */

#include <src/ClaasCan_CombineHarvester_Listener.hpp>
#include <ClaasCan_CombineHarvester.pb.h>

//RSC
#include <rsc/misc/SignalWaiter.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/filter/TypeFilter.h>

#include <unistd.h>
#include <linux/can.h>
#include <byteswap.h>

int m_socketHandle_1 = 0;
int m_socketHandle_2 = 0;
int m_socketHandle_3 = 0;
int m_socketHandle_4 = 0;
int m_socketHandle_5 = 0;
int m_socketHandle_6 = 0;

void writeCanFrame(struct can_frame& frame, int socketHandle){

	int bytes_write = write(socketHandle, &frame, sizeof(struct can_frame));
	if(bytes_write < 1){
		std::cerr<<"Could not write CAN messages: " << std::hex << frame.can_id << std::dec << std::endl;
	}
}

void get_cCascBrc_LaserScnLeftData1(rsb::EventPtr canEvent){
	//std::cout << "Event: "  << canEvent->getClassName() << " " << canEvent->getSequenceNumber() << " " << canEvent->getType() << std::endl;
	std::string canEventType = canEvent->getType();

	if(canEventType.compare("ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData1") == 0){

		boost::shared_ptr<ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData1> sample_data = boost::static_pointer_cast<ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData1>(canEvent->getData());

		uint16_t dig = bswap_16(sample_data->laserscnleft_dig__digit());
		uint8_t hbar = sample_data->laserscnleft_hbar__();
		uint8_t vbar = sample_data->laserscnleft_vbar__();
		uint16_t avgdistance = bswap_16(sample_data->laserscnleft_avgdistance__cm());

		struct can_frame frame= {0};
		frame.can_id = 0x94230090;
		frame.can_dlc = 8;

		memcpy(&frame.data[0], &dig, sizeof(dig));
		memcpy(&frame.data[2], &hbar, sizeof(hbar));
		memcpy(&frame.data[3], &vbar, sizeof(vbar));
		memcpy(&frame.data[4], &avgdistance, sizeof(avgdistance));

		writeCanFrame(frame, m_socketHandle_4);
	}
	else if (canEventType.compare("ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData2") == 0) {

		boost::shared_ptr<ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData2> sample_data = boost::static_pointer_cast<ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData2>(canEvent->getData());
		uint8_t laserScnLeftStatus = sample_data->laserscnleft_status__();

		struct can_frame frame= {0};
		frame.can_id = 0x94240090;
		frame.can_dlc = 8;

		memcpy(&frame.data[0], &laserScnLeftStatus, sizeof(laserScnLeftStatus));

		writeCanFrame(frame, m_socketHandle_4);
	}
	else if(canEventType.compare("ClaasCan_CombineHarvester::cCascBrc_LaserScnRightData1") == 0){

		boost::shared_ptr<ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData1> sample_data = boost::static_pointer_cast<ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData1>(canEvent->getData());

		uint16_t dig = bswap_16(sample_data->laserscnleft_dig__digit());
		uint8_t hbar = sample_data->laserscnleft_hbar__();
		uint8_t vbar = sample_data->laserscnleft_vbar__();
		uint16_t avgdistance = bswap_16(sample_data->laserscnleft_avgdistance__cm());

		struct can_frame frame= {0};
		frame.can_id = 0x94260090;
		frame.can_dlc = 8;

		memcpy(&frame.data[0], &dig, sizeof(dig));
		memcpy(&frame.data[2], &hbar, sizeof(hbar));
		memcpy(&frame.data[3], &vbar, sizeof(vbar));
		memcpy(&frame.data[4], &avgdistance, sizeof(avgdistance));

		writeCanFrame(frame, m_socketHandle_4);
	}
	else if (canEventType.compare("ClaasCan_CombineHarvester::cCascBrc_LaserScnRightData2") == 0) {

		boost::shared_ptr<ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData2> sample_data = boost::static_pointer_cast<ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData2>(canEvent->getData());
		uint8_t laserScnLeftStatus = sample_data->laserscnleft_status__();

		struct can_frame frame= {0};
		frame.can_id = 0x94270090;
		frame.can_dlc = 8;

		memcpy(&frame.data[0], &laserScnLeftStatus, sizeof(laserScnLeftStatus));

		writeCanFrame(frame, m_socketHandle_4);
	}

}

//ClaasCan_CombineHarvester_Listener::ClaasCan_CombineHarvester_Listener(){
//
//}
//
//ClaasCan_CombineHarvester_Listener::~ClaasCan_CombineHarvester_Listener() {
//	// TODO Auto-generated destructor stub
//}

int ClaasCan_CombineHarvester_Listener::setUpListener(){

	rsc::misc::initSignalWaiter();
	std::string scope = "/ClaasCan/tx";
	rsb::Factory& factory = rsb::getFactory();
	rsb::ListenerPtr listener = factory.createListener(scope);
	listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&get_cCascBrc_LaserScnLeftData1)));
	//listener->addFilter(rsb::filter::FilterPtr(new rsb::filter::TypeFilter("ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData1")));
	return rsc::misc::waitForSignal();
}

