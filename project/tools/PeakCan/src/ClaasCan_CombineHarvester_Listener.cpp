/*
 * ClaasCan_CombineHarvester_Listener.cpp
 *
 *  Created on: 10.06.2015
 *      Author: itsowl
 */

#include "ClaasCan_CombineHarvester_Listener.hpp"
#include <CanMessage.pb.h>
#include <ClaasCan_CombineHarvester_mac.h>

//RSC
#include <rsc/misc/SignalWaiter.h>

#include "PeakCan.hpp"

void get_cCascBrc_LaserScnLeftData1(rsb::EventPtr canEvent){

	boost::shared_ptr<rst::claas::CanMessage> sample_data =
			boost::static_pointer_cast<rst::claas::CanMessage>(canEvent->getData());

	unsigned char canData[8];
	for(unsigned int i = 0; i< sample_data->candlc(); ++i){
		canData[i] = sample_data->canmessage(i);
	}

	if(sample_data->canid() == cCascBrc_LaserScnLeftData1_ID){

		uint16_t dig = cCascBrc_LaserScnLeftData1_LaserScnLeft_dig_GETP(canData);
		uint8_t hbar = cCascBrc_LaserScnLeftData1_LaserScnLeft_Hbar_GETP(canData);
		uint8_t vbar = cCascBrc_LaserScnLeftData1_LaserScnLeft_Vbar_GETP(canData);
		uint16_t avgdistance = cCascBrc_LaserScnLeftData1_LaserScnLeft_AvgDistance_GETP(canData);

		struct can_frame frame= {0};
		frame.can_id = 0x94230090;
		frame.can_dlc = 8;

		memcpy(&frame.data[0], &dig, sizeof(dig));
		memcpy(&frame.data[2], &hbar, sizeof(hbar));
		memcpy(&frame.data[3], &vbar, sizeof(vbar));
		memcpy(&frame.data[4], &avgdistance, sizeof(avgdistance));

		PeakCan::writeCanFrame(frame, m_socketHandle_4);
	}
	else if (sample_data->canid() == cCascBrc_LaserScnLeftData2_ID) {

		uint8_t laserScnLeftStatus = cCascBrc_LaserScnLeftData2_LaserScnLeft_Status_GETP(canData);

		struct can_frame frame= {0};
		frame.can_id = 0x94240090;
		frame.can_dlc = 8;

		memcpy(&frame.data[0], &laserScnLeftStatus, sizeof(laserScnLeftStatus));

		PeakCan::writeCanFrame(frame, m_socketHandle_4);
	}
	else if(sample_data->canid() == cCascBrc_LaserScnRightData1_ID){

		uint16_t dig = cCascBrc_LaserScnRightData1_LaserScnRight_dig_GETP(canData);
		uint8_t hbar = cCascBrc_LaserScnRightData1_LaserScnRight_Hbar_GETP(canData);
		uint8_t vbar = cCascBrc_LaserScnRightData1_LaserScnRight_Vbar_GETP(canData);
		uint16_t avgdistance = cCascBrc_LaserScnRightData1_LaserScnRight_AvgDistance_GETP(canData);

		struct can_frame frame= {0};
		frame.can_id = 0x94260090;
		frame.can_dlc = 8;

		memcpy(&frame.data[0], &dig, sizeof(dig));
		memcpy(&frame.data[2], &hbar, sizeof(hbar));
		memcpy(&frame.data[3], &vbar, sizeof(vbar));
		memcpy(&frame.data[4], &avgdistance, sizeof(avgdistance));

		PeakCan::writeCanFrame(frame, m_socketHandle_4);
	}
	else if (sample_data->canid() == cCascBrc_LaserScnRightData2_ID) {

		uint8_t laserScnRightStatus = cCascBrc_LaserScnRightData2_LaserScnRight_Status_GETP(canData);

		struct can_frame frame= {0};
		frame.can_id = 0x94270090;
		frame.can_dlc = 8;

		memcpy(&frame.data[0], &laserScnRightStatus, sizeof(laserScnRightStatus));

		PeakCan::writeCanFrame(frame, m_socketHandle_4);
	}

}

int ClaasCan_CombineHarvester_Listener::setUpListener(){

	rsc::misc::initSignalWaiter();
	std::string scope = "/act/ClaasCan/tx";
	rsb::Factory& factory = rsb::getFactory();
	rsb::ListenerPtr listener = factory.createListener(scope);
	listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&get_cCascBrc_LaserScnLeftData1)));
	//listener->addFilter(rsb::filter::FilterPtr(new rsb::filter::TypeFilter("ClaasCan_CombineHarvester::cCascBrc_LaserScnLeftData1")));
	return rsc::misc::waitForSignal();
}

