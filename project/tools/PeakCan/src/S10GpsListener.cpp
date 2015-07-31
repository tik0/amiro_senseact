/*
 * S10GpsListener.cpp
 *
 *  Created on: 31.07.2015
 *      Author: itsowl
 */

#include "S10GpsListener.hpp"

#include <iostream>

#include "PeakCan.hpp"
#include <cmath>

void get_S10Gps_Data(rsb::EventPtr canEvent){

	std::string type =  canEvent->getType();
	std::cout << "Type: " << type << "\n";
	if(type.compare("rst::claas::Nmea_GPRMC") == 0){
		boost::shared_ptr<rst::claas::Nmea_GPRMC> sample_data = boost::static_pointer_cast<rst::claas::Nmea_GPRMC>(canEvent->getData());

		struct can_frame frame= {0};
		frame.can_id = 0x9C5AECF0;
		frame.can_dlc = 8;

		uint32_t lat = (sample_data->pos().lat() + 90) * 10000000;
		uint32_t lon = (sample_data->pos().lon() + 180) * 10000000;

		lat = bswap_32(lat);
		lon = bswap_32(lon);

		memcpy(&frame.data[0], &lat, sizeof(lat));
		memcpy(&frame.data[4], &lon, sizeof(lon));
		std::cout << "Lat: " << lat << " Speed: " << sample_data->speed() << "  Lon: " << lon << " Direction: "<< sample_data->direction() << "\n";

		PeakCan::writeCanFrame(frame, m_socketHandle_4);

		frame.can_id = 0x9C5BECF0;

		uint16_t direction = (((sample_data->direction() * M_PI) /180) + M_PI) * 1000;
		uint16_t speed = (sample_data->speed() + 100) * 100;

		direction = bswap_16(direction);
		speed = bswap_16(speed);

		memcpy(&frame.data[0], &direction, sizeof(direction));
		memcpy(&frame.data[2], &speed, sizeof(speed));

		PeakCan::writeCanFrame(frame, m_socketHandle_4);

	}

}

S10Gps_Listener::S10Gps_Listener() {
	// TODO Auto-generated constructor stub

}

S10Gps_Listener::~S10Gps_Listener() {
	// TODO Auto-generated destructor stub
}

int S10Gps_Listener::setUpListener(){

	rsc::misc::initSignalWaiter();
	std::string scope = "/sense/S10Gps";
	rsb::Factory& factory = rsb::getFactory();
	rsb::ListenerPtr listener = factory.createListener(scope);
	listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&get_S10Gps_Data)));
	return rsc::misc::waitForSignal();
}
