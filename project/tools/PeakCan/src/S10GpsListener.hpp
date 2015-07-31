/*
 * S10GpsListener.hpp
 *
 *  Created on: 31.07.2015
 *      Author: itsowl
 */

#ifndef SRC_S10GPSLISTENER_HPP_
#define SRC_S10GPSLISTENER_HPP_

//RSC
#include <rsc/misc/SignalWaiter.h>

#include <Nmea.pb.h>

class S10Gps_Listener {
public:
	S10Gps_Listener();
	virtual ~S10Gps_Listener();

	int setUpListener();

};

#endif /* SRC_S10GPSLISTENER_HPP_ */
