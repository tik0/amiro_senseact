/*
 * infraRed.h
 *
 *  Created on: 07.03.2014
 *      Author: tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

// Debug messages
// These are pre-processor flags which can be commented out
// to disable the messages
#define INFO_MSG_
// #define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <bebot.h>

// #include "rsbInformer.h"

class motorControl{

public:

	motorControl();
	~motorControl();

	/**
	 * Set the speed of the engines with v_L and v_R in mm per second 
	 */
	void setSpeedLR( int p_vLeft_mmPs, int p_vRight_mmPs);
	
	/**
	 * Set the speed of the engines with velocity v in mm per second and angular velocity w in degree per second
	 */
	void setSpeedVW( int p_v_mmPs, int p_w_dPs);

private:
	/**
	 * Holds the bebot structure
	 */
	struct bebot BEBOT;
	
	/**
	 * Width of bebot in millimeter
	 */
	static const float uiWidth_dm = 0.75f;
	
};
#endif /* MOTORCONTROL_H_ */