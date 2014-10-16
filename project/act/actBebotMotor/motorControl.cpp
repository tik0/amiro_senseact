/*
 * motorControl.cpp
 *
 *  Created on: 07.03.2014
 *      Author: tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
 */

#include "motorControl.h"


motorControl::motorControl() {
  // Start up with initializing the robot
  bebot_init(&BEBOT);
}

motorControl::~motorControl() {
  bebot_release(&BEBOT);
}

void motorControl::setSpeedLR( int p_vLeft_mmPs, int p_vRight_mmPs) {
  // Print out some info
//   INFO_MSG( "Speed is set" )
  // Set the speed
  bebot_set_speed(&BEBOT, p_vLeft_mmPs, p_vRight_mmPs);
}

void motorControl::setSpeedVW( int p_v_mmPs, int p_w_dPs) {
  // Print out some info
//   INFO_MSG( "Speed is set" )
  DEBUG_MSG( "v=" << p_v_mmPs << "mm/s" )
  DEBUG_MSG( "w=" << p_w_dPs << "degree/s")
  DEBUG_MSG( "v_L=" << (int)(- uiWidth_cm / 2.0f * (float) p_w_dPs) + p_v_mmPs << "mm/s" )
  DEBUG_MSG( "v_R=" << (int)(  uiWidth_cm / 2.0f * (float) p_w_dPs) + p_v_mmPs << "mm/s" )
  // Set the speed
  bebot_set_speed(&BEBOT,
		  (int)(- uiWidth_dm / 2.0f * (float) p_w_dPs) + p_v_mmPs,
		  (int)(  uiWidth_dm / 2.0f * (float) p_w_dPs) + p_v_mmPs);
}