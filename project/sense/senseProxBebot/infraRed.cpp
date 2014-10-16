/*
 * infraRed.cpp
 *
 *  Created on: 07.03.2014
 *      Author: tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
 */

#include "infraRed.h"


InfraRed::InfraRed() {
  // Start up with initializing the robot
  bebot_init(&BEBOT);
  // Get the shared pointer
  irSensors = boost::make_shared< std::vector<int> > ( std::vector<int>(BEBOT_BRIGHTNESS_COUNT,0) );

}

InfraRed::~InfraRed() {
 bebot_release(&BEBOT);
}

void InfraRed::getIRData() {
  if ( bebot_update(&BEBOT) != -1 ) {
    for (std::size_t idx = 0; idx < BEBOT_BRIGHTNESS_COUNT; idx++) {
      irSensors->at(idx) = bebot_get_brightness(&BEBOT, idx);
    }
  }
}

void InfraRed::printIRData() {
  std::string sIrValues;
  std::stringstream ssIrValueTmp;
  
  // Format the output nicely
  // 1. Get the value an convert it to a string
  // 2. Get the size
  // 3. Append missing character as blank space e.g. ' '
  for (std::size_t idx = 0; idx < BEBOT_BRIGHTNESS_COUNT; idx++) {
      ssIrValueTmp << irSensors->at(idx);
      for (std::size_t idx2 = ssIrValueTmp.str().size(); idx2 <= this->uiMaxPotIR ; idx2++) {
	sIrValues.push_back(' ');
      }
      sIrValues.append(ssIrValueTmp.str());
      ssIrValueTmp.str(std::string());
  }
  
  // Print out the data
  INFO_MSG( "IR Data: " << sIrValues )
}