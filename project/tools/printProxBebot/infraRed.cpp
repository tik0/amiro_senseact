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
  // Alloc the size of the vector
  for (unsigned int i = 0; i < BEBOT_BRIGHTNESS_COUNT; i++) {
    irSensors.push_back(0);
  }
}

InfraRed::~InfraRed() {
 bebot_release(&BEBOT);
}

void InfraRed::getIRData() {
  if ( bebot_update(&BEBOT) != -1 ) {
    for (std::size_t idx = 0; idx < BEBOT_BRIGHTNESS_COUNT; idx++) {
      irSensors.at(idx) = bebot_get_brightness(&BEBOT, idx);
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
      ssIrValueTmp << irSensors.at(idx);
      for (std::size_t idx2 = ssIrValueTmp.str().size(); idx2 <= this->uiMaxPotIR ; idx2++) {
	sIrValues.push_back(' ');
      }
      sIrValues.append(ssIrValueTmp.str());
      ssIrValueTmp.str(std::string());
  }
  
  // Print out the data
  INFO_MSG( "IR Data: " << sIrValues )
}

void InfraRed::sendIRData() {

//   //TODO check if the sending of an array is working. Maybe the datatype is declared wrong.
//   rsbInformer* info = new rsbInformer();  // for std::string -> your type
//   info->setup_vec(Constants::RSB_SCOPE_SENSE_BEHAVIOUR);
//   //std::cout << "try to get IR data" << std::endl;
// 
// //	while(bebot_poll(bebot, -1) > 0) {
// 
//   //		if (bebot_update(bebot) > 0) {
//   this->getIRData();
//   //		std::cout << "try to publish IR data" << std::endl;
//   //everytime when getting data send it to our scope.
//   info->publish(*irSensors);
//   //	}
// //	}

}
