//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Stops the motors of the AMiRo and "resets" the lights.
//============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

using namespace std;

int main(int argc, char **argv) {
  // Init the CAN interface
  ControllerAreaNetwork CAN;    
  // Stop motor
  CAN.setTargetSpeed(0, 0);
  INFO_MSG("AMiRo stopped!");

  for(int led=0; led<8; led++) {
    CAN.setLightColor(led, amiro::Color(amiro::Color::WHITE));
  }

  sleep(1);

  CAN.setLightColor(0, amiro::Color(amiro::Color::RED));
  CAN.setLightColor(1, amiro::Color(amiro::Color::GREEN));
  CAN.setLightColor(2, amiro::Color(amiro::Color::BLUE));
  CAN.setLightColor(3, amiro::Color(amiro::Color::WHITE));
  CAN.setLightColor(4, amiro::Color(amiro::Color::RED));
  CAN.setLightColor(5, amiro::Color(amiro::Color::GREEN));
  CAN.setLightColor(6, amiro::Color(amiro::Color::BLUE));
  CAN.setLightColor(7, amiro::Color(amiro::Color::WHITE));
  INFO_MSG("AMiRo lights reset.");

  return EXIT_SUCCESS;
}
