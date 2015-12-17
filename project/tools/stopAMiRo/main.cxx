//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the proximity ring sensor data of AMiRo
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

  for(int led=0; led<8; led++) {
    CAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
  }
  INFO_MSG("AMiRo lights reset.");

  return EXIT_SUCCESS;
}
