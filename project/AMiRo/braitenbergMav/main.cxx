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
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

#include <math.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

using namespace std;
using namespace muroxConverter;


void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN) {
  
  CAN.setTargetSpeed(speed, angle);
  DEBUG_MSG( "v: " << speed << "w: " << angle);
}

int mymcm(int mym) {
  return mym*10000;
}

int main(int argc, char **argv) {

  // Init the CAN interface
  ControllerAreaNetwork CAN;    

  // Datastructure for the CAN messages
  std::vector< uint16_t > proximityRingValue(8,0);

  
  int fail = 1;
  uint8_t sensorIdx = 0;
  sendMotorCmd(mymcm(5), 0, CAN);
  bool ok = true;
  bool turn = 0;
  while(ok) {
    // Read the proximity data
    fail = CAN.getProximityRingValue(proximityRingValue);
    if (fail == 0) {
      for (sensorIdx = 0; sensorIdx < proximityRingValue.size(); sensorIdx++) {
        //INFO_MSG( (int) sensorIdx << ": " << proximityRingValue[sensorIdx]);
      }
      int valueLeft = proximityRingValue[3];
      int valueRight = proximityRingValue[4];
      int valueSide = max(proximityRingValue[2], proximityRingValue[5]);
      INFO_MSG("left: " << valueLeft << ", right: " << valueRight << ", side: " << valueSide);
      if (valueLeft > 2600 || valueRight > 2600 || valueSide > 10000) {
        if (turn == 0 && valueLeft < valueRight) {
          turn = 1;
          sendMotorCmd(0, mymcm(60), CAN);
        } else if (turn == 0) {
          turn = 2;
          sendMotorCmd(0, mymcm(-60), CAN);
        }
      } else {
        turn = 0;
        if (valueLeft > 2500 || valueRight > 2500) {
          sendMotorCmd(mymcm(10), 0, CAN);
        } else {
          sendMotorCmd(mymcm(15), 0, CAN);
        }
      }
    } else {
      WARNING_MSG( "Fail" );
      ok = false;
    }
      
    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(10) );
  }
  sendMotorCmd(0,0,CAN);

  return EXIT_SUCCESS;
}
