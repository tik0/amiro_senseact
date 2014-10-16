//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Example of CAN communication
//============================================================================


 #define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

// Reading inScope and outScope
// #include <ioFlags.hpp>

// Reading inScope, outScope and freq
#include <ofFlags.hpp>

#include <ControllerAreaNetwork.h>

#include <vector>
//#include "ControllerAreaNetwork.h"
#include <iostream>

#include <Types.h>  // types::position
//#include <boost/thread.hpp>
//#include <boost/program_options.hpp>
//#include <boost/shared_ptr.hpp>


//#include <rsb/Informer.h>
//#include <rsb/Factory.h>
//#include <rsb/Event.h>
//#include <rsb/Handler.h>
//#include <rsb/converter/Repository.h>


using namespace std;
using namespace amiro;


/* omitted vital #includes and error checking */


int main(int argc, char **argv)
{

  ControllerAreaNetwork myCAN;

  types::position robotPosition;
  robotPosition.x = 0;
  robotPosition.y = 0;
  robotPosition.f_z = 0;
  myCAN.setOdometry(robotPosition);


 INFO_MSG( "Drive forward with 10 cm/s and 1 rad/s" )
 int32_t v = 100000, w = 100000;
 myCAN.setTargetSpeed(v,w);


 INFO_MSG ("Reading the odometry for 20 times:" );
 for (int idx = 0; idx < 20 ; ++idx ){
   types::position robotPosition = myCAN.getOdometry();
   INFO_MSG(" x: " << robotPosition.x << " y: " << robotPosition.y << " f_z: " << robotPosition.f_z)
 }



 INFO_MSG ("Reading the odometry for 20 times:" );
 for (int idx = 0; idx < 20 ; ++idx ){
   types::position robotPosition = myCAN.getOdometry();
   INFO_MSG(" x: " << robotPosition.x << " y: " << robotPosition.y << " f_z: " << robotPosition.f_z)
 }

 INFO_MSG ("Drive stop" )
 v = 0, w = 0;
 myCAN.setTargetSpeed(v,w);



 return 0;

}


//int main(int argc, char **argv)
//{
//
//  ControllerAreaNetwork myCAN;
//
//
//
// INFO_MSG ("Reading the proximity floor sensors for 20 times:" )
// for (int idx = 0; idx < 0 ; ++idx ){
//   std::vector<uint16_t> prox(4,0);
//   if (myCAN.getProximityFloorValue(prox) == 0) {
//     for (int sensorIdx = 0; sensorIdx < prox.size(); sensorIdx++)
//       INFO_MSG( prox[sensorIdx] )
//    } else {
//     WARNING_MSG( "Fail" )
//    }
//    INFO_MSG( "-------" )
// }
//
// INFO_MSG ("Reading the proximity ring sensors for 20 times:" )
// for (int idx = 0; idx < 0 ; ++idx ){
//   std::vector<uint16_t> prox(8,0);
//   if (myCAN.getProximityRingValue(prox) == 0) {
//     for (int sensorIdx = 0; sensorIdx < prox.size(); sensorIdx++)
//       INFO_MSG( prox[sensorIdx] )
//   } else {
//     WARNING_MSG( "Fail" )
//   }
//   INFO_MSG( "-------" )
// }
//
// INFO_MSG( "Drive forward with 10 cm/s and 1 rad/s" )
// int32_t v = 100000, w = 0;
// myCAN.setTargetSpeed(v,w);
//
//
// int32_t leftWheelRpm,rightWheelRpm;
// INFO_MSG ("Reading the left and right wheel rpm for 20 times:" );
// for (int idx = 0; idx < 0 ; ++idx ){
//   if (myCAN.getSpeedRpm(leftWheelRpm,rightWheelRpm) == 0) {
//     INFO_MSG( leftWheelRpm << " " << rightWheelRpm )
//   } else {
//     WARNING_MSG( "Fail" )
//   }
// }
//
// INFO_MSG ("Reading the odometry for 20 times:" );
// for (int idx = 0; idx < 2000000 ; ++idx ){
//   types::position robotPosition = myCAN.getOdometry();
//   INFO_MSG(" x: " << robotPosition.x << " y: " << robotPosition.y << " f_z: " << robotPosition.f_z)
// }
//
//
// INFO_MSG ("Drive stop" )
// v = 0, w = 0;
// myCAN.setTargetSpeed(v,w);
//
//
//
//
// // Working with LEDs
//
// int brightness = 100;
// INFO_MSG("Set brightness to  "<< brightness );
// myCAN.setLightBrightness(brightness);
//
// for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
//   INFO_MSG("Set LED " << ledIdx << " to magenta" )
//   myCAN.setLightColor(ledIdx, amiro::Color::MAGENTA);
//   sleep(1);
// }
//
// for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
//   INFO_MSG( "Set LED " << ledIdx << " to white" )
//    myCAN.setLightColor(ledIdx, amiro::Color(255,255,255));
//    sleep(1);
//  }
//
// brightness = 100;
// INFO_MSG("Set brightness to  "<< brightness )
// myCAN.setLightBrightness(brightness);
//
// brightness = 10;
// INFO_MSG("Set brightness to  "<< brightness )
// myCAN.setLightBrightness(brightness);
//
// return 0;
//
//}
