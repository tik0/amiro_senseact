//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Print steering and proximity values
//============================================================================



#include <ControllerAreaNetwork.h>

#include <vector>
#include <iostream>


using namespace std;
using namespace amiro;


/* omitted vital #includes and error checking */

int main(int argc, char **argv)
{

  ControllerAreaNetwork myCAN;

// std::cout << "Drive forward with 10 cm/s" << std::endl;
//  int32_t v = 100000, w = 1000000;
//   myCAN.setTargetSpeed(v,w);

const float wheelDiameterSI = 0.05571;
const float pi = 3.14159265358979323846;
const int secondsPerMinute = 60;
const float wheelCircumferenceSI = pi * wheelDiameterSI;
const int wheelCircumference = pi * wheelDiameterSI * 1e6;
const int wheelDistance = 0.08f * 1e6;
const float wheelDistanceSI = 0.08f;
while(true) {
  int32_t leftWheelRpm,rightWheelRpm;
   if (myCAN.getTargetSpeedRpm(leftWheelRpm,rightWheelRpm) == 0) {
     int v_x = (wheelCircumference * (leftWheelRpm + rightWheelRpm)) / secondsPerMinute / 2;
     int w_z = float(wheelCircumference * (rightWheelRpm - leftWheelRpm)) / wheelDistanceSI / secondsPerMinute;
//      std::cout << leftWheelRpm << "," << rightWheelRpm << std::endl;
     std::cout << v_x << "," << w_z << std::endl;
   } else {
     std::cout << "Fail" << std::endl;
   }

      std::vector<uint16_t> prox(8,0);
      myCAN.getProximityValue(prox);

      std::cout << prox[0] << "," << prox[1] << "," << prox[2] << "," << prox[3] << "," << prox[4] << ",";
      std::cout << prox[5] << "," << prox[6] << "," << prox[7] << std::endl;

}


 return 0;

}
