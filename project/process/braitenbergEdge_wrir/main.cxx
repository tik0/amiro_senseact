//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Performs a basic braitenberg-like obstacle avoidanve.
//               Reads sensor values via CAN. It uses the obstacle and edge
//               avoidence behavior. Steering and light commands are given
//               via CAN.
//============================================================================

//#define TRACKING
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
//#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>

/*
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>


using namespace rsb;

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

#include <converter/vecIntConverter/main.hpp>
//#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;
*/
using namespace std;
/*
#include <Types.h>

#include <types/twbTracking.pb.h>
*/

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

//using namespace rsb;
//using namespace rsb::patterns;


// margins
int obstacleMargin = 100;
int obstacleMarginSide = 7500;
int groundMargin = 30;

/* Offsets for AMiRo 36 */
static int IR_OFFSETS[] = {2407, 2451, 2486, 2502, 2472, 2442, 2494, 2324};




void splitString(const std::string &str, vector<std::string> &parts, const std::string delimiters) {
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos) {
        // Found a token, add it to the vector.
        parts.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN) {
  
  CAN.setTargetSpeed(speed, angle);
  DEBUG_MSG( "v: " << speed << "w: " << angle);
}

void configRIR(bool loadOffsets) {
  if (loadOffsets) {
    char input[100];
    FILE *irConfig = fopen("irConfig.conf", "r");
    if (irConfig) {
      fgets(input, 100, irConfig);
      INFO_MSG("Read 'irConfig.conf':");
      vector<std::string> parts;
      splitString(std::string(input), parts, "\t");
      for (int part=0; part<parts.size(); part++) {
        IR_OFFSETS[part] = atoi(std::string(parts[part]).c_str());
        INFO_MSG(" " << (part+1) << ") " << IR_OFFSETS[part]);
      }
    } else {
      WARNING_MSG("Coudn't load 'irConfig.conf'! Now using standard offsets.");
    }
  }
}

int doRIR(std::vector<uint16_t> &obstacleValues, std::vector<uint16_t> &groundValues, ControllerAreaNetwork &CAN) {
  std::vector<uint16_t> proximityRingValue(8,0);
  int fail = CAN.getProximityRingValue(proximityRingValue);

  if (fail == 0) {

    // claculate offsets in proximity values
    for (int sensorIdx = 0; sensorIdx < 8; sensorIdx++) {

      // calculate obstacle values
      if (proximityRingValue[sensorIdx] <= IR_OFFSETS[sensorIdx]) {
        obstacleValues[sensorIdx] = 0;
      } else {
        obstacleValues[sensorIdx] = proximityRingValue[sensorIdx] - IR_OFFSETS[sensorIdx];
      }

      // calculate ground values
      if (proximityRingValue[sensorIdx] >= IR_OFFSETS[sensorIdx]) {
        groundValues[sensorIdx] = 0;
      } else {
        groundValues[sensorIdx] = IR_OFFSETS[sensorIdx] - proximityRingValue[sensorIdx];
      }

      if (sensorIdx >= 2 && sensorIdx <= 5) {
        int ledIdx = sensorIdx+4;
        if (ledIdx >= 8) ledIdx-=8;
        if (groundValues[sensorIdx] > groundMargin) {
          CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::RED));
        } else if (obstacleValues[sensorIdx] > obstacleMargin) {
          CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::BLUE));
        } else {
          CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::GREEN));
        }
      }

    }

    return 0;
  } else {
    return 1;
  }
}

int mymcm(int mym) {
  return mym*10000;
}

int main(int argc, char **argv) {

  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
      ("loadOffsets,l", "Loads offset from the file 'irConfig.conf'.");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
    std::cout << options << "\n";
    exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  // Init the CAN interface
  ControllerAreaNetwork CAN;

  // Configure RIR-Reader
  configRIR(vm.count("loadOffsets"));

  for(int led=0; led<8; led++) {
    CAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
  }

  
  int fail = 1;
  uint8_t sensorIdx = 0;
  sendMotorCmd(mymcm(5), 0, CAN);
  bool ok = true;
  bool turn = 0;

  std::vector<uint16_t> obstacleValues(8,0);
  std::vector<uint16_t> groundValues(8,0);

  while(ok) {
    fail = doRIR(obstacleValues, groundValues, CAN);    
    if (fail == 0) {
      for (sensorIdx = 0; sensorIdx < 8; sensorIdx++) {
        INFO_MSG( (int) sensorIdx << ": " << obstacleValues[sensorIdx] << "/" << groundValues[sensorIdx]);
      }
      int valueLeft = obstacleValues[3];
      int valueRight = obstacleValues[4];
      int valueLeftM = groundValues[3];
      int valueRightM = groundValues[4];
      int valueSide = max(obstacleValues[2], obstacleValues[5]);
      INFO_MSG("left: " << valueLeft << ", right: " << valueRight << ", side: " << valueSide);
      if (valueLeftM > groundMargin || valueRightM > groundMargin) {
        if (turn == 0 && valueLeftM < valueRightM) {
          turn = 1;
          sendMotorCmd(0, mymcm(60), CAN);
        } else if (turn == 0) {
          turn = 2;
          sendMotorCmd(0, mymcm(-60), CAN);
        }
      } else if (valueLeft > obstacleMargin || valueRight > obstacleMargin || valueSide > obstacleMarginSide) {
        if (turn == 0 && valueLeft < valueRight && valueLeftM <= 0) {
          turn = 1;
          sendMotorCmd(0, mymcm(60), CAN);
        } else if (turn == 0) {
          turn = 2;
          sendMotorCmd(0, mymcm(-60), CAN);
        }
      } else {
        turn = 0;
        sendMotorCmd(mymcm(10), 0, CAN);
      }
    } else {
      WARNING_MSG("Fail");
      ok = false;
    }
  }
  sendMotorCmd(0,0,CAN);

  return EXIT_SUCCESS;
}
