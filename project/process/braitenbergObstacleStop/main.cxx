//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Performs a basic braitenberg-like edge avoidanve and stops
//               in front of obstacles. Gets sensor values from RIR-Reader
//               seperated in obstacle and edge values. It uses the obstacle
//               and edge avoidence behavior. Steering and light commands are
//               given via CAN.
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
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

using namespace rsb;

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;
using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <sensorModels/VCNL4020Models.h>

using namespace rsb;
using namespace rsb::converter;
using namespace rsb::util;
using namespace rsb::patterns;


// margins
float OBSTACLE_MARGIN = 0.15;
#define OBSTACLE_MARGIN_DANGER 0.05
#define GROUND_MARGIN 0.06
#define GROUND_MARGIN_DANGER 0.03

// speeds
int driveSpeed = 8; // cm/s

// Offsets for AMiRo 36 
static int GROUND_OFFSETS[] = {2437, 2463, 2483, 2496, 2457, 2443, 2508, 2352};
static int AIR_OFFSETS[] = {2213, 2316, 2341, 2329, 2331, 2290, 2335, 2152};

// scopenames for rsb
std::string proxSensorInscopeObstacle = "/rir_prox/obstacle";
std::string proxSensorInscopeGround = "/rir_prox/ground";
std::string obstacleRecognitionOutscope = "/frontObject/command";

// protocol defines
std::string COMMAND_START = "START";
std::string COMMAND_STOP = "STOP";
std::string COMMAND_QUIT = "QUIT";

// show colors
bool showColors = false;
bool stopDrive = false;

enum colorType {
	black,
	orange,
	red,
	green,
        white,
	blue
};

std::string colorTypeString[] = {
	"black",
	"orange",
	"red",
	"green",
	"white",
	"blue"
};


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
  
  if (!stopDrive) {
    CAN.setTargetSpeed(speed, angle);
  }
  DEBUG_MSG( "v: " << speed << "w: " << angle);
}

int mymcm(int mym) {
  return mym*10000;
}

int main(int argc, char **argv) {

  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
      ("speed,s", po::value<int>(&driveSpeed), "Drive speed in cm/s (default: 8 cm/s).")
      ("obstacleMargin,o", po::value<float>(&OBSTACLE_MARGIN), "Margin for obstacle distance in meters (default: 0.15 m).")
      ("dontDrive,d", "The motor commands won't be sent.")
      ("showColors,c", "Shows measured environment with LEDs.");

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

  showColors = vm.count("showColors");
  stopDrive = vm.count("dontDrive");

  INFO_MSG("Initialize RSB");

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();

  // ------------ Converters ----------------------

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

  // ------------ Informer ------------------------

  // Create obstacle recognition
  Informer<std::string>::Ptr obstacleRecognition = getFactory().createInformer<std::string> (Scope(obstacleRecognitionOutscope));

  // ------------ Listener ----------------------

  // prepare RSB listener for the IR data
  rsb::ListenerPtr proxListenerObstacle = factory.createListener(proxSensorInscopeObstacle);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueObstacle(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
  proxListenerObstacle->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueObstacle)));

  rsb::ListenerPtr proxListenerGround = factory.createListener(proxSensorInscopeGround);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueGround(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
  proxListenerGround->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueGround)));

  // Init the CAN interface
  ControllerAreaNetwork CAN;

  colorType setColors[8];
  for(int led=0; led<8; led++) {
    CAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
    setColors[led] = black;
  }

  
  uint8_t sensorIdx = 0;
  bool ok = true;
  bool turn = 0;
  bool interrupted = false;

  int counter = 0;
  while(ok) {
    if (!proxQueueObstacle->empty() && !proxQueueGround->empty()) {
      counter = 0;

      // Read the proximity data
      boost::shared_ptr<std::vector<int>> sensorValuesObstacle = boost::static_pointer_cast<std::vector<int>>(proxQueueObstacle->pop());
      boost::shared_ptr<std::vector<int>> sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());

      // get obstacle distance
      float valueLeft = VCNL4020Models::obstacleModel(0, sensorValuesObstacle->at(3));
      float valueRight = VCNL4020Models::obstacleModel(0, sensorValuesObstacle->at(4));

      // get edge distance
      float distLeft = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
      float distRight = VCNL4020Models::edgeModel(sensorValuesGround->at(4));

      // set colors
      if (showColors) {
        for (sensorIdx = 0; sensorIdx < 8; sensorIdx++) {
          float obstacleDist = VCNL4020Models::obstacleModel(0, sensorValuesObstacle->at(sensorIdx));
          float edgeDist = VCNL4020Models::edgeModel(sensorValuesGround->at(sensorIdx));
          int led = sensorIdx+4;
          if (led >= 8) led -= 8;
          if (edgeDist < GROUND_MARGIN_DANGER) {
            if (setColors[sensorIdx] != red) {
              CAN.setLightColor(led, amiro::Color(amiro::Color::RED));
              setColors[sensorIdx] = red;
            }
          } else if (edgeDist < GROUND_MARGIN) {
            if (setColors[sensorIdx] != orange) {
              CAN.setLightColor(led, amiro::Color(amiro::Color::ORANGE));
              setColors[sensorIdx] = orange;
            }
          } else if (obstacleDist < OBSTACLE_MARGIN_DANGER) {
            if (setColors[sensorIdx] != white) {
              CAN.setLightColor(led, amiro::Color(amiro::Color::WHITE));
              setColors[sensorIdx] = white;
            }
          } else if (obstacleDist < OBSTACLE_MARGIN) {
            if (setColors[sensorIdx] != blue) {
              CAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
              setColors[sensorIdx] = blue;
            }
          } else if (setColors[sensorIdx] != green) {
            CAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
            setColors[sensorIdx] = green;
          }
        }
      }

      // calculate movement
      if (distLeft < GROUND_MARGIN || distRight < GROUND_MARGIN) {
        if (turn == 0 && distLeft > distRight || (interrupted && turn == 1)) {
          turn = 1;
          sendMotorCmd(0, mymcm(60), CAN);
        } else if (turn == 0 || (interrupted && turn == 2)) {
          turn = 2;
          sendMotorCmd(0, mymcm(-60), CAN);
        }
        interrupted = false;
      } else if (valueLeft < OBSTACLE_MARGIN || valueRight < OBSTACLE_MARGIN) {
        sendMotorCmd(0, mymcm(0), CAN);
        interrupted = true;
      } else {
        if (turn != 0) {
          turn = 0;
        }
        usleep(500000);
        sendMotorCmd(mymcm(driveSpeed), 0, CAN);
        interrupted = false;
      }

      // send command
      std::string command;
      if (interrupted) {
        command = COMMAND_START;
      } else {
        command = COMMAND_STOP;
      }
      boost::shared_ptr<std::string> StringPtr(new std::string(command));
      obstacleRecognition->publish(StringPtr);

    } else if (counter < 4) {
      counter++;
      usleep(50000);
    } else {
      sendMotorCmd(0, 0, CAN);
      WARNING_MSG("Didn't received any sensor data for more than 200 ms. Just stopping!");
      for(int led=0; led<8; led++) {
        CAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
      }
      return -1;
    }
  }

  return EXIT_SUCCESS;
}
