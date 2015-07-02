//============================================================================
// Name        : main.cxx
// Author      : mbarther
// Description : -
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

using namespace rsb;
using namespace rsb::patterns;


// margins
#define OBSTACLE_MARGIN 100
#define OBSTACLE_MARGIN_SIDE 7500
#define GROUND_MARGIN 6
#define GROUND_MARGIN_DANGER 4
#define EDGE_DIFF 0.4

// edge model 6cm
#define EDGEMODEL_M 6.359946153158588
#define EDGEMODEL_B 0.401918238192352

// velocities
#define VEL_FORWARD 8
#define VEL_FORWARD_SLOW 5
#define VEL_TURNING 40
#define VEL_TURNING_SLOW 25


// scopenames for rsb
std::string proxSensorInscopeObstacle = "/rir_prox/obstacle";
std::string proxSensorInscopeGround = "/rir_prox/ground";

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

enum stateType {
	STturnEdge,
	STfindDirection,
	STfirstEdge,
	STturn,
	STcheckTurn,
	STsecondEdge,
	STfinalize
};

std::string stateTypeString[] {
	"turn ortho to edge",
	"find direction",
	"driving first edge",
	"turn",
	"check turn",
	"driving second edge",
	"finalize"
};


float edgeDist(int senValue) {
  return EDGEMODEL_M * ((float)senValue)/10000.0 + EDGEMODEL_B;
}

void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN) {
  
  CAN.setTargetSpeed(speed, angle);
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

  INFO_MSG("Initialize RSB");

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();

  // ------------ Converters ----------------------

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

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
  int oldValues[2];

  stateType state = STturnEdge;
  stateType stateL = STturn;

  int counter = 0;
  while(ok) {
    if (!proxQueueObstacle->empty() && !proxQueueGround->empty()) {
      counter = 0;

      // Read the proximity data
      boost::shared_ptr<std::vector<int>> sensorValuesObstacle = boost::static_pointer_cast<std::vector<int>>(proxQueueObstacle->pop());
      boost::shared_ptr<std::vector<int>> sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());

      for (sensorIdx = 0; sensorIdx < 8; sensorIdx++) {
//        INFO_MSG( (int) sensorIdx << ": " << sensorValuesObstacle->at(sensorIdx) << "/" << sensorValuesGround->at(sensorIdx));

        float senDist = edgeDist(sensorValuesGround->at(sensorIdx));
        int led = sensorIdx+4;
        if (led >= 8) led -= 8;
        if (senDist < GROUND_MARGIN_DANGER) {
          if (setColors[sensorIdx] != red) {
//            INFO_MSG(" -> switch color from " << colorTypeString[setColors[sensorIdx]] << " to red.");
            CAN.setLightColor(led, amiro::Color(amiro::Color::RED));
            setColors[sensorIdx] = red;
          }
        } else if (senDist < GROUND_MARGIN) {
          if (setColors[sensorIdx] != orange) {
//            INFO_MSG(" -> switch color from " << colorTypeString[setColors[sensorIdx]] << " to orange.");
            CAN.setLightColor(led, amiro::Color(amiro::Color::ORANGE));
            setColors[sensorIdx] = orange;
          }
        } else if (sensorValuesObstacle->at(sensorIdx) > OBSTACLE_MARGIN_SIDE) {
          if (setColors[sensorIdx] != white) {
//            INFO_MSG(" -> switch color from " << colorTypeString[setColors[sensorIdx]] << " to white.");
            CAN.setLightColor(led, amiro::Color(amiro::Color::WHITE));
            setColors[sensorIdx] = white;
          }
        } else if (sensorValuesObstacle->at(sensorIdx) > OBSTACLE_MARGIN) {
          if (setColors[sensorIdx] != blue) {
//            INFO_MSG(" -> switch color from " << colorTypeString[setColors[sensorIdx]] << " to blue.");
            CAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
            setColors[sensorIdx] = blue;
          }
        } else if (setColors[sensorIdx] != green) {
//          INFO_MSG(" -> switch color from " << colorTypeString[setColors[sensorIdx]] << " to green.");
          CAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
          setColors[sensorIdx] = green;
        }
      }

      if (state != stateL) {
        INFO_MSG("Switched to new state '" << stateTypeString[state] << "'");
        stateL = state;
      }
      float edgeDistL, edgeDistR;
      int minEdgeIdx = 0;
      float minEdgeDist = 10;
      switch (state) {
        case STturnEdge:
          for (int senIdx=0; senIdx<8; senIdx++) {
            float ed = edgeDist(sensorValuesGround->at(senIdx));
            if (ed < minEdgeDist) {
              minEdgeIdx = senIdx;
              minEdgeDist = ed;
            }
          }
          if (turn == 0 && minEdgeIdx < 7 && minEdgeIdx > 3) {
            turn = 1;
            sendMotorCmd(0, mymcm(VEL_TURNING_SLOW), CAN);
          } else if (turn == 0 && minEdgeIdx > 0 && minEdgeIdx < 4) {
            turn = 2;
            sendMotorCmd(0, mymcm(-VEL_TURNING_SLOW), CAN);
          }
          state = STfindDirection;
          break;
        case STfindDirection:
          edgeDistL = edgeDist(sensorValuesGround->at(7));
          edgeDistR = edgeDist(sensorValuesGround->at(0));
          if (turn == 0 && edgeDistL < edgeDistR - EDGE_DIFF) {
            turn = 1;
            sendMotorCmd(0, mymcm(VEL_TURNING_SLOW), CAN);
          } else if (turn == 0 && edgeDistR < edgeDistL - EDGE_DIFF) {
            turn = 2;
            sendMotorCmd(0, mymcm(-VEL_TURNING_SLOW), CAN);
          } else if (abs(edgeDistR-edgeDistL) <= EDGE_DIFF && edgeDistR < GROUND_MARGIN && edgeDistL < GROUND_MARGIN) {
            turn = 0;
            sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
            state = STfirstEdge;
          }
          break;
        case STfirstEdge:
          edgeDistL = edgeDist(sensorValuesGround->at(3));
          edgeDistR = edgeDist(sensorValuesGround->at(4));
          if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
            turn = 2;
            sendMotorCmd(0, mymcm(-VEL_TURNING), CAN);
            state = STturn;
//          } else if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
//            sendMotorCmd(mymcm(VEL_FORWARD_SLOW), 0, CAN);
          }
          break;
        case STturn:
          edgeDistL = edgeDist(sensorValuesGround->at(1));
          edgeDistR = edgeDist(sensorValuesGround->at(2));
          if (edgeDistL < GROUND_MARGIN && edgeDistR < GROUND_MARGIN && abs(edgeDistR-edgeDistL) <= EDGE_DIFF) {
            turn = 0;
//            sendMotorCmd(0, 0, CAN);
//            oldValues[0] = sensorValuesGround->at(1);
//            oldValues[1] = sensorValuesGround->at(2);
//            state = STcheckTurn;
            sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
            state = STsecondEdge;
          } else if (edgeDistL < GROUND_MARGIN) {
//            sendMotorCmd(0, mymcm(-VEL_TURNING_SLOW), CAN);
          }
          break;
        case STcheckTurn:
          if (oldValues[0] != sensorValuesGround->at(1) || oldValues[1] != sensorValuesGround->at(2)) {
            edgeDistL = edgeDist(sensorValuesGround->at(1));
            edgeDistR = edgeDist(sensorValuesGround->at(2));
            if (edgeDistL < GROUND_MARGIN && edgeDistR < GROUND_MARGIN && abs(edgeDistR-edgeDistL) <= EDGE_DIFF) {
              turn = 0;
              sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
              state = STsecondEdge;
            } else {
              if (edgeDistR < edgeDistL - EDGE_DIFF) {
                turn = 2;
                sendMotorCmd(0, mymcm(-VEL_TURNING_SLOW), CAN);
              } else {
                turn = 1;
                sendMotorCmd(0, mymcm(VEL_TURNING_SLOW), CAN);
              }
              state = STturn;
            }
          } else {
            WARNING_MSG("Waiting for new values");
          }
          break;
        case STsecondEdge:
          edgeDistL = edgeDist(sensorValuesGround->at(3));
          edgeDistR = edgeDist(sensorValuesGround->at(4));
          if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
            sendMotorCmd(0, 0, CAN);
            state = STfinalize;
//          } else if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
//            sendMotorCmd(mymcm(VEL_FORWARD_SLOW), 0, CAN);
          }
          break;
        case STfinalize:
          sendMotorCmd(0, 0, CAN);
          INFO_MSG("All steps done.");
          for(int led=0; led<8; led++) {
            CAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
          }
          return 0;
          break;
        default:
          WARNING_MSG("Unknown state!");
          return -1;
      }

/*
      int valueLeft = sensorValuesObstacle->at(3);
      int valueRight = sensorValuesObstacle->at(4);
      int valueSide = max(sensorValuesObstacle->at(2), sensorValuesObstacle->at(5));
      float distLeft = edgeDist(sensorValuesGround->at(3));
      float distRight = edgeDist(sensorValuesGround->at(4));
      INFO_MSG("left: " << valueLeft << ", right: " << valueRight << ", side: " << valueSide);
      if (distLeft < GROUND_MARGIN || distRight < GROUND_MARGIN) {
        if (turn == 0 && distLeft > distRight) {
          turn = 1;
          sendMotorCmd(0, mymcm(60), CAN);
        } else if (turn == 0) {
          turn = 2;
          sendMotorCmd(0, mymcm(-60), CAN);
        }
      } else if (valueLeft > OBSTACLE_MARGIN || valueRight > OBSTACLE_MARGIN || valueSide > OBSTACLE_MARGIN_SIDE) {
        if (turn == 0 && valueLeft < valueRight) {
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
*/


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
