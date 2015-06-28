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
std::string commandInscope = "/explodrive/command";
std::string answerOutscope = "/explodrive/answer";

// rsb messages
std::string cmdStart = "start";
std::string ansFinish = "finished";
std::string ansProblem = "broken";

enum stateType {
  STturnEdge,
  STfindDirection,
  STfirstEdge,
  STturn,
  STsecondEdge,
  STfinalize
};

std::string stateTypeString[] {
  "turn ortho to edge",
  "find direction",
  "driving first edge",
  "turn",
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

  // prepare RSB listener for commands
  rsb::ListenerPtr cmdListener = factory.createListener(commandInscope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> cmdQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  cmdListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(cmdQueue)));

  // prepare RSB informer for answers
  rsb::Informer<std::string>::Ptr informerAnswer = factory.createInformer<std::string> (answerOutscope);

  // Init the CAN interface
  ControllerAreaNetwork CAN;

  boost::shared_ptr<std::string> stringPublisher(new std::string);          
  
  uint8_t sensorIdx = 0;
  bool ok = true;
  bool turn = 0;
  int oldValues[2];

  int counter = 0;
  while(true) {

    // Read command
    std::string cmd = *cmdQueue->pop();

    ok = true;

    stateType state = STturnEdge;
    stateType stateL = STturn;

    while(ok) {
      if (!proxQueueObstacle->empty() && !proxQueueGround->empty()) {
        counter = 0;
      
        // Read the proximity data
        boost::shared_ptr<std::vector<int>> sensorValuesObstacle = boost::static_pointer_cast<std::vector<int>>(proxQueueObstacle->pop());
        boost::shared_ptr<std::vector<int>> sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());

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
            }
            break;
          case STturn:
            edgeDistL = edgeDist(sensorValuesGround->at(1));
            edgeDistR = edgeDist(sensorValuesGround->at(2));
            if (edgeDistL < GROUND_MARGIN && edgeDistR < GROUND_MARGIN && abs(edgeDistR-edgeDistL) <= EDGE_DIFF) {
              turn = 0;
              sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
              state = STsecondEdge;
            }
            break;
          case STsecondEdge:
            edgeDistL = edgeDist(sensorValuesGround->at(3));
            edgeDistR = edgeDist(sensorValuesGround->at(4));
            if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
              sendMotorCmd(0, 0, CAN);
              state = STfinalize;
            }
            break;
          case STfinalize:
            sendMotorCmd(0, 0, CAN);
            INFO_MSG("All steps done.");
            ok = false;
            *stringPublisher = ansFinish;
            informerAnswer->publish(stringPublisher);
            break;
          default:
            WARNING_MSG("Unknown state!");
            *stringPublisher = ansProblem;
            informerAnswer->publish(stringPublisher);
            return -1;
        }

      } else if (counter < 4) {
        counter++;
        usleep(50000);
      } else {
        sendMotorCmd(0, 0, CAN);
        WARNING_MSG("Didn't received any sensor data for more than 200 ms. Just stopping!");
        *stringPublisher = ansProblem;
        informerAnswer->publish(stringPublisher);
        ok = false;
      }
    }
  }

  return EXIT_SUCCESS;
}
