//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : The robot drives forward to the next edge and turns to the
//               right. This procedure is repeated as often as set (on
//               default it's done twice).
//============================================================================

//#define TRACKING
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
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
#include <rsb/Version.h>
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

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <Constants.h>
#include <Color.h>
#include <sensorModels/VCNL4020Models.h>
#include <actModels/lightModel.h>

#include <extspread/extspread.hpp>

using namespace amiro;

using namespace rsb;
using namespace rsb::patterns;


// margins
#define OBSTACLE_MARGIN 100
#define OBSTACLE_MARGIN_SIDE 7500
#define GROUND_MARGIN 0.06
#define GROUND_MARGIN_DANGER 0.05
#define EDGE_DIFF 0.004

// velocities
#define VEL_FORWARD 8
#define VEL_FORWARD_SLOW 4
#define VEL_TURNING 40
#define VEL_TURNING_SLOW 20


// scopenames for rsb
std::string proxSensorInscopeObstacle = "/rir_prox/obstacle";
std::string proxSensorInscopeGround = "/rir_prox/ground";
std::string commandInscopePart1 = "/tobiamiro";
std::string commandInscopePart2 = "/state";
std::string answerOutscopePart1 = "/amiro";
std::string answerOutscopePart2 = "tobi/state";
std::string lightOutscope = "/amiro/lights";

// rsb commands
std::string cmdRestart = "drive";
std::string cmdTransport = "drive";
std::string ansReady = "finish";
std::string ansFetched = "finish";
std::string cmdansRec = "rec";

// velocities
float forwardSpeed = 0.08; // m/s
float turnSpeed = 20.0 * M_PI/180.0; // rad/s

// amiro constants
unsigned int amiroID = 0;

// behavior values
float irDetectionDist = 0.05; // m
float edgeMaxDist = 0.055; // m
float edgeDistVariance = 0.005; // m
float tableEdgeDist = 0.05; // m

std::string spreadhost = "localhost";
std::string spreadport = "4823";

void sendMotorCmd(float speed, float angle, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed((int)(speed*1000000.0), (int)(angle*1000000.0));
}

bool gotIRCommand(boost::shared_ptr<std::vector<int>> sensorValues) {
	float distBefore = VCNL4020Models::obstacleModel(0, sensorValues->at(ringproximity::SENSOR_COUNT-1));
	for (int i=0; i<ringproximity::SENSOR_COUNT; i++) {
		float distCur = VCNL4020Models::obstacleModel(0, sensorValues->at(i));
		if (distCur < irDetectionDist && distBefore < irDetectionDist) {
			return true;
		}
		distBefore = distCur;
	}
	return false;
}

void printIRDist(boost::shared_ptr<std::vector<int>> sensorValues) {
	for (int i=0; i<ringproximity::SENSOR_COUNT; i++) {
		float dist = VCNL4020Models::obstacleModel(0, sensorValues->at(i));
		DEBUG_MSG("Sensor " << i << ": " << dist);
	}
}

bool edgeIsInfront(boost::shared_ptr<std::vector<int>> sensorValues) {
	for (int i=0; i<4; i++) {
		float dist = VCNL4020Models::edgeModel(sensorValues->at(i+2));
		if (dist < edgeMaxDist) {
			return true;
		}
	}
	return false;
}

bool edgeIsDirectBehind(boost::shared_ptr<std::vector<int>> sensorValues) {
	return VCNL4020Models::edgeModel(sensorValues->at(0)) < edgeMaxDist && VCNL4020Models::edgeModel(sensorValues->at(ringproximity::SENSOR_COUNT-1)) < edgeMaxDist;
}

void driveToEdge(ControllerAreaNetwork &myCAN, boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueGround) {
	boost::shared_ptr<std::vector<int>> sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
	bool realEdge = false;
	bool edge = edgeIsInfront(sensorValues);
	while (!realEdge) {
		sendMotorCmd(forwardSpeed, 0.0, myCAN);
		while (!edge) {
			usleep(100000);
			sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
			edge = edgeIsInfront(sensorValues);
		}
		sendMotorCmd(0.0, 0.0, myCAN);
		usleep(200000);
		sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
		realEdge = edgeIsInfront(sensorValues);
		edge = false;
	}
}

void turnToEdge(ControllerAreaNetwork &myCAN, boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueGround) {
	boost::shared_ptr<std::vector<int>> sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
	float distS = VCNL4020Models::edgeModel(sensorValues->at(0));
	float distB = VCNL4020Models::edgeModel(sensorValues->at(ringproximity::SENSOR_COUNT-1));
	if (distS > distB) {
		float c = distB;
		distB = distS;
		distS = c;
	}
	while (!(edgeIsDirectBehind(sensorValues) && distB-distS <= edgeDistVariance)) {
		sendMotorCmd(0.0, turnSpeed, myCAN);
		while (!(edgeIsDirectBehind(sensorValues) && distB-distS <= edgeDistVariance)) {
			usleep(100000);
			sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
			distS = VCNL4020Models::edgeModel(sensorValues->at(0));
			distB = VCNL4020Models::edgeModel(sensorValues->at(ringproximity::SENSOR_COUNT-1));
			if (distS > distB) {
				float c = distB;
				distB = distS;
				distS = c;
			}
		}
		sendMotorCmd(0.0, 0.0, myCAN);
		usleep(300000);
		sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
		distS = VCNL4020Models::edgeModel(sensorValues->at(0));
		distB = VCNL4020Models::edgeModel(sensorValues->at(ringproximity::SENSOR_COUNT-1));
		if (distS > distB) {
			float c = distB;
			distB = distS;
			distS = c;
		}
	}
}

void setLights(int lightType, Color color, int period, rsb::Informer< std::vector<int> >::Ptr informer) {
	std::vector<int> lightCommand = setLight2Vec(lightType, color, period);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightCommand.begin(),lightCommand.end()));
	informer->publish(commandVector);
}

void waitForCommand(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueObstacle, boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> cmdQueue, rsb::Informer<std::string>::Ptr informerCmd, std::string expCmd, std::string statusCmd) {
	bool gotCommand = false;
	int commandCounter = 0;
	boost::shared_ptr<std::vector<int>> sensorValues;
	boost::shared_ptr<std::string> stringPublisher(new std::string);
	do {
		usleep(200000);
		*stringPublisher = statusCmd;
		informerCmd->publish(stringPublisher);
		sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueObstacle->pop());
		gotCommand = gotIRCommand(sensorValues);
		if (gotCommand) {
			commandCounter++;
		} else {
			commandCounter = 0;
		}
		if (!cmdQueue->empty()) {
			std::string command(*cmdQueue->pop());
			if (command == expCmd) {
				commandCounter = 3;
			}
		}
	} while (commandCounter < 3);
	std::string ans = expCmd + cmdansRec;
	*stringPublisher = ans;
	informerCmd->publish(stringPublisher);
}

int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	float turnSpeedS = 20.0;

	std::string commandInscope, answerOutscope;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
		("id", po::value<unsigned int>(&amiroID), "AMiRo ID (default: 0).")
		("proxObstacleInscope,o", po::value<std::string>(&proxSensorInscopeObstacle), "Inscope for receiving proximity sensor values for obstacle model.")
		("proxGroundInscope,g", po::value<std::string>(&proxSensorInscopeGround), "Inscope for receiving proximity sensor values for edge model.")
		("commandInscope,c", po::value<std::string>(&commandInscope), "Inscope for receiving commands.")
		("answerOutscope,a", po::value<std::string>(&answerOutscope), "Outscope for sending command answers and status messages.")
		("lightOutscope,l", po::value<std::string>(&lightOutscope), "Outscope for light commands.")
		("forwardSpeed,f", po::value<float>(&forwardSpeed), "Forward speed in m/s (default: 0.08).")
		("turnSpeed,t", po::value<float>(&turnSpeedS), "Angular speed in degree/s (default: 20.0).")
		("irDetectionDist,i", po::value<float>(&irDetectionDist), "Maximal distance for command detection by the proximity sensors in m (default: 0.05).")
		("edgeMaxDist,d", po::value<float>(&edgeMaxDist), "Distance for edge detection in m (default: 0.055).")
		("edgeDistVariance,v", po::value<float>(&edgeDistVariance), "Maximal variance between the proximity sensors for edge orientation in m (default: 0.005).")
		("tableEdgeDistance,e", po::value<float>(&tableEdgeDist), "Distance between robot and table edge for grasping and setting objects onto the robot in m (default: 0.05).")
		("host", po::value<std::string>(&spreadhost), "Host of external spread (default: localhost).")
		("port", po::value<std::string>(&spreadport), "Port of external spread (default: 4823).");

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

	if (!vm.count("commandInscope")) commandInscope = commandInscopePart1 + std::to_string(amiroID) + commandInscopePart2;
	if (!vm.count("answerOutscope")) answerOutscope = answerOutscopePart1 + std::to_string(amiroID) + answerOutscopePart2;

	turnSpeed = turnSpeedS * M_PI/180.0;

	INFO_MSG("Initialize RSB:");
	INFO_MSG(" - Proximity Sensor Inscopes:");
	INFO_MSG("    -> Obstacle Model:     " << proxSensorInscopeObstacle);
	INFO_MSG("    -> Edge Model:         " << proxSensorInscopeGround);
	INFO_MSG(" - Command Communication:");
	INFO_MSG("    -> Incomming commands: " << commandInscope);
	INFO_MSG("    -> Sending answers:    " << answerOutscope);
	INFO_MSG(" - Lights:");
	INFO_MSG("    -> Light Command:      " << lightOutscope);
	INFO_MSG("");

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ------------ Listener ----------------------

	// prepare RSB listener for the IR data (obstacles)
	rsb::ListenerPtr proxListenerObstacle = factory.createListener(proxSensorInscopeObstacle);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueObstacle(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListenerObstacle->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueObstacle)));

	// prepare RSB listener for the IR data (edges)
	rsb::ListenerPtr proxListenerGround = factory.createListener(proxSensorInscopeGround);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueGround(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListenerGround->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueGround)));

	// prepare RSB listener for commands
	rsb::ListenerPtr cmdListener = factory.createListener(commandInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> cmdQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	cmdListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(cmdQueue)));

	// ------------ Informer ----------------------

	// prepare RSB informer for answers
	rsb::Informer<std::string>::Ptr informerAnswer = factory.createInformer<std::string> (answerOutscope, extspreadconfig);

	// prepare RSB informer for lights
	rsb::Informer< std::vector<int> >::Ptr informerLights = factory.createInformer< std::vector<int> > (lightOutscope);



	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	boost::shared_ptr<std::string> stringPublisher(new std::string);

	/*
	 * - init start position
	 * - while (doTask)
	 *   - drive forward til table edge
	 *   - turn 180°
	 *   - correct distance to tabel edge
	 *   - wait for drive command
	 *   - drive to start position
	 *   - correct distance to table edge
	 *   - wait for fetch command
	 *   - drive to table edge again
	 *   - turn 180°
	 */

	// set lights
	setLights(LightModel::LightType::SINGLE_CIRCLERIGHT, Color(255,255,0), 0, informerLights);

	// start loop
	while (true) {
		// drive forward
		driveToEdge(myCAN, proxQueueGround);

		// turn to edge
		turnToEdge(myCAN, proxQueueGround);

		// drive a little bit forward
		sendMotorCmd(forwardSpeed/2.0, 0.0, myCAN);
		usleep((int)(tableEdgeDist/(forwardSpeed/2.0)*1000000.0));
		sendMotorCmd(0.0, 0.0, myCAN);

		// set lights
		setLights(LightModel::LightType::SINGLE_SHINE, Color(0,255,0), 0, informerLights);

		// wait for command
		waitForCommand(proxQueueObstacle, cmdQueue, informerAnswer, cmdTransport, ansReady);

		// set lights
		setLights(LightModel::LightType::SINGLE_WARNING, Color(255,255,0), 800, informerLights);

		// wait for some seconds before moving
		sleep(2);

		// drive forward
		driveToEdge(myCAN, proxQueueGround);

		// drive a little bit backward
		sendMotorCmd(-forwardSpeed/2.0, 0.0, myCAN);
		usleep((int)(tableEdgeDist/(forwardSpeed/2.0)*1000000.0));
		sendMotorCmd(0.0, 0.0, myCAN);

		// set lights
		setLights(LightModel::LightType::SINGLE_SHINE, Color(0,255,0), 0, informerLights);

		// wait for command
		waitForCommand(proxQueueObstacle, cmdQueue, informerAnswer, cmdRestart, ansFetched);

		// set lights
		setLights(LightModel::LightType::SINGLE_CIRCLERIGHT, Color(255,255,0), 0, informerLights);

		// wait for some seconds before moving
		sleep(2);

		// drive to edge
		driveToEdge(myCAN, proxQueueGround);

		// turn to edge
		turnToEdge(myCAN, proxQueueGround);
	}



/*
	uint8_t sensorIdx = 0;
	bool ok = true;
	bool turn = 0;
	int oldValues[2];

	int counter = 0;
	while(true) {

    if (!cmdQueue->empty()) {
      // Read command
      std::string cmd = *cmdQueue->pop();

      ok = true;

      stateType state = STturnEdge;
      stateType stateL = STturn;

      edgeNum = 0;

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

          float edgeDistL, edgeDistR, edgeDistFL, edgeDistFR;
          int minEdgeIdx = 0;
          float minEdgeDist = 10;
          int waitingTime_us = 0;
          switch (state) {
            case STturnEdge:
              // Search table edge
              for (int senIdx=0; senIdx<ringproximity::SENSOR_COUNT; senIdx++) {
                float ed = VCNL4020Models::edgeModel(sensorValuesGround->at(senIdx));
                if (ed < minEdgeDist) {
                  minEdgeIdx = senIdx;
                  minEdgeDist = ed;
                }
              }
              state = STfindDirection;
              break;
            case STfindDirection:
              edgeDistL = VCNL4020Models::edgeModel(sensorValuesGround->at(7));
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(0));
                turn = 0;
//                sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
//                state = STdriveEdge;
                sendMotorCmd(0, 0, CAN);
                state = STroundScan;
//              }
              break;
            case STroundScan:
//              waitingTime_us = (int)(((2.0*M_PI*1000.0) / ((float)VEL_TURNING*10.0)) * 1000000.0) - 200000; // us
//              sendMotorCmd(0, mymcm(VEL_TURNING), CAN);
//              usleep(waitingTime_us);
              // Save position
              pose2D = rectPositionsPtr->add_pose();
              pose2D->set_x(0);
              pose2D->set_y(0);
              pose2D->set_orientation(0);
              pose2D->set_id(0);
              sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
              state = STdriveEdge;
            case STdriveEdge:
              edgeDistL = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(4));
              if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
                sendMotorCmd(0, 0, CAN);
                usleep(500000);
                state = STcheckEdge;
              }
              break;
            case STcheckEdge:
              edgeDistL = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(4));
              if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
                INFO_MSG("Edge detected");
                state = STcorrectEdge;
              } else {
                sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
                state = STdriveEdge;
              }
              break;
            case STcorrectEdge:
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
              edgeDistL = edgeDistL*cos(ringproximity::SENSOR_ANGULAR_FRONT_OFFSET);
              edgeDistR = (GROUND_MARGIN_DANGER - edgeDistR) * 10;
              INFO_MSG("Distance to edge: " << edgeDistL << " m (" << edgeDistR << " cm too close) -> Driving with " << VEL_FORWARD_SLOW << " cm/s for " << (int)(edgeDistR/((float)VEL_FORWARD_SLOW)*1000000) << " us");
              if (edgeDistR > 0) {
                sendMotorCmd(mymcm(-VEL_FORWARD_SLOW), 0, CAN);
                usleep((int)(edgeDistR/((float)VEL_FORWARD_SLOW)*1000000));
                sendMotorCmd(0,0,CAN);
              }
              sleep(1);
              if (odomQueue->empty()) {
                WARNING_MSG("No odometry available!");
              }
              odomInput = *odomQueue->pop();
              pose2D = rectPositionsPtr->add_pose();
              pose2D->set_x(odomInput.mutable_translation()->x());
              pose2D->set_y(odomInput.mutable_translation()->y());
              pose2D->set_orientation(odomInput.mutable_rotation()->yaw());
              pose2D->set_id(edgeNum);
              turn = 2;
              sendMotorCmd(0, mymcm(-VEL_TURNING), CAN);
              if (edgeNum >= edgeCount-1) {
                state = STfinalize;
              } else {
                state = STturn;
              }
              edgeNum++;
              DEBUG_MSG("#Edges driven: " << (edgeNum) << " of " << edgeCount);
              DEBUG_MSG("Saved position " << pose2D->x() << "/" << pose2D->y() << " with " << pose2D->orientation() << " rad and ID " << pose2D->id());
              break;
            case STturn:
              edgeDistL = VCNL4020Models::edgeModel(sensorValuesGround->at(1));
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(2));
              edgeDistFL = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
              edgeDistFR = VCNL4020Models::edgeModel(sensorValuesGround->at(4));
              if (edgeDistL < GROUND_MARGIN && edgeDistR < GROUND_MARGIN && abs(edgeDistR-edgeDistL) < EDGE_DIFF
                  && edgeDistFL > GROUND_MARGIN && edgeDistFR > GROUND_MARGIN) {
                turn = 0;
                sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
                state = STdriveEdge;
              }
              break;
            case STfinalize:
              waitingTime_us = (int)(((1.25*M_PI*1000.0) / ((float)VEL_TURNING*10.0)) * 1000000.0); // us
              sendMotorCmd(0, mymcm(VEL_TURNING), CAN);
              usleep(waitingTime_us);
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

    } else if (!positionQueue->empty()) {
      positionQueue->pop();
      positionInformer->publish(rectPositionsPtr);
    } else {
      usleep(500000);
    }
  }

*/

	return EXIT_SUCCESS;
}
