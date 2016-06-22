//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : -
//============================================================================

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

#include <Eigen/Geometry>

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
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>
#include <types/PoseEuler.pb.h>

#include <ControllerAreaNetwork.h>
int canOffset = (int)(0.125 * 1000000.0);
#include <sensorModels/VCNL4020Models.h>

using namespace rsb;
using namespace rsb::patterns;

// robot velocities
float velForward = 0.08; // m/s
float velAngleDegree = 12.0; // degree/s
float velAngle = velAngleDegree * M_PI/180.0; // rad/s


float amiroRadius = 0.05; // m
float maxDetectionDist = 0.4; // m


// scopenames for rsb
std::string fetchTaskInscope = "/fetchObject/status";
std::string fetchTaskOutscope = "/fetchObject/command";
std::string stationDetectionInscope = "/stationDetection/detected";
std::string stationDetectionOutscope = "/stationDetection/command";
std::string proxInscope = "/rir_prox/obstacle";

// string publisher
boost::shared_ptr<std::string> stringPublisher(new std::string);

// station detection constants
float stationDetectionOffset = 6.0; // s


std::string remoteHost = "localhost";
std::string remotePort = "4823";


// method prototypes
int driveCurve(float dist, float angle, float driveSpeed, ControllerAreaNetwork &CAN);
void sendMotorCmd(float speed, float angle, ControllerAreaNetwork &CAN);
float normAngle(float angle);


int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("fetchTaskInscope", po::value<std::string>(&fetchTaskInscope), "Inscope for fetching task status messages.")
			("fetchTaskOutscope", po::value<std::string>(&fetchTaskOutscope), "Outscope for fetching task commands.")
			("stationDetectionInscope", po::value<std::string>(&stationDetectionInscope), "Inscope for station detection position messages.")
			("stationDetectionOutscope", po::value<std::string>(&stationDetectionOutscope), "Outscope for station detection commands.")
			("forwardSpeed,f", po::value<float>(&velForward), "General forward velocity in m/s (default: 0.08).")
			("angleSpeed,a", po::value<float>(&velAngleDegree), "General angular velocity in degree/s (default: 12.0).")
			("stationDetectionOffset,o", po::value<float>(&stationDetectionOffset), "Offset of station detection update in s (default: 6.0).")
			("maxDetectionDist,d", po::value<float>(&maxDetectionDist), "Maximal detection distance in m (default: 0.4).");

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

	velAngle = velAngleDegree * M_PI/180.0;


	// print scope names
	INFO_MSG("Scope names:");
	INFO_MSG(" -> Fetching Task:");
	INFO_MSG("    - Receiving status:   " << fetchTaskInscope);
	INFO_MSG("    - Sending commands:   " << fetchTaskOutscope);
	INFO_MSG(" -> Station Detection:");
	INFO_MSG("    - Receiving position: " << stationDetectionInscope);
	INFO_MSG("    - Sending commands:   " << stationDetectionOutscope);
//	INFO_MSG(" -> Proximity Sensor Values:");
//	INFO_MSG("    - Receiving data:     " << proxInscope);
	INFO_MSG("");

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

/*
  //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
  ///////////////////////////////////////////////////////////////////////////////
  // Get the global participant config as a template
  rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
        {
          // disable socket transport
          rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();
          tmpPropSocket["enabled"] = boost::any(std::string("0"));

          // Get the options for spread transport, because we want to change them
          rsc::runtime::Properties tmpPropSpread  = tmpPartConf.mutableTransport("spread").getOptions();

          // enable socket transport
          tmpPropSpread["enabled"] = boost::any(std::string("1"));

          // Change the config
          tmpPropSpread["host"] = boost::any(std::string(remoteHost));

          // Change the Port
          tmpPropSpread["port"] = boost::any(std::string(remotePort));

          // Write the tranport properties back to the participant config
          tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
          tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
        }
  ///////////////////////////////////////////////////////////////////////////////
*/

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
	converterRepository<std::string>()->registerConverter(converter);

	// ------------ Listener ----------------------

	// Create listener for detections
	rsb::ListenerPtr detectListener = factory.createListener(stationDetectionInscope);
	boost::shared_ptr < rsc::threading::SynchronizedQueue < boost::shared_ptr< std::vector<int> > > >detectQueue(new rsc::threading::SynchronizedQueue< boost::shared_ptr< std::vector<int> > >(1));
	detectListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler< std::vector<int> >(detectQueue)));

	// Create listener for fetching object status messages
	rsb::ListenerPtr fetchingListener = factory.createListener(fetchTaskInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>fetchingQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	fetchingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(fetchingQueue)));

/*	// Create listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));
*/

	// ---------------- Informer ---------------------

	// Create informer for station detection commands
	rsb::Informer<std::string>::Ptr stationDetectionInformer = factory.createInformer< std::string > (stationDetectionOutscope);

	// Create informer for fetching object commands
        rsb::Informer<std::string>::Ptr fetchingInformer = factory.createInformer< std::string > (fetchTaskOutscope);



	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	/*
	 * 
         */
        INFO_MSG("Start heading for loading station.");

	bool stationReached = false;
	while (!stationReached) {

		int stationDetectedCount = 0;
		int stationDetectedCountMax = 2;

		float dist = 0.0;
		float angle = 0.0;

		INFO_MSG("Search station.");
		sendMotorCmd(0.0, velAngle, myCAN);

		bool stationFound = false;
		int searchTimeSum = 0;
		while (!stationFound) {
			usleep(100000);
			searchTimeSum += 100000;

			if (!detectQueue->empty()) {
				boost::shared_ptr< std::vector<int> > stationPos = boost::static_pointer_cast< std::vector<int> >(detectQueue->pop());
				// check for station detection
				if (stationPos->at(0) != 0.0) {
					dist = ((float)stationPos->at(0))/1000000.0;
					angle = ((float)stationPos->at(1))/1000000.0;
					stationDetectedCount++;
				} else {
					stationDetectedCount = 0;
				}
			}

			stationFound = stationDetectedCount >= stationDetectedCountMax;
		}

		sendMotorCmd(0.0, 0.0, myCAN);
		INFO_MSG("Station is at " << dist << "m and " << (angle*180.0/M_PI) << "°");

		float stationDetectionOffsetNow = stationDetectionOffset;
		if (stationDetectionOffset*1000000.0 > searchTimeSum) {
			stationDetectionOffsetNow = (float)(searchTimeSum)/1000000.0;
		}
		float turnAngle = velAngle * stationDetectionOffsetNow - angle; // rad
		int turnTime = turnAngle/velAngle*1000000.0 - canOffset/2; // us
		sendMotorCmd(0.0, -velAngle, myCAN);
		usleep(turnTime);
		sendMotorCmd(0.0, 0.0, myCAN);

		INFO_MSG("Drive to station by blob detection.");

		bool lostStation = false;
		while (!stationReached && !lostStation) {
			usleep((int)(stationDetectionOffset*1000000.0));
			bool gotPos = false;
			int breakCounter = 0;
			while (!gotPos && !lostStation) {
				if (!detectQueue->empty()) {
					boost::shared_ptr< std::vector<int> > stationPos = boost::static_pointer_cast< std::vector<int> >(detectQueue->pop());
					if (stationPos->at(0) != 0.0) {
						dist = ((float)stationPos->at(0))/1000000.0;
						angle = ((float)stationPos->at(1))/1000000.0;
						gotPos = true;
					}
				}
				usleep(10000);
				breakCounter++;
				lostStation = breakCounter > 20;
			}
			if (lostStation) break;

			DEBUG_MSG("Station: " << dist << "m and " << angle << "°");

			float driveDist, driveAngle;
			if (dist > maxDetectionDist) {
				driveDist = dist*0.5;
				driveAngle = angle*2.0/3.0;
			} else {
				driveDist = dist;
				driveAngle = angle*0.25;
				stationReached = true;
			}

			int time = driveCurve(driveDist, driveAngle, velForward, myCAN);
			if (time > 0) {
				usleep(time);
			}
			sendMotorCmd(0.0, 0.0, myCAN);
		}

		if (lostStation) {
			WARNING_MSG("Station has been lost!");
			continue;
		}
	}

	INFO_MSG("Arrived at loading station.");

	return EXIT_SUCCESS;
}


float normAngle(float angle) {
	while (angle >= 2*M_PI) {
		angle -= 2*M_PI;
	}
	while (angle < 0) {
		angle += 2*M_PI;
	}
	return angle;
}

int driveCurve(float dist, float angle, float driveSpeed, ControllerAreaNetwork &CAN) {
	if (angle > M_PI) angle -= 2*M_PI;

	float angleBasis = abs(M_PI/2.0 - abs(angle));
	float angleTop = M_PI - 2.0*angleBasis;

	float moveRadius = dist * sin(angleBasis) / sin(angleTop);
	float moveAngle = 2.0*angle;
	float moveDist = 2.0*moveRadius*M_PI * abs(moveAngle)/(2.0*M_PI);
	if (moveDist == 0.0) {
		moveDist = dist;
	}

	// calculate speeds
	float timeAction = moveDist/driveSpeed; // s
	int waitTime = timeAction*1000000.0 - canOffset/2; // us
	float v = driveSpeed; // s
	float w = moveAngle/timeAction; // rad/s

	DEBUG_MSG("  ==> drive for " << moveDist << "m and " << moveAngle << "°");

	// set speed
	if (waitTime > 0) {
		DEBUG_MSG("Driving " << v << " m/s and " << w << " rad/s for " << waitTime << " us.");
		sendMotorCmd(v, w, CAN);
	}
	return waitTime;
}


void sendMotorCmd(float speed, float angle, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed((int)(speed*1000000.0), (int)(angle*1000000.0));
}


