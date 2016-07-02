//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Navigation based on RRL.
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
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>
#include <rsb/util/EventQueuePushHandler.h>
#include <rsb/MetaData.h>
#include <rsb/util/QueuePushHandler.h>


using namespace rsb;

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

#include <Types.h>

#include <types/twbTracking.pb.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <sensorModels/VCNL4020Models.h>
#include <actModels/lightModel.h>
#include <extspread.hpp>

using namespace rsb;
using namespace rsb::patterns;

#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
using namespace boost::chrono;
using namespace boost;


float distanceError = 0.01; // m
float maxForwardAngle = 5.0 * M_PI/180.0; //rad

float driveSpeed = 0.08; // m/s

// status data
std::string CMD_STARTED = "started";
std::string CMD_FINISHED = "finished";
std::string CMD_STOPPED = "stopped";


// scopenames and spread data for rsb
std::string trackingInscope = "/amiro/tracking";
std::string GuideOutscope = "/amiro/drive/status";
std::string GuideInscope = "/amiro/drive/guide";
std::string CommandInscope = "/amiro/command";
std::string lightOutscope = "/amiro/lights";
std::string lightInscope = "/amiro/lightsrec";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4803";

// color data and light type
amiro::Color curColor(255, 255, 255);
int lightType = LightModel::LightType::SINGLE_SHINE;


// method prototypes
std::vector<int> polar2Kart(boost::shared_ptr< std::vector<int> > polarPos);
float frontAngle(float angle);
float normAngle(float angle);
void setLights(int lightType, amiro::Color color, int period, rsb::Informer< std::vector<int> >::Ptr informer);
int motorAction(float speed, float turn, ControllerAreaNetwork &CAN);


// main function
int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;
	
	float drivingDist = 0.2; // m
	float forwardSpeed = 0.03; // m/s
	float turnSpeed = 20.0 * M_PI/180.0; // rad/s

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("trackingScope,t", po::value<std::string>(&trackingInscope), "Inscope for tracking data (default: '/amiro/tracking').")
			("commandScope,c", po::value<std::string>(&CommandInscope), "Inscope for commands (default: '/amiro/command').")
			("guideInscope,i", po::value<std::string>(&GuideInscope), "Inscope for guided position (default: '/amiro/drive/guide').")
			("guideOutscope,o", po::value<std::string>(&GuideOutscope), "Outscope for status updates for the guide (default: '/amiro/drive/status').")
			("lightOutscope,l", po::value<std::string>(&lightOutscope), "Outscope for light commands (default: '/amiro/lights').")
			("host,h", po::value<std::string>(&spreadhost), "Host for Programatik Spread (default: '127.0.0.1').")
			("port,p", po::value<std::string>(&spreadport), "Port for Programatik Spread (default: '4803').")
			("speed", po::value<float>(&forwardSpeed), "Distance to drive in m (default: 0.03).")
			("drivingdist", po::value<float>(&drivingDist), "Distance to drive in m (default: 0.2).");

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
	INFO_MSG("");
	INFO_MSG("External Spread Scopes:");
	INFO_MSG(" -> Incomming Commands: " << CommandInscope);
	INFO_MSG(" -> Sending Status:     " << GuideOutscope);
	INFO_MSG(" -> Tracking Data:      " << trackingInscope);
	INFO_MSG("Internal Spread Scopes:");
	INFO_MSG(" -> Light Commands:     " << lightOutscope);
	INFO_MSG("");

	boost::shared_ptr<twbTracking::proto::Pose2D> finalePos(new twbTracking::proto::Pose2D());

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// Register converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// Register new converter for Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterlocalization);

	// ------------ Listener ----------------------

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr< std::vector<int> >>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr< std::vector<int> >>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler< std::vector<int> >(trackingQueue)));
	
	// prepare rsb listener for progress data
        rsb::ListenerPtr guideListener = factory.createListener(GuideInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>guideQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	guideListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(guideQueue)));

	// prepare RSB listener for amiros
	rsb::ListenerPtr commandListener = factory.createListener(CommandInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> commandQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(commandQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish progress data
	rsb::Informer<std::string>::Ptr guideInformer = factory.createInformer<std::string>(GuideOutscope, extspreadconfig);

	// prepare RSB informer for lights
	rsb::Informer< std::vector<int> >::Ptr informerLights = factory.createInformer< std::vector<int> > (lightOutscope);



	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	boost::shared_ptr<twbTracking::proto::Pose2D> guidePos(new twbTracking::proto::Pose2D());

	bool driving = false;
	bool exitProg = false;
	bool errorProg = false;

	setLights(lightType, curColor, 0, informerLights);
	std::vector<int> startPosition(2,0), actPosition(2,0), lastPosition(2,0), goalPosition(2,0);
	float drivenDist = 0.0;
	float robAngleBefore = 0.0;
	
	bool firstPos = true;
	bool positionReached = false;
	float direction = 1.0;
	
	// Initialize clock
	system_clock::time_point systime1, systime2;
	milliseconds mstime;

	INFO_MSG("Starting driving loop");
	while (!positionReached) {
		// check for new command input
		if (!commandQueue->empty()) {
			EventPtr event = commandQueue->pop(0);
			std::string command = *static_pointer_cast<std::string>(event->getData());
			//TODO Check command
		}
		
		// check for new tracking information
		if (!trackingQueue->empty()) {
			// get position
			actPosition = polar2Kart(boost::static_pointer_cast< std::vector<int> >(trackingQueue->pop()));
			if (firstPos) {
				startPosition = actPosition;
				goalPosition[0] = startPosition[0] + int(drivingDist*1000000.0);
				goalPosition[1] = startPosition[1] - int(drivingDist*1000000.0);
				firstPos = false;

				// start motors
				motorAction(forwardSpeed, 0.0, myCAN);
				INFO_MSG("Start driving with " << forwardSpeed << " m/s.");
				
			} else {
				float robAngle;
				if (direction > 0) robAngle = normAngle(atan2(lastPosition[1]-actPosition[1], lastPosition[0]-actPosition[0]));
				else robAngle = normAngle(atan2(actPosition[1]-lastPosition[1], actPosition[0]-lastPosition[0]));
//				robAngle = normAngle(robAngle + (robAngle-robAngleBefore)/2.0);
				robAngleBefore = robAngle;
				DEBUG_MSG("Believed robot angle: " << (robAngle*180.0/M_PI) << "°");
				float goalAngle = frontAngle(atan2(actPosition[1]-goalPosition[1], actPosition[0]-goalPosition[0]) - robAngle);
				DEBUG_MSG("Believed goal angle: " << (goalAngle*180.0/M_PI) << "° (will be ranged in [pi/4, -pi/4])");
				direction = 1.0;
				if (goalAngle < -M_PI/4.0) {
					direction = -1.0;
					goalAngle = -M_PI/4.0;
				} else if (goalAngle > M_PI/4.0) {
					goalAngle = M_PI/4.0;
					direction = -1.0;
				}
				float driveS = direction * forwardSpeed * (1-abs(goalAngle)/(M_PI/2.0));
				float angleS = turnSpeed * goalAngle/(M_PI/2.0);
				motorAction(driveS, angleS, myCAN);
				INFO_MSG("Driving with " << driveS << " m/s and " << angleS << " rad/s.");
			}
			float xDiff = float(goalPosition[0]-actPosition[0]);
			float yDiff = float(goalPosition[1]-actPosition[1]);
			drivenDist = sqrt(xDiff*xDiff + yDiff*yDiff);
			positionReached = drivenDist < 0.05 * 1000000.0;
			DEBUG_MSG("Goal distance: goal " << goalPosition[0] << "/" << goalPosition[1] << ", current " << actPosition[0] << "/" << actPosition[1]);
			DEBUG_MSG("Position reached: " << positionReached << " (" << drivenDist << " < 50000)");

			lastPosition = actPosition;
		}

		// sleep 100 ms
		usleep(100000);
	}
	
	INFO_MSG("Position has been reached");

	// stop motors
	motorAction(0.0, 0.0, myCAN);

	// send stopping recognitions
	boost::shared_ptr<std::string> StringPtr(new std::string(CMD_FINISHED));
	guideInformer->publish(StringPtr);

	// reset lights
	setLights(LightModel::LightType::SINGLE_INIT, curColor, 0, informerLights);

	// exit procedure if error
	if (errorProg) {
		ERROR_MSG("Exit with Errors!");
		return EXIT_FAILURE;
	}

	// normal exit procedure
	INFO_MSG("Exit");
	return EXIT_SUCCESS;
}

std::vector<int> polar2Kart(boost::shared_ptr< std::vector<int> > polarPos) {
	std::vector<int> kart(2,0);
	DEBUG_MSG("Tracking position at " << (float(polarPos->at(0))/1000000.0) << " m and " << (float(polarPos->at(1))/1000000.0 * 180.0/M_PI) << "°");
	float angle = float(polarPos->at(1))/1000000.0;
	float dist = float(polarPos->at(0));
	kart[0] = int(dist * cos(angle));
	kart[1] = int(dist * sin(angle));
	return kart;
}

void setLights(int lightType, amiro::Color color, int period, rsb::Informer< std::vector<int> >::Ptr informer) {
	std::vector<int> lightCommand = setLight2Vec(lightType, color, period);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightCommand.begin(),lightCommand.end()));
	informer->publish(commandVector);
}


float frontAngle(float angle) {
	while (angle < -M_PI) angle += 2.0*M_PI;
	while (angle > M_PI) angle -= 2.0*M_PI;
	return angle;
}

float normAngle(float angle) {
	while (angle < 0) angle += 2.0*M_PI;
	while (angle >= 2.0*M_PI) angle -= 2.0*M_PI;
	return angle;
}


int motorAction(float speed, float turn, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed((int)(speed*1000000.0), (int)(turn*1000000.0));
	return EXIT_SUCCESS;
}
