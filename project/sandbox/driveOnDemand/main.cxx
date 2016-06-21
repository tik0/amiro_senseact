//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Driving to position as demanded.
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

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <sensorModels/VCNL4020Models.h>
#include <actModels/lightModel.h>
#include <extspread.hpp>

using namespace rsb;
using namespace rsb::patterns;


float distanceError = 0.01; // m
float maxForwardAngle = 5.0 * M_PI/180.0; //rad

float driveSpeed = 0.08; // m/s

// status data
std::string CMD_STARTED = "started";
std::string CMD_FINISHED = "finished";
std::string CMD_STOPPED = "stopped";

// commands
std::string CMD_STOP = "stop";
std::string CMD_COLOR = "color_";

// color command additions
#define COL_WHITE 'w'
#define COL_RED 'r'
#define COL_BLUE 'b'
#define COL_GREEN 'g'
#define COL_YELLOW 'y'
#define CTP_SHINE 'S'
#define CTP_BLINK 'B'
#define CTP_WARNING 'W'
#define CTP_CIRCLELEFT 'L'
#define CTP_CIRCLERIGHT 'R'


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
void setLights(int lightType, amiro::Color color, int period, rsb::Informer< std::vector<int> >::Ptr informer);
void setColor(std::string colorCommand, rsb::Informer< std::vector<int> >::Ptr lightInformer, rsb::Informer< std::string >::Ptr guideInformer);
void setSteering(boost::shared_ptr<twbTracking::proto::Pose2D> guidePos, boost::shared_ptr<twbTracking::proto::Pose2D> trackPos, ControllerAreaNetwork &CAN);
bool positionReached(boost::shared_ptr<twbTracking::proto::Pose2D> guidePos, boost::shared_ptr<twbTracking::proto::Pose2D> trackPos);
int motorAction(float speed, float turn, ControllerAreaNetwork &CAN);


// main function
int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("trackingScope,t", po::value<std::string>(&trackingInscope), "Inscope for tracking data (default: '/amiro/tracking').")
			("commandScope,c", po::value<std::string>(&CommandInscope), "Inscope for commands (default: '/amiro/command').")
			("guideInscope,i", po::value<std::string>(&GuideInscope), "Inscope for guided position (default: '/amiro/drive/guide').")
			("guideOutscope,o", po::value<std::string>(&GuideOutscope), "Outscope for status updates for the guide (default: '/amiro/drive/status').")
			("lightOutscope,l", po::value<std::string>(&lightOutscope), "Outscope for light commands (default: '/amiro/lights').")
			("lightInscope", po::value<std::string>(&lightInscope), "Inscope for light recognition answers (default: '/amiro/lightsrec').")
			("host,h", po::value<std::string>(&spreadhost), "Host for Programatik Spread (default: '127.0.0.1').")
			("port,p", po::value<std::string>(&spreadport), "Port for Programatik Spread (default: '4803').");

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
	INFO_MSG(" -> Incomming Position: " << GuideInscope);
	INFO_MSG(" -> Sending Status:     " << GuideOutscope);
	INFO_MSG(" -> Tracking Data:      " << trackingInscope);
	INFO_MSG("Internal Spread Scopes:");
	INFO_MSG(" -> Light Commands:     " << lightOutscope);
	INFO_MSG(" -> Light Recognition:  " << lightInscope);
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
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(trackingQueue)));

	// prepare rsb listener for progress data
        rsb::ListenerPtr guideListener = factory.createListener(GuideInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>guideQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	guideListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(guideQueue)));

	// prepare RSB listener for amiros
	rsb::ListenerPtr commandListener = factory.createListener(CommandInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> commandQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(commandQueue)));

	// prepare RSB listener for color change
	rsb::ListenerPtr colorListener = factory.createListener(lightInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> colorQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	colorListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(colorQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish progress data
	rsb::Informer<std::string>::Ptr guideInformer = factory.createInformer<std::string>(GuideOutscope, extspreadconfig);

	// prepare RSB informer for lights
	rsb::Informer< std::vector<int> >::Ptr informerLights = factory.createInformer< std::vector<int> > (lightOutscope);



	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	boost::shared_ptr<twbTracking::proto::Pose2D> guidePos(new twbTracking::proto::Pose2D());
	boost::shared_ptr<twbTracking::proto::Pose2D> trackPos(new twbTracking::proto::Pose2D());

	bool driving = false;
	int blinkCounter = 0;
	bool exitProg = false;
	bool errorProg = false;

	setLights(lightType, curColor, 0, informerLights);

	INFO_MSG("Starting driving loop");
	while (!exitProg) {
		// wait for new guide command
		while (guideQueue->empty() && !exitProg) {
			// continue driving
			if (driving) {
				bool tracked = false;
				for (int i=0; i<6; i++) {
					if (!trackingQueue->empty()) {
						tracked = true;
						break;
					}
					usleep(50000);
				}
				if (!tracked) {
					motorAction(0, 0, myCAN);
					ERROR_MSG("There wasn't any tracking information for 300 ms!");
					exitProg = true;
					errorProg = true;
					break;
				}
				trackPos = trackingQueue->pop();

				// check if position has been reached				
				if (positionReached(guidePos, trackPos)) {
					motorAction(0, 0, myCAN);
					driving = false;
					boost::shared_ptr<std::string> StringPtr(new std::string(CMD_FINISHED));
					guideInformer->publish(StringPtr);
					INFO_MSG("Position " << guidePos->x() << "/" << guidePos->y() << " [m] reached.");
				} else {
					setSteering(guidePos, trackPos, myCAN);
				}
			}

			// check for new color input
			if (!commandQueue->empty()) {
				EventPtr event = commandQueue->pop(0);
				std::string command = *static_pointer_cast<std::string>(event->getData());
				DEBUG_MSG("Incomming command: " << command);
				if (command == CMD_STOP) {
					exitProg = true;
				} else if (command.substr(0, CMD_COLOR.length()) == CMD_COLOR) {
					setColor(command, informerLights, guideInformer);
				}
			}

			if (!colorQueue->empty()) {
				EventPtr event = colorQueue->pop(0);
				std::string rec = *static_pointer_cast<std::string>(event->getData());
				boost::shared_ptr<std::string> StringPtr(new std::string("color_" + rec));
				guideInformer->publish(StringPtr);
			}

			// sleep 100 ms
			usleep(100000);
		}
		if (!exitProg) {
			// get new guided position and start driving
			guidePos = guideQueue->pop();
			driving = true;
			boost::shared_ptr<std::string> StringPtr(new std::string(CMD_STARTED));
			guideInformer->publish(StringPtr);
			INFO_MSG("Start driving to position " << guidePos->x() << "/" << guidePos->y() << " [m].");
		}
	}

	// stop motors
	motorAction(0, 0, myCAN);

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

void setLights(int lightType, amiro::Color color, int period, rsb::Informer< std::vector<int> >::Ptr informer) {
	std::vector<int> lightCommand = setLight2Vec(lightType, color, period);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightCommand.begin(),lightCommand.end()));
	informer->publish(commandVector);
}


void setColor(std::string colorCommand, rsb::Informer< std::vector<int> >::Ptr lightInformer, rsb::Informer< std::string >::Ptr guideInformer) {
	std::string colorInput = colorCommand.substr(CMD_COLOR.length());
	// check if there is input
	if (colorInput.length() >= 1) {
		// check color change
		switch (colorInput[0]) {
			case COL_WHITE: curColor.setRedGreenBlue(255, 255, 255); break;
			case COL_RED: curColor.setRedGreenBlue(255, 0, 0); break;
			case COL_BLUE: curColor.setRedGreenBlue(0, 0, 255); break;
			case COL_GREEN: curColor.setRedGreenBlue(0, 255, 0); break;
			case COL_YELLOW: curColor.setRedGreenBlue(255, 255, 0); break;
			default: WARNING_MSG("Color shortcut '" << colorInput[0] << "' is unknown!"); break;
		}

		// check light type change
		char lightTypeC = CTP_SHINE;
		if (colorInput.length() >= 2) {
			lightTypeC = colorInput[1];
		}
		int lightTypeCheck = -1;
		switch (lightTypeC) {
			case CTP_SHINE: lightTypeCheck = LightModel::LightType::SINGLE_SHINE; break;
			case CTP_BLINK: lightTypeCheck = LightModel::LightType::SINGLE_BLINK; break;
			case CTP_WARNING: lightTypeCheck = LightModel::LightType::SINGLE_WARNING; break;
			case CTP_CIRCLELEFT: lightTypeCheck = LightModel::LightType::SINGLE_CIRCLELEFT; break;
			case CTP_CIRCLERIGHT: lightTypeCheck = LightModel::LightType::SINGLE_CIRCLERIGHT; break;
			default: WARNING_MSG("Light type '" << colorInput[0] << "' is unknown!"); break;
		}
		if (lightTypeCheck >= 0) {
			lightType = lightTypeCheck;
		}
		int period = 0;
		std::string lightTypeDesc;
		switch (lightType) {
			case LightModel::LightType::SINGLE_SHINE: lightTypeDesc = "shine"; break;
			case LightModel::LightType::SINGLE_BLINK: lightTypeDesc = "blinking normal"; period = 600; break;
			case LightModel::LightType::SINGLE_WARNING: lightTypeDesc = "blinking warning"; period = 800; break;
			case LightModel::LightType::SINGLE_CIRCLELEFT: lightTypeDesc = "circled left"; break;
			case LightModel::LightType::SINGLE_CIRCLERIGHT: lightTypeDesc = "circled right"; break;
			default: break;
		}

		// print color change information
		std::string colorName;
		if (curColor.getRed() == 255 && curColor.getGreen() == 255 && curColor.getBlue() == 255) {
			colorName = "white";
		} else if (curColor.getRed() == 255 && curColor.getGreen() == 255) {
			colorName = "yellow";
		} else if (curColor.getRed() == 255) {
			colorName = "red";
		} else if (curColor.getGreen() == 255) {
			colorName = "green";
		} else {
			colorName = "blue";
		}
		INFO_MSG("Color changed: Color '" << colorName << "', Light Type '" << lightTypeDesc << "'");

		setLights(lightType, curColor, period, lightInformer);
	}
}


void setSteering(boost::shared_ptr<twbTracking::proto::Pose2D> guidePos, boost::shared_ptr<twbTracking::proto::Pose2D> trackPos, ControllerAreaNetwork &CAN) {
	float moveDirX = guidePos->x() - trackPos->x();
	float moveDirY = guidePos->y() - trackPos->y();
	float actAngle = trackPos->orientation();
	while (actAngle >= 2.0*M_PI) actAngle -= 2.0*M_PI;
	while (actAngle < 0) actAngle += 2.0*M_PI;

	float distDir = sqrt(moveDirX*moveDirX + moveDirY*moveDirY);
	float angleDir = atan2(moveDirY, moveDirX) - actAngle;
	while (angleDir >= 2.0*M_PI) angleDir -= 2.0*M_PI;
	while (angleDir < 0) angleDir += 2.0*M_PI;
	if (angleDir > M_PI) angleDir -= 2*M_PI;

	float angleBasis = abs(M_PI/2.0 - abs(angleDir));
	float angleTop = M_PI - 2.0*angleBasis;

	float moveRadius = distDir * sin(angleBasis) / sin(angleTop);
	float moveAngle = 2.0*angleDir;
	float moveDist = 2.0*moveRadius*M_PI * abs(moveAngle)/(2.0*M_PI);
	if (moveAngle == 0.0) {
		moveDist = distDir;
	} else if (moveAngle > maxForwardAngle) {
		moveDist = 0.0;
	}

	// calculate speeds
	if (moveDist > 0.0) {
		float timeAction = moveDist/driveSpeed; // s
		float turnSpeed = moveAngle/timeAction * 1.05; // rad/s
		motorAction(driveSpeed, turnSpeed, CAN);
	} else {
		float turnSpeed = 60.0 * M_PI/180.0; // rad/s
		motorAction(0.0, turnSpeed, CAN);
	}
}


bool positionReached(boost::shared_ptr<twbTracking::proto::Pose2D> guidePos, boost::shared_ptr<twbTracking::proto::Pose2D> trackPos) {
	float xDiff = guidePos->x() - trackPos->x();
	float yDiff = guidePos->y() - trackPos->y();
	float errorDist = sqrt(xDiff*xDiff + yDiff*yDiff);
	return errorDist < distanceError;
}


int motorAction(float speed, float turn, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed((int)(speed*1000000.0), (int)(turn*1000000.0));
	return EXIT_SUCCESS;
}
