//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Statemachine to perform a choreography including brakes.
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// std includes
#include <iostream>
using namespace std;

// boost
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
namespace po = boost::program_options;
using namespace boost::chrono;

// rsb
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Handler.h>
#include <rsb/util/EventQueuePushHandler.h>
#include <rsb/MetaData.h>
#include <boost/shared_ptr.hpp>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>
#include <converter/vecIntConverter/main.hpp>
using namespace rsb;
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>
#include <ControllerAreaNetwork.h>
#include <actModels/lightModel.h>

#include "choreo.h"
#include <Types.h>

// unit calculations
#define TO_MILLI 1000.0
#define TO_MICRO 1000000.0
#define MILLI_TO 0.001
#define MICRO_TO 0.000001

// xml constants
std::string XMLMainName = "choreo";
std::string XMLStep = "choreoStep";
std::string XMLInclude = "choreoinclude";
std::string XMLBrake = "brake";
std::string XMLPositionX = "posx";
std::string XMLPositionY = "posy";
std::string XMLLightAll = "la";
std::string XMLLightID = "l"; // + ID in [0,7]
std::string XMLLightSeperator = ",";
std::string XMLIncludeName = "choreopart";

// command constants
std::string COMMAND_INIT = "init";
std::string COMMAND_START = "start";
std::string COMMAND_STOP = "stop";

// initialize CAN
ControllerAreaNetwork myCAN;

// constants
float driveSpeed = 0.08; // m/s
int timebuffer = (int)(0.125 * TO_MICRO); // us
std::string amiroName = "amiro0";
int markerID = 0;
float meterPerPixel = 1.0/400.0; // m/pixel
float trackingVariance = 0.005; // m

// Flags
bool printChoreo = false;
bool relativePosition = false;
bool useOdo = false;
bool useTwb = false;


// scopenames for rsb
std::string lightOutscope = "/amiro/lights";
std::string choreoInscope = "/harvesters/choreo";
std::string commandInscope = "/harvesters/command";
std::string amiroScope = "/harvesters/harvester";
std::string trackingInscope = "/twb/tracking";

types::position odoPos;



types::position readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID) {
	twbTracking::proto::Pose2D pose2D;
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingMarkerID == data->pose(i).id()) {
			pose2D = data->pose(i);
			break;
		}
	}
	types::position pos;
	pos.x = pose2D.x()*meterPerPixel*TO_MICRO;
	pos.y = pose2D.y()*meterPerPixel*TO_MICRO;
	pos.f_z = pose2D.orientation()*TO_MICRO;
	return pos;
}



bool isBeingTracked(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingID) {
	bool isIn = false;
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingID == data->pose(i).id()) {
			if (data->pose(i).x() != 0 && data->pose(i).y() != 0) {
				isIn = true;
			}
			break;
		}
	}
	return isIn;
}

types::position getNextTrackingPos(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>> trackingQueue) {
	boost::shared_ptr<twbTracking::proto::Pose2DList> data;
	do {
		if (!trackingQueue->empty()) {
			data = boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop());
			if (isBeingTracked(data, markerID)) {
				break;
			}
		} else {
			// sleep for 10 ms
			usleep(10000);
		}
	} while (true);
	return readTracking(data, markerID);
}


void calcSpeeds(types::position curPos, float relDistx, float relDisty, float &v, float &w, float &time, float &dist, float &angle) {
	float actAngle = (float)(curPos.f_z)*MICRO_TO;
	while (actAngle >= 2.0*M_PI) actAngle -= 2.0*M_PI;
	while (actAngle < 0) actAngle += 2.0*M_PI;

	float distDir = sqrt(relDistx*relDistx + relDisty*relDisty);
	float angleDir = atan2(relDisty, relDistx) - actAngle;
	while (angleDir >= 2.0*M_PI) angleDir -= 2.0*M_PI;
	while (angleDir < 0) angleDir += 2.0*M_PI;
	if (angleDir > M_PI) angleDir -= 2*M_PI;

	float angleBasis = abs(M_PI/2.0 - abs(angleDir));
	float angleTop = M_PI - 2.0*angleBasis;

	float moveRadius = distDir * sin(angleBasis) / sin(angleTop);
	float moveAngle = 2.0*angleDir;
	float moveDist = 2.0*moveRadius*M_PI * abs(moveAngle)/(2.0*M_PI);
	if (moveDist == 0.0) {
		moveDist = distDir;
	}

	// calculate speeds; // m/s
	time = moveDist/driveSpeed; // s
	v = driveSpeed; // m/s
	w = moveAngle/time; // rad/s
	dist = moveDist; // m
	angle = moveAngle; // rad
}



// load a (sub)choreography from a file
Choreo loadSubChoreo(std::string choreoName, int subChoreoNum, std::vector<std::string> parents) {
	bool loadingCorrect = true;
	bool parentFound = false;
	parents.push_back(choreoName);

	// create spaces for "tabs"
	string spacesP = "";
	for (int i=0; i<subChoreoNum; i++) {
		spacesP += "  ";
	}
	string spacesC = spacesP + "  ";

	if (printChoreo) {
		string frontPart = "";
		if (subChoreoNum > 0) {
			frontPart += spacesP + "-> choreo include: ";
		}
		DEBUG_MSG(frontPart << "Load choreo '" << choreoName << "'");
	}
	Choreo choreo;
	using boost::property_tree::ptree;
	ptree pt;

	try {

		read_xml(choreoName, pt);
		BOOST_FOREACH( ptree::value_type const&tree, pt.get_child(XMLMainName)) {
			if (tree.first == XMLStep) {
				ChoreoStep choreoStep;
				choreoStep.braking = tree.second.get<int>(XMLBrake);
				position_t position;
				position[0] = (float)(tree.second.get<int>(XMLPositionX))*MICRO_TO;
				position[1] = (float)(tree.second.get<int>(XMLPositionY))*MICRO_TO;
				choreoStep.position = position;
				light_t lights;
				std::string lightinput;
				try {
					lightinput = tree.second.get<std::string>(XMLLightAll);
					std::vector<std::string> splitstring;
					splitstring.clear();
					boost::split(splitstring, lightinput, boost::is_any_of(XMLLightSeperator));
					for (int i = 0; i < 8; i++) {
						lights[i][0] = boost::lexical_cast<int>(splitstring[2]);
						lights[i][1] = boost::lexical_cast<int>(splitstring[1]);
						lights[i][2] = boost::lexical_cast<int>(splitstring[0]);
					}
				} catch (...) {
					for (int i = 0; i < 8; i++) {
						std::vector<std::string> splitstring;
						std::string field = XMLLightID + boost::lexical_cast<std::string>(i);
						lightinput = tree.second.get<std::string>(field);
						splitstring.clear();
						boost::split(splitstring, lightinput, boost::is_any_of(XMLLightSeperator));
						lights[i][0] = boost::lexical_cast<int>(splitstring[2]);
						lights[i][1] = boost::lexical_cast<int>(splitstring[1]);
						lights[i][2] = boost::lexical_cast<int>(splitstring[0]);
					}
				}
				choreoStep.lights = lights;
				choreo.push_back(choreoStep);
				if (printChoreo) {
					DEBUG_MSG(spacesC << "-> choreo step: " << position[0] << "/" << position[1] << " [m]");
				}
			} else if (tree.first == XMLInclude) {
				std::string newfile = tree.second.get<std::string>(XMLIncludeName);
				// check for parent name
				parentFound = false;
				for (std::string parentName : parents) {
					if (newfile == parentName) {
						if (printChoreo) {
							ERROR_MSG(spacesC << "-> choreo include:");
							ERROR_MSG(spacesC << "   The choreo '" << newfile << "' is a parent choreo! Loops are not allowed!");
						}
						parentFound = true;
						break;
					}
				}
				// break if parent choreo shall be called and remove choreo elements
				if (parentFound) {
					choreo.clear();
					break;
				}
				// otherwise go on
				Choreo choreoInclude = loadSubChoreo(newfile, subChoreoNum+1, parents);
				if (choreoInclude.empty()) {
					loadingCorrect = false;
					choreo.clear();
					break;
				}
				choreo.insert(choreo.end(), choreoInclude.begin(), choreoInclude.end());
			} else if (printChoreo) {
				WARNING_MSG(spacesC << " -> Unknown childs!");
			}
		}
	} catch (...) {
		if (printChoreo) {
			ERROR_MSG(spacesP << "Problem in reading xml file => Clearing Choreo!");
		}
		choreo.clear();
	}
	if (choreo.empty() && loadingCorrect && !parentFound && printChoreo) {
		ERROR_MSG(spacesP << "Choreo is empty!");
	}	
	return choreo;
}

// loading choreography procedure
Choreo loadChoreo(std::string choreoName) {
	std::vector<std::string> parents;
	return loadSubChoreo(choreoName, 0, parents);
}


void setSpeeds(float v, float w, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed((int)(v*TO_MICRO), (int)(w*TO_MICRO));
}

void setLights(rsb::Informer< std::vector<int> >::Ptr informer, int lightType, amiro::Color color, int periodTime) {
	std::vector<int> lightVector = LightModel::setLight2Vec(lightType, color, periodTime);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightVector.begin(),lightVector.end()));
	informer->publish(commandVector);
}

void setLightsVec(rsb::Informer< std::vector<int> >::Ptr informer, int lightType, light_t colors, int periodTime) {
	std::vector<amiro::Color> colorVec;
	for (int l=0; l<8; l++) {
		amiro::Color color(colors[l][0], colors[l][1], colors[l][2]);
		colorVec.push_back(color);
	}
	std::vector<int> lightVector = LightModel::setLights2Vec(lightType, colorVec, periodTime);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightVector.begin(),lightVector.end()));
	informer->publish(commandVector);
}

int main(int argc, char **argv) {

	// delay to start the choreo after the rsb-event was created in ms
	int choreoDelay = 2000;
	int stepDelay = 1000;
	int idDelay = 1000;

	// default choreo name
	std::string choreoName = "testChoreo.xml";

	int amiroID = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("verbose,v", "Print values that are published via CAN.")
			("amiroID,a", po::value<int>(&amiroID), "ID of the AMiRo, which has to be unique! Flag must be set!")
			("lightsOut", po::value<std::string>(&lightOutscope), "Light outscope.")
			("choreoIn", po::value<std::string>(&choreoInscope), "Choreography inscope.")
			("choreoname,c", po::value<std::string>(&choreoName), "Initial Choreography name.")
			("choreoRelative,r", "Flag if the positions in the choreo are relative.")
			("printChoreo,p", "Prints the loaded steps of the choreo.")
			("choreoDelay", po::value<int>(&choreoDelay), "Delay between receiving the start command via RSB and starting the choreography in ms (default 2000 ms).")
			("stepDelay", po::value<int>(&stepDelay), "Delay of the brake between two steps of the choreography in ms (default 1000 ms).")
			("idDelay", po::value<int>(&idDelay), "Delay between the AMiRo starts in ms (default: 1000 ms).")
			("useOdo,o", "Flag if for navigation just the odometry shall be used.")
			("useTwb,t", "Flag if for navigation the telework bench shall be used (tracking navigation).")
			("markerId,m", po::value<int>(&markerID), "ID of the marker for robot detection (has to be set if tracking navigation is activated).")
			("mmp", po::value<float>(&meterPerPixel), "Meter per pixel of the robot detection (has to be set if tracking navigation is activated).");

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

	// check for given AMiRo ID
	if (!vm.count("amiroID")) {
		std::cout << "The AMiRo ID has to be given! Please check the options.\n\n" << options << "\n";
		exit(1);
	}

	// check for given navigation type
	if (!vm.count("useOdo") && !vm.count("useTwb")) {
		std::cout << "Please set the navigation type:\n -> Odometry only ('useOdo')\n -> TWB detection ('useTwb')\n\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	}
	if (vm.count("useTwb")) {
		if (!vm.count("markerId") && !vm.count("mmp")) {
			std::cout << "The navigation by TWB is actived. Please set the marker ID and the meter per pixel factor!\nPlease check the options.\n\n" << options << "\n";
			exit(1);
		} else if (!vm.count("markerId")) {
			std::cout << "The navigation by TWB is actived. Please set the marker ID!\nPlease check the options.\n\n" << options << "\n";
			exit(1);
		} else if (!vm.count("mmp")) {
			std::cout << "The navigation by TWB is actived. Please set the meter per pixel factor!\nPlease check the options.\n\n" << options << "\n";
			exit(1);
		}
		useTwb = true;
	} else {
		useOdo = true;
	}


	// Set AMiRo name
	amiroName = "amiro" + boost::lexical_cast<std::string>(amiroID);

	// Define delays
	int delay = choreoDelay + idDelay*amiroID;
	int delayS = stepDelay + idDelay*amiroID;

	printChoreo = vm.count("printChoreo");
	relativePosition = vm.count("choreoRelative");


	INFO_MSG("Start harvester algorithm with name '" << amiroName << "'.");

	// +++++ RSB Initalization +++++

	// Get the RSB factory
#if RSB_VERSION_NUMERIC<1200
	rsb::Factory& factory = rsb::Factory::getInstance();
#else
	rsb::Factory& factory = rsb::getFactory();
#endif

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ------------ ExtSpread Config ----------------

	// Generate the programatik Spreadconfig for extern communication
//	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER
	// Get the global participant config as a template
	rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig(); {
		// disable socket transport
		rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();
		tmpPropSocket["enabled"] = boost::any(std::string("0"));

		// Get the options for spread transport, because we want to change them
		rsc::runtime::Properties tmpPropSpread  = tmpPartConf.mutableTransport("spread").getOptions();

		// enable socket transport
		tmpPropSpread["enabled"] = boost::any(std::string("1"));

		// Change the config
		tmpPropSpread["host"] = boost::any(std::string("localhost"));

		// Change the Port
		tmpPropSpread["port"] = boost::any(std::string("4823"));

		// Write the tranport properties back to the participant config
		tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
		tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
	}

	// ------------ Listener ---------------------

	// prepare RSB listener for choreos
	rsb::ListenerPtr choreoListener = factory.createListener(choreoInscope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> choreoQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	choreoListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(choreoQueue)));

	// prepare RSB listener for commands
	rsb::ListenerPtr commandListener = factory.createListener(commandInscope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> commandQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(commandQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

	// prepare RSB listener for amiros
	rsb::ListenerPtr amiroListener = factory.createListener(amiroScope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> amiroQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(50));
	amiroListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(amiroQueue)));

	// ------------ Informer -----------------

	// prepare RSB informer for amiros
	rsb::Informer<std::string>::Ptr amiroInformer = factory.createInformer<std::string> (amiroScope, tmpPartConf);

	// prepare RSB informer for setting lights
	rsb::Informer< std::vector<int> >::Ptr lightInformer = factory.createInformer< std::vector<int> > (lightOutscope);



	// Initialize the robot communication
	boost::shared_ptr<twbTracking::proto::Pose2DList> data;
	std::vector<std::string> amiros;
	bool initialized = false;
	bool exitProg = false;
	bool ledsLeft = true;
	int waitCounter = 0;
	INFO_MSG("");
	INFO_MSG("");
	INFO_MSG("Initializing:");
	// set lights
	setLights(lightInformer, LightModel::LightType::CMD_WARNING, amiro::Color(255,255,0), 600);
	while (!initialized) {
		// check amiro queue
		if (!amiroQueue->empty()) {
			EventPtr event = amiroQueue->pop(0);
			std::string inputName = *static_pointer_cast<std::string>(event->getData());
			if (amiroName != inputName) {
				bool isIn = false;
				for (std::string name : amiros) {
					if (name == inputName) {
						isIn = true;
						break;
					}
				}
				if (!isIn) {
					amiros.push_back(inputName);
					INFO_MSG(" -> Recognized AMiRo: '" << inputName << "'");
				}
			}
		}

		// check command queue
		if (!commandQueue->empty()) {
			EventPtr event = commandQueue->pop(0);
			std::string command = *static_pointer_cast<std::string>(event->getData());
			initialized = command == COMMAND_INIT;
		}

		// inform other amiros
		if (waitCounter >= 5) {
			waitCounter = 0;
			boost::shared_ptr<std::string> StringPtr(new std::string(amiroName));
			amiroInformer->publish(StringPtr);
		} else {
			waitCounter++;
		}

		// wait 100 ms
		usleep(100000);
	}

	bool amiroCheck[amiros.size()];
	for (int i=0; i<amiros.size(); i++) {
		amiroCheck[i] = false;
	}

	// initialize the robots attributes
	light_t lights;
	system_clock::time_point *startSystemTime = new system_clock::time_point(system_clock::now());
	srand(time(NULL));

	bool quitProg = false;
	while (!quitProg) {
		// set lights
		setLights(lightInformer, LightModel::LightType::CMD_SHINE, amiro::Color(255,255,255), 0);

		// wait for start command
		bool startHarvest = false;
		INFO_MSG("");
		INFO_MSG("Waiting for commands ...");
		while (!startHarvest) {
			// check choreo queue
			if (!choreoQueue->empty()) {
				EventPtr event = choreoQueue->pop(0);
				choreoName = *static_pointer_cast<std::string>(event->getData());
				INFO_MSG(" -> Choreo name has been changed to '" << choreoName << "'");
			}

			// check command queue
			if (!commandQueue->empty()) {
				EventPtr event = commandQueue->pop(0);
				std::string command = *static_pointer_cast<std::string>(event->getData());
				startHarvest = command == COMMAND_START;
				exitProg = command == COMMAND_STOP;
				if (startHarvest) {
					delete startSystemTime;
					startSystemTime = new system_clock::time_point(microseconds(event->getMetaData().getReceiveTime()) + milliseconds(delay));
				} else if (exitProg) {
					INFO_MSG("");
					INFO_MSG("Exit Harvester ...");
					break;
				}
			}

			// wait for 100 ms
			usleep(100000);
		}
		if (exitProg) {
			quitProg = true;
			break;
		}

		// load choreo
		Choreo choreo = loadChoreo(choreoName);
		INFO_MSG("");
		string frontPart = " => Start choreo '" + choreoName + "': ";
		if (choreo.empty()) {
			ERROR_MSG(frontPart << "ERROR in Choreo!");
			continue;
		} else {
			INFO_MSG(frontPart << choreo.size() << " steps");
		}

		// reset odometry
		odoPos.x = 0;
		odoPos.y = 0;
		odoPos.f_z = 0;
		myCAN.setOdometry(odoPos);

		// wait for choreo to begin
		setLights(lightInformer, LightModel::LightType::CMD_WARNING, amiro::Color(255,255,0), 600);
		if (vm.count("verbose")) DEBUG_MSG("Waiting ...");
		boost::this_thread::sleep_until(*startSystemTime);
		if (vm.count("verbose")) DEBUG_MSG("Start choreo ...");

		// Clear amiro queue
		while (!amiroQueue->empty()) {
			amiroQueue->pop(0);
		}

		// perform the choreo
		for (ChoreoStep cs : choreo) {

			float moveDirX = cs.position[0];
			float moveDirY = cs.position[1];
			// get current position
			if (useTwb) {
				odoPos = getNextTrackingPos(trackingQueue);
				DEBUG_MSG("Position of marker " << markerID << ": " << odoPos.x << "/" << odoPos.y << ", " << odoPos.f_z);
			}
			if (!relativePosition) {
				moveDirX -= (float)(odoPos.x)*MICRO_TO;
				moveDirY -= (float)(odoPos.y)*MICRO_TO;
			}

			// set lights
			setLightsVec(lightInformer, LightModel::LightType::CMD_SHINE, cs.lights, 0);

			// set motors
			if (useOdo) {
				// calculate speeds
				float v, w, time, dist, angle;
				calcSpeeds(odoPos, moveDirX, moveDirY, v, w, time, dist, angle);

				// print action messages
				if (vm.count("verbose")) {
					DEBUG_MSG("------------------------------------------------------------");
					// print the CAN values to output
					DEBUG_MSG("own position: " << (odoPos.x*MICRO_TO) << "/" << (odoPos.y*MICRO_TO) << ", " << (odoPos.f_z*MICRO_TO));
					DEBUG_MSG("goal position: " << cs.position[0] << "/" << cs.position[1]);
					DEBUG_MSG("v: " << v << " m/s");
					DEBUG_MSG("w: " << w << " rad/s");
					DEBUG_MSG("time: " << time << " s");
				}

				// set speed
				if ((int)(time*TO_MICRO) - timebuffer/2 > 0) {
					setSpeeds(v, w, myCAN);
					usleep((int)(time*TO_MICRO) - timebuffer/2);
				}

				// set position
				if (relativePosition) {
					odoPos.x += (int)(cs.position[0] * TO_MICRO);
					odoPos.y += (int)(cs.position[1] * TO_MICRO);
				} else {
					odoPos.x = (int)(cs.position[0] * TO_MICRO);
					odoPos.y = (int)(cs.position[1] * TO_MICRO);
				}
				odoPos.f_z = odoPos.f_z + angle*TO_MICRO;
				myCAN.setOdometry(odoPos);

			} else if (useTwb) {
				types::position goalPos;
				goalPos.x = odoPos.x + moveDirX*TO_MICRO;
				goalPos.y = odoPos.y + moveDirY*TO_MICRO;
				while (moveDirX > trackingVariance || moveDirY > trackingVariance) {
					// calculate speeds
					float v, w, time, dist, angle;
					calcSpeeds(odoPos, moveDirX, moveDirY, v, w, time, dist, angle);
					setSpeeds(v, w, myCAN);

					// print action messages
					if (vm.count("verbose")) {
						DEBUG_MSG("------------------------------------------------------------");
						// print the CAN values to output
						DEBUG_MSG("own position: " << (odoPos.x*MICRO_TO) << "/" << (odoPos.y*MICRO_TO) << ", " << (odoPos.f_z*MICRO_TO));
						DEBUG_MSG("goal position: " << (goalPos.x*MICRO_TO) << "/" << (goalPos.y*MICRO_TO));
						DEBUG_MSG("v: " << v << " m/s");
						DEBUG_MSG("w: " << w << " rad/s");
						DEBUG_MSG("time: " << time << " s");
					}

					odoPos = getNextTrackingPos(trackingQueue);
					moveDirX = (goalPos.x - odoPos.x)*MICRO_TO;
					moveDirY = (goalPos.y - odoPos.y)*MICRO_TO;
				}
			}


			if (cs.braking) {
				// stop motors
				setSpeeds(0, 0, myCAN);

				// reset amiro flags
				for (int i=0; i<amiros.size(); i++) {
					amiroCheck[i] = false;
				}

				// Check for all ready amiros
				if (vm.count("verbose")) DEBUG_MSG("List of AMiRos which are already finished:");
				while (!amiroQueue->empty()) {
					EventPtr event = amiroQueue->pop(0);
					std::string inputName = *static_pointer_cast<std::string>(event->getData());
					bool isIn = false;
					int id = -1;
					for (int i=0; i<amiros.size(); i++) {
						if (amiros[i] == inputName) {
							isIn = true;
							id = i;
							break;
						}
					}
					if (isIn && !amiroCheck[id]) {
						amiroCheck[id] = true;
						if (vm.count("verbose")) DEBUG_MSG(" -> Ready AMiRo: '" << inputName.c_str() << "'");
					}
				}
				usleep (100000);

				// check for rest (and own)
				bool ownNameReceived = false;
				bool allOthersReceived = false;
				if (vm.count("verbose")) DEBUG_MSG("Waiting for rest:");
				int publishCounter = 5;
				// set lights
				setLights(lightInformer, LightModel::LightType::CMD_WARNING, amiro::Color(255,255,0), 600);
				while (!ownNameReceived || !allOthersReceived) {
					// Send own name
					boost::shared_ptr<std::string> StringPtr(new std::string(amiroName));
					amiroInformer->publish(StringPtr);

					// check for rest
					while (!amiroQueue->empty()) {
						EventPtr event = amiroQueue->pop(0);
						std::string inputName = *static_pointer_cast<std::string>(event->getData());
						if (amiroName != inputName) {
							bool isIn = false;
							int id = -1;
							for (int i=0; i<amiros.size(); i++) {
								if (amiros[i] == inputName) {
									isIn = true;
									id = i;
									break;
								}
							}
							if (isIn && !amiroCheck[id]) {
								amiroCheck[id] = true;
								if (vm.count("verbose")) DEBUG_MSG(" -> Received AMiRo: '" << inputName << "'");
								delete startSystemTime;
								startSystemTime = new system_clock::time_point(microseconds(event->getMetaData().getReceiveTime()) + milliseconds(delayS));
							}
						} else if (amiroName == inputName && !ownNameReceived) {
							ownNameReceived = true;
							if (vm.count("verbose")) DEBUG_MSG(" -> Received AMiRo: '" << inputName << "' => own name!");
							delete startSystemTime;
							startSystemTime = new system_clock::time_point(microseconds(event->getMetaData().getReceiveTime()) + milliseconds(delayS));
						}
					}

					allOthersReceived = true;
					for (int i=0; i<amiros.size(); i++) {
						if (!amiroCheck[i]) {
							allOthersReceived = false;
							break;
						}
					}

					// wait for 300 ms
					usleep(300000);
				}
				// Clear amiro queue
				while (!amiroQueue->empty()) {
					amiroQueue->pop(0);
				}

				// Set LEDs
				setLights(lightInformer, LightModel::LightType::CMD_SHINE, amiro::Color(255,255,0), 0);

				// to continue just wait until deadline
				if (vm.count("verbose")) DEBUG_MSG("All are finished. Waiting ...");
				boost::this_thread::sleep_until(*startSystemTime);
				if (vm.count("verbose")) DEBUG_MSG("Continue choreo ...");

				// Clear amiro queue again (there have been messages again)
				while (!amiroQueue->empty()) {
					amiroQueue->pop(0);
				}
			} else {
				if (vm.count("verbose")) DEBUG_MSG("No brake, continue ...");
			}
		}

		// stop Robot
		setSpeeds(0, 0, myCAN);

		// rest print
		if (vm.count("verbose")) {
			DEBUG_MSG("------------------------------------------------------------");
		}

		// set lights
		setLights(lightInformer, LightModel::LightType::CMD_SHINE, amiro::Color(255,255,255), 0);
	}

	// set lights
	setLights(lightInformer, LightModel::LightType::CMD_INIT, amiro::Color(0,0,0), 0);

	return EXIT_SUCCESS;
}
