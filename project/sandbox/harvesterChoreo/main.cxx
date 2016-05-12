//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Statemachine to perform a choreography including brakes.
//============================================================================

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
using namespace rsb;

// types
#include <ControllerAreaNetwork.h>

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
std::string CMD_INIT = "init";
std::string CMD_START = "start";
std::string CMD_STOP = "stop";

// initialize CAN
ControllerAreaNetwork myCAN;

// constants
int driveSpeed = (int)(0.08 * TO_MICRO);
int timebuffer = (int)(0.125 * TO_MICRO);
std::string amiroName = "amiro0";

// Flags
bool printChoreo = false;
bool relativePosition = false;


// scopenames for rsb
std::string choreoInscope = "/harvesters/choreo";
std::string commandInscope = "/harvesters/command";
std::string amiroScope = "/harvesters/harvester";

types::position odoPos;

// load a (sub)choreography from a file
Choreo loadSubChoreo(std::string choreoName, int subChoreoNum, std::vector<std::string> parents) {
	bool loadingCorrect = true;
	bool parentFound = false;
	parents.push_back(choreoName);
	if (printChoreo) {
		cout << "Load choreo '" << choreoName << "'" << endl;
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
					for (int i=0; i<=subChoreoNum; i++) {
						cout << "  ";
					}
					cout << "-> choreo step: " << position[0] << "/" << position[1] << " [m]" << endl;
				}
			} else if (tree.first == XMLInclude) {
				std::string newfile = tree.second.get<std::string>(XMLIncludeName);
				// check for parent name
				parentFound = false;
				for (std::string parentName : parents) {
					if (newfile == parentName) {
						if (printChoreo) {
							for (int i=0; i<=subChoreoNum; i++) {
								cout << "  ";
							}
							cout << "-> choreo include: ERROR!\n";
							for (int i=0; i<=subChoreoNum; i++) {
								cout << "  ";
							}
							cout << "   The choreo '" << newfile.c_str() << "' is a parent choreo! Loops are not allowed!" << endl;
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
				if (printChoreo) {
					for (int i=0; i<=subChoreoNum; i++) {
						cout << "  ";
					}
					cout << "-> choreo include: ";
				}
				Choreo choreoInclude = loadSubChoreo(newfile, subChoreoNum+1, parents);
				if (choreoInclude.empty()) {
					loadingCorrect = false;
					choreo.clear();
					break;
				}
				choreo.insert(choreo.end(), choreoInclude.begin(), choreoInclude.end());
			} else if (printChoreo) {
				for (int i=0; i<=subChoreoNum; i++) {
					cout << "  ";
				}
				cout << " -> Unknown childs!" << endl;
			}
		}
	} catch (...) {
		if (printChoreo) {
			for (int i=0; i<subChoreoNum; i++) {
				cout << "  ";
			}
			cout << "=> ERROR !!!" << endl;
		}
		choreo.clear();
	}
	if (choreo.empty() && loadingCorrect && !parentFound && printChoreo) {
		for (int i=0; i<subChoreoNum; i++) {
			cout << "  ";
		}
		cout << "=> ERROR: Choreo is empty!" << endl;
	}	
	return choreo;
}

// loading choreography procedure
Choreo loadChoreo(std::string choreoName) {
	std::vector<std::string> parents;
	return loadSubChoreo(choreoName, 0, parents);
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
			("amiroID,a", po::value<int>(&amiroID), "ID of the AMiRo, which has to be unique!")
			("choreoIn", po::value<std::string>(&choreoInscope), "Choreography inscope.")
			("choreoname,c", po::value<std::string>(&choreoName), "Choreography name.")
			("choreoRelative,r", "Flag if the positions in the choreo are relative.")
			("printChoreo,p", "Prints the loaded steps of the choreo.")
			("choreoDelay", po::value<int>(&choreoDelay), "Delay between receiving the start command via RSB and starting the choreography in ms (default 2000 ms).")
			("stepDelay", po::value<int>(&stepDelay), "Delay of the brake between two steps of the choreography in ms (default 1000 ms).")
			("idDelay", po::value<int>(&idDelay), "Delay between the AMiRo starts in ms (default: 1000 ms).");

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

	if (!vm.count("amiroID")) {
		std::cout << "The AMiRo ID has to be given! Please check the options.\n\n" << options << "\n";
		exit(1);
	}

	// Set AMiRo name
	amiroName = "amiro" + boost::lexical_cast<std::string>(amiroID);

	// Define delays
	int delay = choreoDelay + idDelay*amiroID;
	int delayS = stepDelay + idDelay*amiroID;

	printChoreo = vm.count("printChoreo");
	relativePosition = vm.count("choreoRelative");


	cout << "Start harvester algorithm with name '" << amiroName << "'." << endl;

	// +++++ RSB Initalization +++++

	// Get the RSB factory
#if RSB_VERSION_NUMERIC<1200
	rsb::Factory& factory = rsb::Factory::getInstance();
#else
	rsb::Factory& factory = rsb::getFactory();
#endif

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

	// prepare RSB listener for amiros
	rsb::ListenerPtr amiroListener = factory.createListener(amiroScope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> amiroQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(50));
	amiroListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(amiroQueue)));

	// prepare RSB informer for amiros
	rsb::Informer<std::string>::Ptr amiroInformer = factory.createInformer<std::string> (amiroScope, tmpPartConf);



	// Initialize the robot communication
	std::vector<std::string> amiros;
	bool initialized = false;
	bool exitProg = false;
	bool ledsLeft = true;
	for (int l=0; l<8; l++) {
		if (l<4 && !ledsLeft || l>3 && ledsLeft) {
			myCAN.setLightColor(l, amiro::Color(255, 255, 0));
		} else {
			myCAN.setLightColor(l, amiro::Color(0, 0, 0));
		}
	}
	int waitCounter = 0;
	cout << "\n\nInitializing:" << endl;
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
					cout << " -> Recognized AMiRo: '" << inputName.c_str() << "'" << endl;
				}
			}
		}

		// check command queue
		if (!commandQueue->empty()) {
			EventPtr event = commandQueue->pop(0);
			std::string command = *static_pointer_cast<std::string>(event->getData());
			initialized = command == CMD_INIT;
		}

		// inform other amiros
		if (waitCounter >= 5) {
			waitCounter = 0;
			boost::shared_ptr<std::string> StringPtr(new std::string(amiroName));
			amiroInformer->publish(StringPtr);
			ledsLeft = !ledsLeft;			
			for (int l=0; l<8; l++) {
				if (l<4 && !ledsLeft || l>3 && ledsLeft) {
					myCAN.setLightColor(l, amiro::Color(255, 255, 0));
				} else {
					myCAN.setLightColor(l, amiro::Color(0, 0, 0));
				}
			}
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

	while (true) {
		// set lights
		for (int l = 0; l < 8; ++l) {
			myCAN.setLightColor(l, amiro::Color(255, 255, 255));
		}

		// wait for start command
		bool startHarvest = false;
		cout << "\nWaiting for commands ..." << endl;
		while (!startHarvest) {
			// check choreo queue
			if (!choreoQueue->empty()) {
				EventPtr event = choreoQueue->pop(0);
				choreoName = *static_pointer_cast<std::string>(event->getData());
				cout << " -> Choreo name has been changed to '" << choreoName.c_str() << "'" << endl;
			}

			// check command queue
			if (!commandQueue->empty()) {
				EventPtr event = commandQueue->pop(0);
				std::string command = *static_pointer_cast<std::string>(event->getData());
				startHarvest = command == CMD_START;
				exitProg = command == CMD_STOP;
				if (startHarvest) {
					delete startSystemTime;
					startSystemTime = new system_clock::time_point(microseconds(event->getMetaData().getReceiveTime()) + milliseconds(delay));
				} else if (exitProg) {
					cout << "\nExit Harvester ..." << endl;
					break;
				}
			}

			// wait for 100 ms
			usleep(100000);
		}
		if (exitProg) {
			break;
		}

		// load choreo
		Choreo choreo = loadChoreo(choreoName);
		cout << "\n => Start choreo '" << choreoName.c_str() << "': ";
		if (choreo.empty()) {
			cout << "ERROR in Choreo!" << endl;
			continue;
		} else {
			cout << choreo.size() << " steps" << endl;
		}

		// reset odometry
		odoPos.x = 0;
		odoPos.y = 0;
		odoPos.f_z = 0;
		myCAN.setOdometry(odoPos);

		// wait for choreo to begin
		if (vm.count("verbose")) cout << "Waiting ..." << endl;
		boost::this_thread::sleep_until(*startSystemTime);
		if (vm.count("verbose")) cout << "Start choreo ..." << endl;

		// Clear amiro queue
		while (!amiroQueue->empty()) {
			amiroQueue->pop(0);
		}

		// perform the choreo
		for (ChoreoStep cs : choreo) {

			float moveDirX = cs.position[0];
			float moveDirY = cs.position[1];
			if (!relativePosition) {
				moveDirX -= (float)(odoPos.x)*MICRO_TO;
				moveDirY -= (float)(odoPos.y)*MICRO_TO;
			}
			float actAngle = (float)(odoPos.f_z)*MICRO_TO;
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
			if (moveDist == 0.0) {
				moveDist = distDir;
			}

			// calculate speeds
			float basicSpeed = (float)driveSpeed * MICRO_TO; // m/s
			float timeAction = moveDist/basicSpeed; // s
			int v = driveSpeed; // um/s
			int w = (int)(moveAngle/timeAction * TO_MICRO); // urad/s

			// set lights
			for (int l = 0; l < 8; ++l) {
				myCAN.setLightColor(l, amiro::Color(cs.lights[l][0], cs.lights[l][1], cs.lights[l][2]));
			}

			// print action messages
			if (vm.count("verbose")) {
				cout << "------------------------------------------------------------" << endl;
				// print the CAN values to output
				cout << "own position: " << (odoPos.x*MICRO_TO) << "/" << (odoPos.y*MICRO_TO) << ", " << actAngle << "\ngoal position: " << cs.position[0] << "/" << cs.position[1] << "\nbasic speed: " << basicSpeed << " m/s\nv: " << v << " um/s\nw: " << w << " urad/s\ntime: " << timeAction << " s\nlights: ";
				for (int l = 0; l < 8; ++l) {
					cout << "\n  - [" << cs.lights[l][0] << "," << cs.lights[l][1] << "," << cs.lights[l][2] << "]";
				}
				cout << endl;
			}

			// set speed
			if ((int)(timeAction*TO_MICRO) - timebuffer/2 > 0) {
				myCAN.setTargetSpeed(v, w);
				usleep((int)(timeAction*TO_MICRO) - timebuffer/2);
			}
			if (cs.braking) {
				// stop motors
				myCAN.setTargetSpeed(0, 0);

				// reset amiro flags
				for (int i=0; i<amiros.size(); i++) {
					amiroCheck[i] = false;
				}

				// Check for all ready amiros
				if (vm.count("verbose")) cout << "\nList of AMiRos which are already finished:" << endl;
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
						if (vm.count("verbose")) cout << " -> Ready AMiRo: '" << inputName.c_str() << "'" << endl;
					}
				}
				usleep (100000);

				// check for rest (and own)
				bool ownNameReceived = false;
				bool allOthersReceived = false;
				if (vm.count("verbose")) cout << "Waiting for rest:" << endl;
				int publishCounter = 5;
				ledsLeft = true;
				while (!ownNameReceived || !allOthersReceived) {
					// Set LEDs
					for (int l=0; l<8; l++) {
						if (l<4 && !ledsLeft || l>3 && ledsLeft) {
							myCAN.setLightColor(l, amiro::Color(255, 255, 0));
						} else {
							myCAN.setLightColor(l, amiro::Color(0, 0, 0));
						}
					}
					ledsLeft = !ledsLeft;

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
								if (vm.count("verbose")) cout << " -> Received AMiRo: '" << inputName.c_str() << "'" << endl;
								delete startSystemTime;
								startSystemTime = new system_clock::time_point(microseconds(event->getMetaData().getReceiveTime()) + milliseconds(delayS));
							}
						} else if (amiroName == inputName && !ownNameReceived) {
							ownNameReceived = true;
							if (vm.count("verbose")) cout << " -> Received AMiRo: '" << inputName.c_str() << "' => own name!" << endl;
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
				for (int l=0; l<8; l++) {
					myCAN.setLightColor(l, amiro::Color(255, 255, 0));
				}

				// to continue just wait until deadline
				if (vm.count("verbose")) cout << "All are finished. Waiting ..." << endl;
				boost::this_thread::sleep_until(*startSystemTime);
				if (vm.count("verbose")) cout << "Continue choreo ..." << endl;

				// Clear amiro queue again (there have been messages again)
				while (!amiroQueue->empty()) {
					amiroQueue->pop(0);
				}
			} else {
				if (vm.count("verbose")) cout << "\nNo brake, continue ..." << endl;
			}

			// set position
			if (relativePosition) {
				odoPos.x += (int)(cs.position[0] * TO_MICRO);
				odoPos.y += (int)(cs.position[1] * TO_MICRO);
			} else {
				odoPos.x = (int)(cs.position[0] * TO_MICRO);
				odoPos.y = (int)(cs.position[1] * TO_MICRO);
			}
			odoPos.f_z = (int)((actAngle + moveAngle) * TO_MICRO);
			myCAN.setOdometry(odoPos);
		}

		// stop Robot
		myCAN.setTargetSpeed(0, 0);

		// rest print
		if (vm.count("verbose")) {
			cout << "------------------------------------------------------------" << endl;
		}
	}

	return EXIT_SUCCESS;
}
