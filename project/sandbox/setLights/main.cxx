//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Sets the lights based on RSB commands.
//============================================================================

//#define TRACKING
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
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
#include <rsb/util/EventQueuePushHandler.h>
#include <rsb/MetaData.h>
#include <rsb/QueuePushHandler.h>


using namespace rsb;

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <extspread.hpp>

using namespace rsb;
using namespace rsb::patterns;

// color commands
#define CMD_INIT 0
#define CMD_SHINE 1
#define CMD_BLINK 2
#define CMD_WARNING 3
#define CMD_CIRCLELEFT 4
#define CMD_CIRCLERIGHT 5

#define MIN_TIME_BLINK 200
#define MIN_TIME_WARNING 400
#define MIN_TIME_CIRCLED 800


// scopenames and spread data for rsb
std::string commandInscope = "/amiro/lights";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4803";

// color data and light type
amiro::Color curColor(255, 255, 255);
bool init = true;
bool shine = false;
bool blinking = false;
bool blinkWarning = false;
bool circleLeft = false;
bool circleRight = false;

amiro::Color colorSet;
int commandTypeSet = CMD_INIT;
int periodTimeSet = 0;

bool exitProg = false;


// method prototypes
bool getCommand(boost::shared_ptr<std::vector<int>> commandVector);
void setColor(bool colorChanged, int counter100ms, ControllerAreaNetwork &myCAN);


// main function
int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("commandScope,c", po::value<std::string>(&commandInscope), "Inscope for commands (default: '/amiro/lights').")
			("host", po::value<std::string>(&spreadhost), "Host for Programatik Spread (default: '127.0.0.1').")
			("port", po::value<std::string>(&spreadport), "Port for Programatik Spread (default: '4803').");

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

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ------------ Listener ----------------------

	// prepare RSB listener for commands
	rsb::ListenerPtr commandListener = factory.createListener(commandInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(commandQueue)));



	// Init the CAN interface
	ControllerAreaNetwork myCAN;
	boost::shared_ptr<std::vector<int>> commandVector;
	int timeCounter = 0;
	setColor(true, timeCounter, myCAN);

	INFO_MSG("Starting lighting loop");
	exitProg = false;
	while (!exitProg) {
		// check for new color input
		bool colorChanged = false;
		if (!commandQueue->empty()) {
			commandVector = boost::static_pointer_cast<std::vector<int> >(commandQueue->pop());
			colorChanged = getCommand(commandVector);
		}
		// set time counter
		if (timeCounter >= periodTimeSet/100) {
			timeCounter = 0;
		} else {
			timeCounter++;
		}
		// set colors
		setColor(colorChanged, timeCounter, myCAN);

		// sleep 100 ms
		usleep(100000);
	}

	commandTypeSet = CMD_INIT;
	init = true;
	setColor(true, 0, myCAN);

	// normal exit procedure
	INFO_MSG("Exit");
	return EXIT_SUCCESS;
}


bool getCommand(boost::shared_ptr<std::vector<int>> commandVector) {
	if (commandVector->size() == 5) {
		// check if everything negative
		bool allNegative = true;
		for (int c=0; c<5; c++) {
			if (commandVector->at(c) >= 0) {
				allNegative = false;
				break;
			}
		}
		if (allNegative) {
			exitProg = true;
			return false;
		}

		// read command
		bool error = false;
		int commandType = commandVector->at(0);
		amiro::Color color(commandVector->at(1), commandVector->at(2), commandVector->at(3));
		int periodTime = commandVector->at(4);

		bool colorChanged = commandType != commandTypeSet
					|| color.getRed() != colorSet.getRed()
					|| color.getGreen() != colorSet.getGreen()
					|| color.getBlue() != colorSet.getBlue()
					|| periodTime != periodTimeSet;

		if (colorChanged) {
			init = false;
			shine = false;
			blinking = false;
			blinkWarning = false;
			circleLeft = false;
			circleRight = false;

			switch (commandType) {
				case CMD_INIT: init = true; break;
				case CMD_SHINE: shine = true; break;
				case CMD_BLINK: blinking = true; break;
				case CMD_WARNING: blinkWarning = true; break;
				case CMD_CIRCLELEFT: circleLeft = true; break;
				case CMD_CIRCLERIGHT: circleRight = true; break;
				default: WARNING_MSG("Light type '" << commandType << "' is unknown!"); error = true; break; 
			}
			if (error) {
				switch (commandTypeSet) {
					case CMD_INIT: init = true; break;
					case CMD_SHINE: shine = true; break;
					case CMD_BLINK: blinking = true; break;
					case CMD_WARNING: blinkWarning = true; break;
					case CMD_CIRCLELEFT: circleLeft = true; break;
					case CMD_CIRCLERIGHT: circleRight = true; break;
				}
				return false;
			} else {
				commandTypeSet = commandType;
				colorSet.setRedGreenBlue(color.getRed(), color.getGreen(), color.getBlue());
				periodTimeSet = periodTime;
				if (periodTimeSet % 200 != 0) periodTimeSet += (200 - periodTimeSet % 200);
				switch (commandTypeSet) {
					case CMD_BLINK: if (periodTimeSet < MIN_TIME_BLINK) periodTimeSet = MIN_TIME_BLINK; break;
					case CMD_WARNING: if (periodTimeSet < MIN_TIME_WARNING) periodTimeSet = MIN_TIME_WARNING; break;
					case CMD_CIRCLELEFT:
					case CMD_CIRCLERIGHT: if (periodTimeSet < MIN_TIME_CIRCLED) periodTimeSet = MIN_TIME_CIRCLED; break;
					default: break;
				}

				switch (commandTypeSet) {
					case CMD_INIT: INFO_MSG("Color changed: Type " << commandTypeSet << ", Color initial"); break;
					case CMD_SHINE: INFO_MSG("Color changed: Type " << commandTypeSet << ", Color " << ((int)(colorSet.getRed())) << "/" << ((int)(colorSet.getGreen())) << "/" << ((int)(colorSet.getBlue())) << " (red/green/blue)"); break;
					default: INFO_MSG("Color changed: Type " << commandTypeSet << ", Color " << ((int)(colorSet.getRed())) << "/" << ((int)(colorSet.getGreen())) << "/" << ((int)(colorSet.getBlue())) << " (red/green/blue), period time " << periodTimeSet << "ms"); break;
				}

				return true;
			}
		}
		return false;
	}
	return false;
}


void setColor(bool colorChanged, int counter100ms, ControllerAreaNetwork &myCAN) {
	int steps100ms = periodTimeSet/100;
	int steps100msHalf = steps100ms/2;
	bool firstHalf = counter100ms < steps100msHalf;
	int roundStepCounter = -1;
	if (periodTimeSet > 0) roundStepCounter = counter100ms*100/(periodTimeSet/8);

	if (   colorChanged
	    || ((blinking || blinkWarning) && counter100ms % steps100msHalf == 0	)
	    || (circleLeft || circleRight)
	) {
		if (init) {
			myCAN.setLightColor(0, amiro::Color(amiro::Color::RED));
			myCAN.setLightColor(1, amiro::Color(amiro::Color::GREEN));
			myCAN.setLightColor(2, amiro::Color(amiro::Color::BLUE));
			myCAN.setLightColor(3, amiro::Color(amiro::Color::WHITE));
			myCAN.setLightColor(4, amiro::Color(amiro::Color::RED));
			myCAN.setLightColor(5, amiro::Color(amiro::Color::GREEN));
			myCAN.setLightColor(6, amiro::Color(amiro::Color::BLUE));
			myCAN.setLightColor(7, amiro::Color(amiro::Color::WHITE));
		} else {
			for (int led=0; led<8; led++) {

				int ledNext = led+1;
				if (ledNext >= 8) ledNext -= 8;

				if (   (shine)
				    || (blinking && firstHalf)
				    || (blinkWarning && firstHalf && led >= 4)
				    || (blinkWarning && !firstHalf && led < 4)
				    || (circleLeft && (7-led == roundStepCounter || 7-ledNext == roundStepCounter))
				    || (circleRight && (led == roundStepCounter || ledNext == roundStepCounter))
				) {
					myCAN.setLightColor(led, colorSet);
				} else {
					myCAN.setLightColor(led, amiro::Color(0, 0, 0));
				}
			}
		}
	}
}
