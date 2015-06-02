// Messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// CAN
#include <ControllerAreaNetwork.h>
#include <Types.h>
#include <Constants.h>
ControllerAreaNetwork myCAN;

// For running system comands
#include <stdlib.h>

// For using kbhit
#include <unistd.h>

// For checking character pressed in the console
#include <kbhit.hpp>

// For using vector
#include <vector>

// For using string
#include <string>

// For program options
#include <boost/program_options.hpp>

// RSB
#include <converter/matConverter/matConverter.hpp>
#include <converter/vecIntConverter/main.hpp>
#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>

using namespace boost;
using namespace std;
using std::vector;
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// State machine states
#define NUM_STATES 5
enum states                          { idle , init , following , waypoint, stopped};
std::string statesString[NUM_STATES] {"idle","init","following","waypoint","stopped"};
states amiroState;

bool personEntered = false;

// Following of the leader
// We send a command "start" string  for start of the following
const std::string sFollowingCmdScope("/following");
const std::string sWaypointCmdScope("/waypoint/command");
// We assume a "finish" string, which indicates the current state of the following program
const std::string sFollowingAnswerScope("/followingState");
const std::string sWaypointAnswerScope("/waypoint/state");

int processSM(void);
void setAMiRoColor(int r, int g, int b);
void set_state_idle();
void set_state_init();
void set_state_following();
void set_state_waypoint();
void set_state_waypoint_entered();
void set_state_stopped();
void stopWaypoint();
void motorActionMilli(int speed, int turn);

std::string g_sInScopeTobi = "/tobiamiro";
std::string g_sOutScopeStateTobi = "/amiro";
std::string g_sRemoteServerPort = "4823";
std::string g_sRemoteServer = "localhost";
std::string amiro_id = "0";
int turn_degree = 180;

int main(int argc, char **argv) {
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
    				("spread,s", po::value < std::string > (&g_sRemoteServer), "IP of remote spread server.")
    				("spreadPort", po::value < std::string > (&g_sRemoteServerPort), "Port of remote spread server.")
    				("outscopeStateTobi", po::value < std::string > (&g_sOutScopeStateTobi), "Scope for sending the current state to Tobi.")
    				("inscopeTobi,i", po::value < std::string > (&g_sInScopeTobi), "Scope for receiving Tobi's messages.")
    				("id", po::value < std::string > (&amiro_id), "ID of AMiRo.")
				("turn", po::value < int > (&turn_degree), "Angle to turn when switching from 'following' to 'waypoint' (deg).");

	// Allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

	// First, process the help option
	if (vm.count("help")) {
		std::cout << options << "\n";
		exit(1);
	}

	// Afterwards, let program options handle argument errors
	po::notify(vm);
	
	g_sInScopeTobi.append("0");
	g_sOutScopeStateTobi.append("0").append("tobi");

	return processSM();
}

int processSM(void) {

	// Create the factory
	rsb::Factory &factory = rsb::getFactory();

	//////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
	////////////////////////////////////////////////////////////////////////////////
	// Get the global participant config as a template
	rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
	{
		// Get the options for socket transport, because we want to change them
		rsc::runtime::Properties tmpProp  = tmpPartConf.mutableTransport("socket").getOptions();

		// Disable socket transport
		std::string enabled = "0";
		tmpProp["enabled"] = boost::any(enabled);

		// Write the socket tranport properties back to the participant config
		tmpPartConf.mutableTransport("socket").setOptions(tmpProp);
	}
	{
		// Get the options for spread transport, because we want to change them
		rsc::runtime::Properties tmpPropSpread = tmpPartConf.mutableTransport("spread").getOptions();

		// Enable socket transport
		std::string enabled = "1";
		tmpPropSpread["enabled"] = boost::any(enabled);

		// The port of the server
		tmpPropSpread["port"] = boost::any(g_sRemoteServerPort);

		// Change the server
		tmpPropSpread["host"] = boost::any(g_sRemoteServer);

		// Write the tranport properties back to the participant config
		tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);

		std:: cout << tmpPropSpread << std::endl;
	}
	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////

	/////////////////// LOCAL SCOPES///////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////

	// Following: Listener and Informer
	rsb::ListenerPtr listenerFollowing = factory.createListener(sFollowingAnswerScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueFollowingAnswer(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
	listenerFollowing->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueFollowingAnswer)));

	rsb::Informer< std::string >::Ptr informerFollowing = factory.createInformer< std::string > (sFollowingCmdScope);

	// Waypoint: Listener and Informer
	rsb::ListenerPtr listenerWaypoint = factory.createListener(sWaypointAnswerScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueWaypointAnswer(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
	listenerWaypoint->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueWaypointAnswer)));

	rsb::Informer< std::string >::Ptr informerWaypoint = factory.createInformer< std::string > (sWaypointCmdScope);

	/////////////////// REMOTE SCOPES///////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	// Prepare RSB informer and listener
	rsb::Informer< std::string >::Ptr informerRemoteState[NUM_STATES];

	// Create the listener
	rsb::ListenerPtr listenerRemoteTobiState;
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueRemoteTobiState(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

	try {
		for (int idx = 0; idx < NUM_STATES; ++idx) {
			std::string sOutScopeStateTobiTmp(g_sOutScopeStateTobi);
			informerRemoteState[idx] = factory.createInformer< std::string > (sOutScopeStateTobiTmp.append("/").append(statesString[idx]), tmpPartConf);
		}

		// Create the listener for the standby task
		listenerRemoteTobiState = factory.createListener(g_sInScopeTobi, tmpPartConf);
		listenerRemoteTobiState->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueRemoteTobiState)));
	}
	catch(std::exception& e) {
		ERROR_MSG("Remote connection not established!");
		return -1;
	}

	// When we reached this point, everything should be fine

	// Local variables
	bool bGotFromTobi_something = false;
	bool bGotFromTobi_init = false;  // True if a state from tobi was received
	bool bGotFromTobi_initfollowing = false;
	bool bGotFromTobi_stopfollowing = false;
	bool bGotFromTobi_initwaypoint = false;
	bool bGotFromTobi_stopwaypoint = false;
//	bool bGotFromWaypoint_entered = false; // True if something entered the waypoint's area
//	bool bGotFromWaypoint_left = false; // True if something left the waypoint's area

//	// Variables for the keypress
//	// Check for keypress in window
//	int KB_codeCV = 0;
//	// Check for keypress in terminal
//	int KB_code = 0;

	boost::shared_ptr<std::string> signal_state(new std::string);
	boost::shared_ptr<std::string> signal_init(new std::string("init"));
	boost::shared_ptr<std::string> signal_stop(new std::string("stop"));

	set_state_idle();

	// Process the behavior
	while (true) {

		bool bGotFromWaypoint_entered = false; // True if something entered the waypoint's area
		bool bGotFromWaypoint_left = false; // True if something left the waypoint's area

		// --- Check for incoming messaturn_degreeges/triggers/signals ---

		// Get messages from Tobi
		if (!queueRemoteTobiState->empty()) {
			bGotFromTobi_something = true;
			INFO_MSG("Received a state from Tobi.")
		    std::string msg(*queueRemoteTobiState->pop());
			if (msg.compare("init") == 0) {
				INFO_MSG("Initialising...")
				bGotFromTobi_init = true;
			} else if (msg.compare("initfollow")== 0) {
				INFO_MSG("Init following...")
				bGotFromTobi_initfollowing = true;
			} else if (msg.compare("stopfollow") == 0) {
				INFO_MSG("Stop following...")
				bGotFromTobi_stopfollowing = true;
			} else if (msg.compare("initwaypoint") == 0) {
				INFO_MSG("Init waypoint...")
				bGotFromTobi_initwaypoint = true;
			} else if (msg.compare("stopwaypoint") == 0) {
				INFO_MSG("Stop waypoint...")
				bGotFromTobi_stopwaypoint = true;
			}
			queueRemoteTobiState->clear();
		} else {
			bGotFromTobi_something = false;
			bGotFromTobi_init = false;
			bGotFromTobi_initfollowing = false;
			bGotFromTobi_stopfollowing = false;
			bGotFromTobi_initwaypoint = false;
			bGotFromTobi_stopwaypoint = false;
		}

//		// Get messages from following
//		if (!queueFollowingAnswer->empty()) {
//			INFO_MSG("Received a message from following.")
//    	    std::string msg(*queueFollowingAnswer->pop());
//			if (msg.compare("undock") == 0) {
//				INFO_MSG("Following stopped.")
//        		bGotFollowing_undock = true;
//			}
////			else if (msg.compare("false") == 0){
////				ERROR_MSG("Exploration failed")
////			}
//		} else {
//			bGotFollowing_undock = false;
//		}

		// Get messages from waypoint
		if (!queueWaypointAnswer->empty()) {
			INFO_MSG("Received a message from waypoint.")
		    std::string msg(*queueWaypointAnswer->pop());
			if (msg.compare("entered") == 0) {
				INFO_MSG("Something entered the waypoint's area.")
		        bGotFromWaypoint_entered = true;
			} else if (msg.compare("left") == 0) {
				INFO_MSG("Something left the waypoint's area.")
		        bGotFromWaypoint_left = true;
			}
		} else {
			bGotFromWaypoint_entered = false;
			bGotFromWaypoint_left = false;
		}

//		// Check for keypress in terminal
//		KB_code = 0;
//		if (kbhit()) {
//			KB_code = getchar();
//			INFO_MSG("KB_code = " << KB_code)
//			// Do an object detection if "d" button was pressed
//			if (KB_code == 1000) {
//				INFO_MSG("Do object detection")
//        				amiroState = objectDetection;
//			}
//		}

		INFO_MSG("STATE: " << statesString[amiroState])

		// --- Produce current state's behavior and do transition ---
		switch (amiroState) {
		case idle:
			setAMiRoColor(255,255,255);
//			if(bGotFromTobi_init) {
				set_state_init();
//			}
			break;
		case init:
			setAMiRoColor(255,255,255);
			// Clear all variables
			bGotFromTobi_init = false;
			bGotFromTobi_stopfollowing = false;
			bGotFromTobi_stopwaypoint = false;
			bGotFromWaypoint_entered = false;
			bGotFromWaypoint_left = false;
			set_state_stopped();
			if(bGotFromTobi_initfollowing) {
				informerFollowing->publish(signal_init);
				set_state_following();
			} else if(bGotFromTobi_initwaypoint) {
				informerWaypoint->publish(signal_init);
				set_state_waypoint();
			}
			bGotFromTobi_initfollowing = false;
			bGotFromTobi_initwaypoint = false;
			break;
		case following:
			if (bGotFromTobi_stopfollowing) {
				informerFollowing->publish(signal_stop);
				set_state_stopped();
			} else if(bGotFromTobi_initwaypoint) {
				informerFollowing->publish(signal_stop);
				informerWaypoint->publish(signal_init);
				set_state_waypoint();
			}
			break;

		case waypoint:
			DEBUG_MSG("Entered=" << bGotFromWaypoint_entered << ", Left=" << bGotFromWaypoint_left << ", personEntered=" << personEntered);
			if(bGotFromTobi_stopwaypoint) {
				informerWaypoint->publish(signal_stop);
				stopWaypoint();
				set_state_stopped();
			} else if(bGotFromWaypoint_entered && !personEntered) {
				DEBUG_MSG("Switched to entered in waypoint!");
				set_state_waypoint_entered();
				boost::shared_ptr<std::string> tmp(new std::string("entered"));
				//informerRemoteState[amiroState]->publish(tmp);
			} else if(bGotFromWaypoint_left && personEntered) {
				DEBUG_MSG("Switched to left in waypoint!");
				set_state_waypoint();
				boost::shared_ptr<std::string> tmp(new std::string("left"));
				informerRemoteState[amiroState]->publish(tmp);
			} else if(bGotFromTobi_initfollowing) {
				informerWaypoint->publish(signal_stop);
				stopWaypoint();
				informerFollowing->publish(signal_init);
				set_state_following();
			}
			break;
		case stopped:
			setAMiRoColor(0,0,255);
			if(bGotFromTobi_initwaypoint) {
				informerWaypoint->publish(signal_init);
				set_state_waypoint();
			} else if(bGotFromTobi_initfollowing) {
				informerFollowing->publish(signal_init);
				set_state_following();
			}
			break;
		default:
			WARNING_MSG("Unknown state of AMiRo's state machine.");
		}

		*signal_state = statesString[amiroState];
		INFO_MSG("SENDING STATE SCOPE: " << g_sOutScopeStateTobi << "/" << *signal_state)

//		informerRemoteState[amiroState]->publish(signal_state);
		// informerAmiroState->publish(signal_state);
		boost::this_thread::sleep(boost::posix_time::seconds(1));

	}
	return 0;
}

void setAMiRoColor(int r, int g, int b) {
	for (int l = 0; l < 8; ++l) {
		myCAN.setLightColor(l,amiro::Color(r,g,b));
	}
}

void set_state_idle() {
	amiroState = idle;
	setAMiRoColor(255,255,255);
}

void set_state_init() {
	amiroState = init;
	setAMiRoColor(255,255,255);
}

void set_state_following() {
	amiroState = following;
	setAMiRoColor(255,255,0);
}

void set_state_waypoint() {
	personEntered = false;
	amiroState = waypoint;
	setAMiRoColor(255,0,127);
}

void set_state_waypoint_entered() {
	personEntered = true;
	amiroState = waypoint;
	setAMiRoColor(0,255,0);
}

void set_state_stopped() {
	amiroState = stopped;
	setAMiRoColor(255,255,255);
}

void stopWaypoint() {
	motorActionMilli(0,-M_PI/4*1000);
	sleep(2);
	motorActionMilli(0,0);
}

void motorActionMilli(int speed, int turn) {
	myCAN.setTargetSpeed(speed*1000, turn*1000);
}
