// std includes
#include <iostream>
using namespace std;

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "../../includes/MSG.h"

#include <math.h>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <types/LocatedLaserScan.pb.h>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace rsb::patterns;

//    if(lparams_.angle_max < lparams_.angle_min){
//      // flip readings
//      for(int i=0; i < scan.scan_values_size(); i++)
//        data.d[i] = (int) (scan.scan_values(scan.scan_values_size()-1-i)*METERS_TO_MM);
//    }else{
//      for(int i=0; i < scan.scan_values_size(); i++)
//        data.d[i] = (int) (scan.scan_values(i)*METERS_TO_MM);
//    }
//

int main(int argc, const char **argv) {
	DEBUG_MSG("Start waypoint")
	namespace po = boost::program_options;
	std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
	std::string stateOutScope = "/waypoint/state";
	std::string commandInscope = "/waypoint/command";
	float range = 1.0;
	float diffThreshold = 0.3;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("lidarinscope",po::value<std::string>(&lidarInScope),"Scope for receiving lidar data")
			("stateoutscope",po::value<std::string>(&stateOutScope), "Scope for sending states")
			("commandinscope", po::value<std::string>(&commandInscope),"Scope for receiving commands")
			("range,r",po::value<float>(&range), "Range of detection in m")
			("diffThreshold",po::value<float>(&diffThreshold), "Difference threshold in m");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(),vm);

	// first, process the help option
	if (vm.count("help")) {
		std::cout << options << "\n";
		exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// Register
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan> > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan>());
	rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);

	// ---------------- Informer ---------------------

	// Prepare RSB listener for incomming lidar scans
	rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>> lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
	lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));

	rsb::ListenerPtr commandListner = factory.createListener(commandInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	commandListner->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));

	rsb::Informer<std::string>::Ptr stateInformer = factory.createInformer<std::string>(stateOutScope);

	rst::vision::LocatedLaserScan scan, initscan, initscan_raw;

	bool enabled = false;
	bool triggered = false;

	while (true) {
		// check if the checkpoint is enabled/disabled by the stateMachine
		if (!enabled) {
			std::string command(*commandQueue->pop());
			if (command.compare("init") == 0) {
				DEBUG_MSG("init")
				enabled = true;
				// initialize scan
				initscan_raw = *lidarQueue->pop();
				initscan = initscan_raw;
				for (int i = 0; i < initscan_raw.scan_values_size(); ++i) {
					if (initscan_raw.scan_values(i) < initscan_raw.scan_values_min()) {
						initscan_raw.set_scan_values(i,10000);
					}
				}
				// filter initial scan
				for (int i = 1; i < initscan.scan_values_size()-1; ++i) {
					initscan.set_scan_values(i,min(initscan_raw.scan_values(i-1),min(initscan_raw.scan_values(i),initscan_raw.scan_values(i+1))));
					if (initscan.scan_values(i) == 10000) {
						initscan.set_scan_values(i,0);
					}
				}
			}
		} else {
			if (!commandQueue->empty()) {
				std::string command(*commandQueue->pop());
				if (command.compare("stop") == 0) {
					enabled = false;
				}
			}
		}

		// Fetch a new scan and store it to scan
		scan = *lidarQueue->pop();

		// check if the waypoint is triggered
		bool triggered_new = false;
		for (int i = 1; i < scan.scan_values_size()-1; ++i) {
			if ((scan.scan_values(i) > scan.scan_values_min() && initscan.scan_values(i) > initscan.scan_values_min() && scan.scan_values(i) < range
					&& initscan.scan_values(i) - scan.scan_values(i)> diffThreshold)
					|| (initscan.scan_values(i) < initscan.scan_values_min() && scan.scan_values(i) > scan.scan_values_min() && scan.scan_values(i) < range)) {
				//DEBUG_MSG(abs(scan.scan_values(i) - initscan.scan_values(i)));
				triggered_new = true;
				break;
			}
		}
		// send update state
		if (triggered != triggered_new) {
			if (triggered_new) {
				stateInformer->publish(
						Informer<string>::DataPtr(new string("entered")));
				DEBUG_MSG("entered");
			} else {
				stateInformer->publish(
						Informer<string>::DataPtr(new string("left")));
				DEBUG_MSG("left");
			}
			triggered = !triggered;
		}
	}

	return 0;
}

