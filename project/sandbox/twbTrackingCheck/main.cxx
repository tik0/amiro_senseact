//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Prints tracking data.
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
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/Event.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>
using namespace rsb;

// types
#include <types/enum.pb.h>
#include <types/loc.pb.h>
#include <types/pose.pb.h>
#include <types/rotation.pb.h>
#include <types/shapes.pb.h>
#include <types/vertex.pb.h>

#include <Types.h>

// unit calculations
#define TO_MICRO 1000000
#define MICRO_TO 0.000001
#define TRACKING_TIMEOUT 5 // s

// constants
int markerID = 0;
float meterPerPixel = 1.0; // m/pixel


// scopenames for rsb
std::string trackingInscope = "/tracking/merger";



float normAngle(float angle) {
	float normed = fmod(angle, 2.0*M_PI);
	if (normed < 0) normed += 2.0*M_PI;
	return normed;
}


types::position readTracking(boost::shared_ptr<twbTracking::proto::ObjectList> data, int trackingMarkerID) {
	twbTracking::proto::Pose pose2D;
	for (int i = 0; i < data->object_size(); i++) {
		if (trackingMarkerID == data->object(i).id()) {
			pose2D = data->object(i).position();
			break;
		}
	}
	types::position pos;
	pos.x = pose2D.translation().x()*meterPerPixel*TO_MICRO;
	pos.y = pose2D.translation().y()*meterPerPixel*TO_MICRO;
	pos.f_z = normAngle(pose2D.rotation().z())*TO_MICRO;
	return pos;
}


bool isBeingTracked(boost::shared_ptr<twbTracking::proto::ObjectList> data, int trackingID) {
	bool isIn = false;
	for (int i = 0; i < data->object_size(); i++) {
		if (trackingID == data->object(i).id()) {
			if (data->object(i).position().translation().x() != 0 && data->object(i).position().translation().y() != 0) {
				isIn = true;
			}
			break;
		}
	}
	return isIn;
}

types::position getNextTrackingPos(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>> trackingQueue) {
	boost::shared_ptr<twbTracking::proto::ObjectList> data;
	int restTime = TRACKING_TIMEOUT * 1000; // ms
	do {
		if (!trackingQueue->empty()) {
			data = boost::static_pointer_cast<twbTracking::proto::ObjectList>(trackingQueue->pop());
			if (isBeingTracked(data, markerID)) {
				break;
			}
		} else if (restTime <= 0) {
			types::position errorPos;
			errorPos.x = 0;
			errorPos.y = 0;
			errorPos.f_z = -1;
			ERROR_MSG("Marker could not be detected in the last " << TRACKING_TIMEOUT << " seconds!");
			return errorPos;
		} else {
			// sleep for 10 ms
			usleep(10000);
			restTime -= 10;
		}
	} while (true);
	return readTracking(data, markerID);
}


int main(int argc, char **argv) {
	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("markerId,i", po::value<int>(&markerID), "ID of the marker for robot detection.")
			("mmp,m", po::value<float>(&meterPerPixel), "Meter per pixel of the robot detection.");

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

	if (!vm.count("markerId") && !vm.count("mmp")) {
		std::cout << "Please set the marker ID and the meter per pixel factor!\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	} else if (!vm.count("markerId")) {
		std::cout << "Please set the marker ID!\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	} else if (!vm.count("mmp")) {
		std::cout << "Please set the meter per pixel factor!\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	}

	// +++++ RSB Initalization +++++

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// ------------ Converters ----------------------

	// Register new converter for ObjectList
	rsb::converter::ProtocolBufferConverter<twbTracking::proto::ObjectList>::Ptr converterObjectList(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::ObjectList>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converterObjectList);	

	// ------------ ExtSpread Config ----------------

	// CREATE A CONFIG TO COMMUNICATE WITH ALPIA.TECHFAK.UNI-BIELEFELD
	// Get the global participant config as a template
	rsb::ParticipantConfig trackingPartConf = factory.getDefaultParticipantConfig(); {
		// disable socket transport
		rsc::runtime::Properties trackingPropSocket  = trackingPartConf.mutableTransport("socket").getOptions();
		trackingPropSocket["enabled"] = boost::any(std::string("0"));

		// Get the options for spread transport, because we want to change them
		rsc::runtime::Properties trackingPropSpread  = trackingPartConf.mutableTransport("spread").getOptions();

		// enable socket transport
		trackingPropSpread["enabled"] = boost::any(std::string("1"));

		// Change the config
		trackingPropSpread["host"] = boost::any(std::string("alpia.techfak.uni-bielefeld.de"));

		// Change the Port
		trackingPropSpread["port"] = boost::any(std::string("4803"));

		// Write the tranport properties back to the participant config
		trackingPartConf.mutableTransport("socket").setOptions(trackingPropSocket);
		trackingPartConf.mutableTransport("spread").setOptions(trackingPropSpread);
	}

	// ------------ Listener ---------------------

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, trackingPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::ObjectList>(trackingQueue)));

	while(true) {
		types::position pos = getNextTrackingPos(trackingQueue);
		if (pos.f_z < 0) return EXIT_FAILURE;

		INFO_MSG("Current Position:");
		INFO_MSG(" x: " << (pos.x*MICRO_TO) << " m");
		INFO_MSG(" y: " << (pos.y*MICRO_TO) << " m");
		INFO_MSG(" Ø: " << (pos.f_z*MICRO_TO) << "°");

		usleep(500000);
	}

	return EXIT_SUCCESS;
}
