//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send controls to the motor of AMiRo
//               Velocity are received in mm/s via RSB
//               Angular velocity are received in °/s via RSB
//============================================================================
// #

#define SEND_ODO
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>
#include <types/twbTracking.pb.h>
#include <ControllerAreaNetwork.h>

using namespace std;
using namespace muroxConverter;

#include <Types.h>
using namespace amiro;

int main(int argc, const char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	std::string rsbOutScope = "/odometrydata";

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("outscope,o",
			po::value<std::string>(&rsbOutScope),
			"Scope for sending odometrydata");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store(
			po::command_line_parser(argc, argv).options(options).positional(p).run(),
			vm);

	// first, process the help option
	if (vm.count("help")) {
		std::cout << options << "\n";
		exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	// Create the CAN interface
	ControllerAreaNetwork CAN;

	types::position robotPosition;
	robotPosition.x = 0;
	robotPosition.y = 0;
	robotPosition.f_z = 0;
	CAN.setOdometry(robotPosition);

	boost::shared_ptr<twbTracking::proto::Pose2D> poseWrapper(new twbTracking::proto::Pose2D);

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	boost::shared_ptr<
			rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(
			new rsb::converter::ProtocolBufferConverter<
					twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(
			converterlocalization);
	// Prepare RSB informer

	rsb::Informer<twbTracking::proto::Pose2D>::Ptr informer_vec =
			factory.createInformer<twbTracking::proto::Pose2D>(rsbOutScope);

	for (;;) {
		// Wait for the message

		robotPosition = CAN.getOdometry();

		poseWrapper->set_x((float) robotPosition.x);
		poseWrapper->set_y((float) robotPosition.y);
		poseWrapper->set_orientation((float) robotPosition.f_z);

		informer_vec->publish(poseWrapper);

		//INFO_MSG(" x: " << robotPosition.x << " y: " << robotPosition.y << " f_z: " << robotPosition.f_z);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

	return EXIT_SUCCESS;
}
