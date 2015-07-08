//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the proximity floor sensor data of AMiRo
//============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

using namespace std;
using namespace muroxConverter;

int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	std::string rsbOutScope = "/prox/floor";
	uint32_t rsbPeriod = 0;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("verbose,v", "Print values.")("outscope,o",
			po::value<std::string>(&rsbOutScope), "Scope for sending proximity values")("period,t",
			po::value<uint32_t>(&rsbPeriod), "Update interval (0 for maximum rate)");

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

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
	converterRepository<std::string>()->registerConverter(converter);

	// Prepare RSB informer
	rsb::Factory& factory = rsb::Factory::getInstance();
	rsb::Informer<std::vector<int> >::Ptr floorProxInformer = factory.createInformer<std::vector<int> >(rsbOutScope);

	// Init the CAN interface
	ControllerAreaNetwork CAN;

	// Datastructure for the CAN messages
	std::vector<uint16_t> floorProxValues(4, 0);

	while (true) {
		// Read the proximity data
		if (CAN.getProximityFloorValue(floorProxValues) == 0) {
			// Datastructure for the RSB messages
			boost::shared_ptr<std::vector<int> > floorProxData = boost::shared_ptr<std::vector<int> >(
					new std::vector<int>(floorProxValues.begin(), floorProxValues.end()));

			// Send proximity data
			floorProxInformer->publish(floorProxData);

			// Print proximity data
			if (vm.count("verbose")) {
				for (int i : *floorProxData) {
					cout << i << " ";
				}
				cout << endl;
			}

			// Sleep for a while
			boost::this_thread::sleep(boost::posix_time::milliseconds(rsbPeriod));
		} else
			WARNING_MSG("Reading floor proximity data from CAN failed.");
	}

	return EXIT_SUCCESS;
}
