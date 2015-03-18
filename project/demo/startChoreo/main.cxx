//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Statemachine to perform a choreography.
//============================================================================

#define SIMULATION

#include <iostream>
#include <string>
using namespace std;

// boost
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
namespace po = boost::program_options;
using namespace boost::chrono;

// rsb
#include <rsb/Factory.h>
#include <rsb/Informer.h>
using namespace rsb;

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string choreoOutscope = "/choreo";

	std::string choreoName = "testChoreo.xml";

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
		("name,n",po::value<std::string>(&choreoName),"Name of the choreography.")
			("choreoOut", po::value<std::string>(&choreoOutscope),"Choreography outscope.");

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

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// prepare RSB informer for choreos
	Informer<string>::Ptr choreoInformer = factory.createInformer<string>(choreoOutscope);

	// publish the data
	Informer<string>::DataPtr message(new string(choreoName));
	choreoInformer->publish(message);

	return EXIT_SUCCESS;
}
