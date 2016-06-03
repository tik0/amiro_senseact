//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : -
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
#include <actModels/lightModel.h>
#include <extspread.hpp>

using namespace rsb;
using namespace rsb::patterns;


// scopenames and spread data for rsb
std::string commandOutscope = "/amiro/lights";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4803";


// main function
int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	int commandType, colorRed, colorGreen, colorBlue, periodTime;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("commandScope,c", po::value<std::string>(&commandOutscope), "Inscope for commands (default: '/amiro/lights').")
			("host,h", po::value<std::string>(&spreadhost), "Host for Programatik Spread (default: '127.0.0.1').")
			("port,p", po::value<std::string>(&spreadport), "Port for Programatik Spread (default: '4803').")
			("type,t", po::value<int>(&commandType), "Command Type")
			("red,r", po::value<int>(&colorRed), "Color Red")
			("green,g", po::value<int>(&colorGreen), "Color Green")
			("blue,b", po::value<int>(&colorBlue), "Color Blue")
			("init,i", "Set random colors")
			("period,d", po::value<int>(&periodTime), "Period Time");

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

	if ((!vm.count("type") || !vm.count("red") || !vm.count("green") || !vm.count("blue") || !vm.count("period"))
	 && (!vm.count("type") || !vm.count("init") || !vm.count("period"))) {
		std::cout << "Some parameters have to be set! Please check the options!\n\n" << options << "\n";
		exit(1);
	}

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ------------ Informer ----------------------

	rsb::Informer< std::vector<int> >::Ptr commandInformer = factory.createInformer< std::vector<int> > (commandOutscope);


	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	// prepare publication
	boost::shared_ptr<std::vector<int>> commandVector;
	if (!vm.count("init")) {
		std::vector<int> command(5,0);
		command[0] = commandType;
		command[1] = colorRed;
		command[2] = colorGreen;
		command[3] = colorBlue;
		command[4] = periodTime;
		commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(command.begin(),command.end()));
	} else {
/*		std::vector<int> command(26,0);
		command[0] = commandType;
		command[25] = periodTime;
		for (int led=0; led<8; led++) {
			command[led*3+1] = (int)(LightModel::initColors[led].getRed());
			command[led*3+2] = (int)(LightModel::initColors[led].getGreen());
			command[led*3+3] = (int)(LightModel::initColors[led].getBlue());
		}*/
		std::vector<int> command = LightModel::setLights2Vec(commandType, LightModel::initColors, periodTime);
		commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(command.begin(),command.end()));
	}

	// publish command
	commandInformer->publish(commandVector);


	return EXIT_SUCCESS;
}
