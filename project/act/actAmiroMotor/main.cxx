
//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send controls to the motor of AMiRo
//               Velocity are received in mm/s via RSB
//               Angular velocity are received in °/s via RSB
//============================================================================

// #define INFO_MSG_
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

// Include own converter
#include <converter/vecIntConverter/main.hpp>

#include <ControllerAreaNetwork.h>

using namespace std;
using namespace muroxConverter;

void sendMotorCmd(rsb::EventPtr event, ControllerAreaNetwork &CAN) {
  
  // Get the message
  boost::shared_ptr<std::vector<int> > message = boost::static_pointer_cast<std::vector<int> >(event->getData());
  
  // Set the motor speed
  // Velocity send in µm/s
  // Angular velocity send in µrad/s
  CAN.setTargetSpeed(message->at(0), int(message->at(1)));
  DEBUG_MSG( "v: " << message->at(0) << "w: " << message->at(1))
}


int main (int argc, const char **argv){
 
  // Handle program options
  namespace po = boost::program_options;
  
  std::string rsbInScope = "/motor";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,i", po::value < std::string > (&rsbInScope), "Scope for receiving the motor steering commands");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  // Create the CAN interface
  ControllerAreaNetwork CAN;
  
  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converterVecInt);

  // Prepare RSB reader
  ReaderPtr reader = factory.createReader(rsbInScope);

  for(;;) {
          // Wait for the message
          sendMotorCmd(reader->read(), CAN);
  }

  return EXIT_SUCCESS;
}
