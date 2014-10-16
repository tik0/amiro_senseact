//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send controls to the motor
//============================================================================




#define INFO_MSG_
// #define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// Reading inScope and outScope
// #include <ioFlags.hpp>

#include "motorControl.h"
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

using namespace std;
using namespace muroxConverter;

void sendMotorCmd(rsb::EventPtr event, motorControl* motorCtr) {
  
  // Get the message
  shared_ptr<std::vector<int> > message = static_pointer_cast<std::vector<int> >(event->getData());
  
  // Set the motor speed for testing
  // Velocity for left and right wheel is set in mm/s
//   motorCtr->setSpeedLR(message->at(0), message->at(1));
//   DEBUG_MSG( "left: " << message->at(0) << "right: " << message->at(1))
  // Velocity is set in mm/s, angular velocity is set in °/s
  motorCtr->setSpeedVW(message->at(0), message->at(1));
  DEBUG_MSG( "v: " << message->at(0) << "w: " << message->at(1))
}


int main (int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string rsbInScope = "/motor";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,o", po::value < std::string > (&rsbInScope), "Scope for receiving the motor steering commands");

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


  // Create the motor controls
  motorControl motorCtr;

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();

  // Register new converter for std::vector<int>
  shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converterVecInt);

  // Prepare RSB reader
  ReaderPtr reader = factory.createReader(rsbInScope);


  for(;;) {
    // Read the cmd and send it to the motor
    sendMotorCmd(reader->read(), &motorCtr);
  }

  return EXIT_SUCCESS;
}