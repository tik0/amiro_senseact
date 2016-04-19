
//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send controls to the motor of AMiRo
//               Velocity are received in mm/s via RSB
//               Angular velocity are received in °/s via RSB
//============================================================================

#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/misc/SignalWaiter.h>
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
  CAN.setTargetSpeed(int(message->at(0)), int(message->at(1)));
  DEBUG_MSG( "v: " << message->at(0) << "w: " << message->at(1))
}

static std::string rsbInScope = "/motor";
static int staticVelocity_um_s = 0;
static int staticAngularVelocity_urad_s = 0;
static bool useStaticVelocities = false;
static bool sendStaticVelocitiesOnce = false;
static int sendStaticVelocitiesPeriod_us = 100 /*ms*/ * 1e3;

int main (int argc, const char **argv){
 
  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,i", po::value < std::string > (&rsbInScope), "Scope for receiving the motor steering commands")
    ("staticVelocity", po::value < int > (&staticVelocity_um_s), "Static forward velocity in µm/s")
    ("staticAngularVelocity", po::value < int > (&staticAngularVelocity_urad_s), "Static angular velocity in µrad/s")
    ("useStaticVelocities", po::bool_switch(&useStaticVelocities)->default_value(false), "Make use of the given velocities, instead of using spread")
    ("sendStaticVelocitiesOnce", po::bool_switch(&sendStaticVelocitiesOnce)->default_value(false), "Send the static velocity command once")
    ("sendStaticVelocitiesPeriod", po::value < int > (&sendStaticVelocitiesPeriod_us), "Period between static velocity sending in µs");


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

  // STATIC BEHAVIOR
  rsc::misc::initSignalWaiter();
  if ( useStaticVelocities ) {
    while (rsc::misc::lastArrivedSignal() != rsc::misc::INTERRUPT_REQUESTED && rsc::misc::lastArrivedSignal() != rsc::misc::TERMINATE_REQUESTED) {
      CAN.setTargetSpeed(staticVelocity_um_s, staticAngularVelocity_urad_s);
      if (sendStaticVelocitiesOnce) {
        break;
      }
      usleep(sendStaticVelocitiesPeriod_us);
    }
    return 0;
  }


  // RSB BEHAVIOUR
  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converterVecInt);

  // Prepare RSB reader
  ReaderPtr reader = factory.createReader(rsbInScope);

  while (rsc::misc::lastArrivedSignal() != rsc::misc::INTERRUPT_REQUESTED && rsc::misc::lastArrivedSignal() != rsc::misc::TERMINATE_REQUESTED) {
          // Wait for the message
          sendMotorCmd(reader->read(), CAN);
  }

  return EXIT_SUCCESS;
}
