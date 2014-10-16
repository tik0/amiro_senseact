//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the IR sensor data from a scope, then processes the data
//               and send steering commands to the act node
//============================================================================




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

// Include own converter
#include <converter/vecIntConverter/main.hpp>
#include <converter/vecFloatConverter/main.hpp>
      
using namespace std;
using namespace muroxConverter;

// The last angular velocity
float g_fLast_angVelocity_degPmm = 0;
// Scaling for the angular velocity
const float g_fAngVelocitySkaling = 0.1;
// The driving velocity
const int g_v_mmPs = 0;
// Factor for the low pass of the angular velocity [1..0)
// 1: always take the sensor values
// (1..0): take the given fraction of the sensor values and 1-fraction of the last values
const float g_LPFactor = 0.3;
// The minimum of ticks of one ir sensor, so that the behaviour takes place
const int g_iMinTicks = 100;

void braitenberg(rsb::EventPtr event, rsb::Informer< std::vector<int> >::Ptr informer_vec) {

      // Get the message
      shared_ptr<std::vector<int> > message = static_pointer_cast<std::vector<int> >(event->getData());
      
      // Calculate the new steering (two elements with 0 initialized)
      boost::shared_ptr< std::vector<int> > vecSteering(new std::vector<int> (2,0));
      
      // Velocity for left and right wheel is set in mm/s
//       vecSteering->at(0) = (int)(((float)(message->at(0) + message->at(1) + message->at(2))) / 3.0);
//       vecSteering->at(1) = (int)((float)(message->at(11) + message->at(10) + message->at(9)) / 3.0);
      
      if(message->at(11) > g_iMinTicks ||
       message->at(10) > g_iMinTicks ||
       message->at(9) > g_iMinTicks ||
       message->at(0) > g_iMinTicks ||
       message->at(1) > g_iMinTicks ||
       message->at(2) > g_iMinTicks) {
	DEBUG_MSG( "Avoidance is starting")
	float newAngVelocity_degPmm = (float)(message->at(11) + message->at(10) + message->at(9) - message->at(0) - message->at(1) - message->at(2) ) / 3.0;
	float nextAngVelocity_degPmm = g_fAngVelocitySkaling * (g_LPFactor * newAngVelocity_degPmm + (1-g_LPFactor) * g_fLast_angVelocity_degPmm);
	g_fLast_angVelocity_degPmm = nextAngVelocity_degPmm;
	
	// Velocity is set in mm/s, angular velocity is set in °/s
	vecSteering->at(0) = g_v_mmPs;
	vecSteering->at(1) = (int)nextAngVelocity_degPmm;
	
      }
	// Publish the new steering data
	informer_vec->publish(vecSteering);
}

int main (int argc, const char **argv){
  // Handle program options
  namespace po = boost::program_options;
  
  std::string rsbOutScope = "/steering";
  std::string rsbInScope = "/prox";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending the steering commands")
    ("inscope,i", po::value < std::string > (&rsbInScope), "Scope for receiving the proximity values");

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

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();
  
  // Register new converter for std::vector<int>
  shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converterVecInt);
  
  // Prepare RSB reader
  ReaderPtr reader = factory.createReader(rsbInScope);

  // Prepare RSB informer
  rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > (rsbOutScope);
    
  // Print events as they are received.
  while (true) {
    braitenberg(reader->read(), informer_vec);
  }

  return EXIT_SUCCESS;
}