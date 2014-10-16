//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the steering data from a scope and send it to the steering scope
//               iff there is nothing on the subscope of the steeringscope which is the
//               steering command for the obstacle avoidance
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
const int g_v_mmPs = 100;
// Factor for the low pass of the angular velocity [1..0)
// 1: always take the sensor values
// (1..0): take the given fraction of the sensor values and 1-fraction of the last values
const float g_LPFactor = 0.3;

std::string rsbOutScope = "/steering/cmd";
std::string rsbInScope = "/prox";

// boost::shared_ptr< std::vector<int> > vecControl(new std::vector<int> (2,0));
// boost::shared_ptr< std::vector<int> > vecAvoidance(new std::vector<int> (2,0));
int g_iVCtrl = 0, g_iWCtrl = 0;
// Calculate the new steering (two elements with 0 initialized)
boost::shared_ptr< std::vector<int> > vecSteering(new std::vector<int> (2,0));

void fusion(rsb::EventPtr event, rsb::Informer< std::vector<int> >::Ptr informer_vec) {

      // Get the message
      shared_ptr<std::vector<int> > message = static_pointer_cast<std::vector<int> >(event->getData());
      
      // Zuweisung der scopes
       DEBUG_MSG( "CHECK " << event->getScope().toString() << " is subscope " << event->getScope().isSubScopeOf(Scope(rsbInScope)) )
       
       // The obstacle avoidance is send on any subscope of the inscope
       // while the control commands are send on the original scope
       int iVAvoidance = 0, iWAvoidance = 0;
       if (event->getScope().isSubScopeOf(Scope(rsbInScope))) {
         iVAvoidance = message->at(0);
         iWAvoidance = message->at(1);
       } else {
         g_iVCtrl = message->at(0);
         g_iWCtrl = message->at(1);
       }
      
      // Velocity is set in mm/s, angular velocity is set in °/s
      vecSteering->at(0) = g_iVCtrl + iVAvoidance;
      vecSteering->at(1) = g_iWCtrl + iWAvoidance;
      
      // Publish the new steering data
      informer_vec->publish(vecSteering);
}

int main (int argc, const char **argv){
  
  // Handle program options
  namespace po = boost::program_options;

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
      fusion(reader->read(), informer_vec);
  }

  return EXIT_SUCCESS;
}