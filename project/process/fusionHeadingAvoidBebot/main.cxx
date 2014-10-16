//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the steering data from a scope and send it to the motor
//               iff the values of the proximity sensors are not to high
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
      
using namespace std;
using namespace muroxConverter;

// The last steering commands
int g_iVCtrl = 0, g_iWCtrl = 0;
int g_iIRDataFrontLeft = 0, g_iIRDataFrontRight = 0;
// Calculate the new steering (two elements with 0 initialized)
boost::shared_ptr< std::vector<int> > vecSteering(new std::vector<int> (2,0));
rsb::Informer< void >::Ptr informerHeadingStop;
// The IR-Data ticks, where the heading needs to stop
int g_iIRDataUpperLimit = 500;

std::string g_sInScope_Steering = "/proc/steering/raw";
std::string g_sOutScope_Steering = "/act/steering";
std::string g_sOutScope_HeadingStop = "/heading/stop";

void fusion(rsb::EventPtr event, rsb::Informer< std::vector<int> >::Ptr informer_vec) {

      // Get the message
      shared_ptr<std::vector<int> > message = static_pointer_cast<std::vector<int> >(event->getData());
      
      // Zuweisung der scopes
       DEBUG_MSG( "CHECK " << event->getScope().toString() << " is subscope " << event->getScope().isSubScopeOf(Scope(g_sInScope_Steering)) )
       
       // The IR data is send on any subscope
       // while the control commands are send on the original scope
       if (event->getScope().isSubScopeOf(Scope(g_sInScope_Steering))) {
	 g_iIRDataFrontLeft = message->at(0);
	 g_iIRDataFrontRight = message->at(11);
       } else {
	 g_iVCtrl = message->at(0);
	 g_iWCtrl = message->at(1);
       }
      
      // If the IR-Data is to high then stop, otherwise
      if (g_iIRDataFrontLeft > g_iIRDataUpperLimit || g_iIRDataFrontRight > g_iIRDataUpperLimit) {
	// ... but if you want to go back, then you are allowed to do so
	if ( g_iVCtrl < 0) {
	  vecSteering->at(0) = g_iVCtrl;
	  vecSteering->at(1) = g_iWCtrl;
	} else {
	  vecSteering->at(0) = 0;
	  vecSteering->at(1) = 0;
	  boost::shared_ptr< void > voidData(new int(0));
	  informerHeadingStop->publish(voidData);
	}
      } else {
	vecSteering->at(0) = g_iVCtrl;
	vecSteering->at(1) = g_iWCtrl;
      }
      
      // Publish the new steering data
      informer_vec->publish(vecSteering);
}

int main (int argc, const char **argv){

   namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("inscopeSteering,i", po::value < std::string > (&g_sInScope_Steering), "Scope for receiving steering commands.")
            ("outscopeSteering,o", po::value < std::string > (&g_sOutScope_Steering),"Scope for sending fused steering commands.")
            ("outscopeHeadingStop,h", po::value < std::string > (&g_sOutScope_HeadingStop), "Sending void to this scope, if the obstacle avoidance stops the BeBot.");
 
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
    
    // Get the RSB factory
    rsb::Factory& factory = rsb::Factory::getInstance();
    
    // Register new converter for std::vector<int>
    shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
    converterRepository<std::string>()->registerConverter(converterVecInt);
          
    // Prepare RSB reader
    ReaderPtr reader = factory.createReader(g_sInScope_Steering);

    // Prepare RSB informer
    rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > (g_sOutScope_Steering);
    informerHeadingStop = factory.createInformer< void > (g_sOutScope_HeadingStop);
    

    // Print events as they are received.
    while (true) {
        fusion(reader->read(), informer_vec);
    }

      return EXIT_SUCCESS;
}