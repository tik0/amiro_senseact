//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the proximity sensor data and send them to a rsb scope
//============================================================================

// #define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include "infraRed.h"
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

using namespace std;
using namespace muroxConverter;


int main (int argc, const char **argv){
  
  // Handle program options
  namespace po = boost::program_options;
  
  std::string rsbOutScope = "/prox";
  int periodMs = 125;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,i", po::value < std::string > (&rsbOutScope), "Scope for sending the proximity data")
    ("outscope,T", po::value < int > (&periodMs), "Priod in millisecounds of sending the proximity data");

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

    // Create the IR device
    InfraRed myIR;

    // Register new converter for std::vector<int>
    shared_ptr<vecIntConverter> converter(new vecIntConverter());
    converterRepository<std::string>()->registerConverter(converter);
    
    // Prepare RSB informer
    rsb::Factory& factory = rsb::Factory::getInstance();
    rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > (rsbOutScope);

    // Share the pointer to the IR data
    shared_ptr< std::vector<int> > vecData = myIR.irSensors;
    
    for(;;) {
      // Read the IR data
      myIR.getIRData();
      // Print the IR data
      // myIR.printIRData();
      // Send IR data
      informer_vec->publish(vecData);
      // Sleep for a while
      boost::this_thread::sleep( boost::posix_time::milliseconds(periodMs) );
    }

      return EXIT_SUCCESS;
}