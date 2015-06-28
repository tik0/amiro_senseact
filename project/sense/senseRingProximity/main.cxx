//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the proximity ring sensor data of AMiRo
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
#include <rsb/Version.h>
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
  
  std::string rsbOutScope = "/prox";
  uint32_t rsbPeriod = 0;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending proximity values")
    ("period,t", po::value < uint32_t > (&rsbPeriod), "Update interval (0 for maximum rate)");

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

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converter);
  
  // Prepare RSB informer
#if RSB_VERSION_NUMERIC<1200
  rsb::Factory& factory = rsb::Factory::getInstance();
#else
  rsb::Factory& factory = rsb::getFactory();
#endif
  rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > (rsbOutScope);

  // Init the CAN interface
  ControllerAreaNetwork CAN;    

  // Datastructure for the CAN messages
  std::vector< uint16_t > proximityRingValue(8,0);

  int fail = 1;
  uint8_t sensorIdx = 0;
  for(;;) {
    // Read the proximity data
    fail = CAN.getProximityRingValue(proximityRingValue);
      if (fail == 0) {
        for (sensorIdx = 0; sensorIdx < proximityRingValue.size(); sensorIdx++) {
          INFO_MSG( (int) sensorIdx << ": " << proximityRingValue[sensorIdx])
        }
        // Copy the data
        // Datastructure for the RSB messages
        boost::shared_ptr< std::vector<int> > vecData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(proximityRingValue.begin(),proximityRingValue.end()));
//         copy(proximityRingValue.begin(), proximityRingValue.end(), back_inserter(*vecData));
        // Send proximity data
        informer_vec->publish(vecData);
      } else {
        WARNING_MSG( "Fail" )
      }
      
      // Sleep for a while
      boost::this_thread::sleep( boost::posix_time::milliseconds(rsbPeriod) );
  }

  return EXIT_SUCCESS;
}