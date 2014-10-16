//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the proximity sensor data of AMiRo
//============================================================================




#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

// Reading inScope and outScope
// #include <ioFlags.hpp>

// Reading inScope, outScope and freq
#include <ofFlags.hpp>

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

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>


using namespace std;
using namespace muroxConverter;

int main(int argc, char **argv) {
  
  // Handle program options
  namespace po = boost::program_options;
  
  std::string rsbOutScope = "/prox";
  uint32_t rsbPeriod = 125;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending proximity values")
    ("period,t", po::value < uint32_t > (&rsbPeriod), "Update interval (0 for maximum rate)");;

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
  rsb::Factory& factory = rsb::Factory::getInstance();
  rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > (rsbOutScope);

  // Init the CAN interface
  ControllerAreaNetwork myCAN;    
  
  // Datastructure for the CAN messages
  std::vector< uint16_t > prox(8,0);
  
  // Share the pointer to the IR data
//      boost::shared_ptr< std::vector<int> > vecData;
  
  for(;;) {
    // Read the IR data
    int fail = myCAN.getProximityRingValue(prox);
      if (fail == 0) {
        for (int sensorIdx = 0; sensorIdx < prox.size(); sensorIdx++) {
          INFO_MSG( prox[sensorIdx] << " " << 1 - float(prox[sensorIdx]) / float(0xFFFFu)  << " " << 100.0f * ((1.0f - 1.0f / sqrt(float(prox[sensorIdx]))) - 0.979f) / 0.0170938 )
          if ( prox[sensorIdx] != 0)
            prox[sensorIdx] = 4.0f * 100.0f * (((1.0f - 1.0f / sqrt(float(prox[sensorIdx]))) - 0.979f) / 0.0170938f );
//                prox[sensorIdx] = 50;
        }
      } else {
        WARNING_MSG( "Fail" )
      }
      INFO_MSG( "-------" )
    // Copy the data
//        vecData->assign(prox.begin(),prox.end());
//          std::vector<float> v_float(v_int.begin(), v_int.end());
      boost::shared_ptr< std::vector<int> > vecData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(prox.begin(), prox.end()));
    // Send IR data
    informer_vec->publish(vecData);
    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(rsbPeriod) );
  }

      return EXIT_SUCCESS;
}
