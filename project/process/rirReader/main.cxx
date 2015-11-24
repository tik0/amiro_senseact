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
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

#include <stdint.h>  // int32
#include <string>

#include <ControllerAreaNetwork.h>

using namespace std;
using namespace muroxConverter;
// old offsets
//static int IR_OFFSETS[] = {2481,2471,2494,2379,2421,2351,2477,2404};

/* Offsets for AMiRo 9 */
//static int IR_OFFSETS[] = {2406,2488,2290,2371,2342,2524,2423,2507};
/* Offsets for AMiRo 2 */
//static int IR_OFFSETS[] = {2500,2356,2433,2529,2507,2426,2450,2337};

/* Offsets for AMiRo 3 */
//static int IR_OFFSETS[] = {2503,2354,2433,2536,2509,2431,2448,2343};
/* Offsets for AMiRo 14 */
//static int IR_OFFSETS[] = {2503,2354,2433,2536,2509,2431,2448,2343};

/* Offsets for AMiRo 11 */
//static int IR_OFFSETS[] = {2568, 2502, 2467, 2415, 2444, 2424, 2340, 2342};
/* Offsets for AMiRo 13 */
//static int IR_OFFSETS[] = {2387, 2360, 2524, 2313, 2533, 2363, 2477, 2578};
/* Offsets for AMiRo 14 */
//static int IR_OFFSETS[] = {2494, 2347, 2427, 2501, 2494, 2410, 2444, 2331};

/* Offsets for AMiRo 36 */
static int GROUND_OFFSETS[] = {2437, 2463, 2483, 2496, 2457, 2443, 2508, 2352};
static int AIR_OFFSETS[] = {2213, 2316, 2341, 2329, 2331, 2290, 2335, 2152};

void justReadValues(std::string rsbOutScopeObstacle, std::string rsbOutScopeGround, uint32_t rsbPeriod, bool print) {

  rsb::Factory& factory = rsb::Factory::getInstance();

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converter);
  
  // Prepare RSB informer
  rsb::Informer< std::vector<int> >::Ptr informerObstacleValues = factory.createInformer< std::vector<int> > (rsbOutScopeObstacle);
  rsb::Informer< std::vector<int> >::Ptr informerGroundValues = factory.createInformer< std::vector<int> > (rsbOutScopeGround);

  // Init the CAN interface
  ControllerAreaNetwork CAN;

  // Datastructure for the CAN messages
  //std::vector<uint16_t> prvRead(8,0);
  std::vector<uint16_t> proximityRingValue(8,0);
  std::vector<uint16_t> obstacleValues(8,0);
  std::vector<uint16_t> groundValues(8,0);
  std::vector<uint16_t> groundValuesFac(8,0);

  int fail = 1;
  uint8_t sensorIdx = 0;
  for(;;) {
    // Read the proximity data
    fail = CAN.getProximityRingValue(proximityRingValue);

    if (fail == 0) {
      // claculate offsets in proximity values
      for (sensorIdx = 0; sensorIdx < 8; sensorIdx++) {
        /* turn order
        proximityRingValue[0] = prvRead[7];
        for (int i = 1; i < 8; i++) {
            proximityRingValue[i] = prvRead[i-1];
        }
        */

        // calculate obstacle values
        if (proximityRingValue[sensorIdx] <= GROUND_OFFSETS[sensorIdx]) {
          obstacleValues[sensorIdx] = 0;
        } else {
          obstacleValues[sensorIdx] = proximityRingValue[sensorIdx] - GROUND_OFFSETS[sensorIdx];
        }

        // calculate ground values
        if (proximityRingValue[sensorIdx] >= GROUND_OFFSETS[sensorIdx]) {
          groundValues[sensorIdx] = GROUND_OFFSETS[sensorIdx] - AIR_OFFSETS[sensorIdx];
        } else if (proximityRingValue[sensorIdx] < AIR_OFFSETS[sensorIdx]) {
          groundValues[sensorIdx] = 0;
        } else {
          groundValues[sensorIdx] = proximityRingValue[sensorIdx] - AIR_OFFSETS[sensorIdx];
        }
        groundValuesFac[sensorIdx] = (uint16_t)(((float)groundValues[sensorIdx]) / ((float)(GROUND_OFFSETS[sensorIdx] - AIR_OFFSETS[sensorIdx])) * 10000.0);

      }

      // print proximity data
      if (print) {
        for (sensorIdx = 0; sensorIdx < 8; sensorIdx++) {
          INFO_MSG((int)sensorIdx << ": " << obstacleValues[sensorIdx] << "/" << groundValues[sensorIdx] << " (" << groundValuesFac[sensorIdx] << ")");
        }
      }

      // Datastructure for the RSB messages
      boost::shared_ptr< std::vector<int> > vecDataObstacle = boost::shared_ptr<std::vector<int> >(new std::vector<int>(obstacleValues.begin(),obstacleValues.end()));
      boost::shared_ptr< std::vector<int> > vecDataGround = boost::shared_ptr<std::vector<int> >(new std::vector<int>(groundValuesFac.begin(),groundValuesFac.end()));

      // Send proximity data
      informerObstacleValues->publish(vecDataObstacle);
      boost::this_thread::sleep(boost::posix_time::milliseconds(2));
      informerGroundValues->publish(vecDataGround);
    }

    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(rsbPeriod) );
  }

}


void splitString(const std::string &str, vector<std::string> &parts, const std::string delimiters) {
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos) {
        // Found a token, add it to the vector.
        parts.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}



int main(int argc, char **argv) {

  // Handle program options
  namespace po = boost::program_options;
  
  std::string rsbOutScopeObstacle = "/rir_prox/obstacle";
  std::string rsbOutScopeGround = "/rir_prox/ground";
  uint32_t rsbPeriod = 0;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscopeObstacle,o", po::value < std::string > (&rsbOutScopeObstacle), "Scope for sending generalized proximity values for the obstacle model.")
    ("outscopeGround,g", po::value < std::string > (&rsbOutScopeGround), "Scope for sending generalized proximity values the egde model.")
    ("period,t", po::value < uint32_t > (&rsbPeriod), "Update interval in milliseconds (0 for maximum rate).")
    ("print,p", "Prints read proximity values")
    ("loadOffsets,l", "Loads offsets from the files 'irConfig.conf' and 'irEmpty.conf'.");

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

  if (vm.count("loadOffsets")) {
    char input[100];
    FILE *irConfig = fopen("irConfig.conf", "r");
    if (irConfig) {
      fgets(input, 100, irConfig);
      INFO_MSG("Read 'irConfig.conf':");
      vector<std::string> parts;
      splitString(std::string(input), parts, "\t");
      for (int part=0; part<parts.size(); part++) {
        GROUND_OFFSETS[part] = atoi(std::string(parts[part]).c_str());
        INFO_MSG(" " << (part+1) << ") " << GROUND_OFFSETS[part]);
      }
    } else {
      WARNING_MSG("Coudn't load 'irConfig.conf'! Now using standard offsets.");
    }
    irConfig = fopen("irEmpty.conf", "r");
    if (irConfig) {
      fgets(input, 100, irConfig);
      INFO_MSG("Read 'irEmpty.conf':");
      vector<std::string> parts;
      splitString(std::string(input), parts, "\t");
      for (int part=0; part<parts.size(); part++) {
        AIR_OFFSETS[part] = atoi(std::string(parts[part]).c_str());
        INFO_MSG(" " << (part+1) << ") " << AIR_OFFSETS[part]);
      }
    } else {
      WARNING_MSG("Coudn't load 'irEmpty.conf'! Now using standard offsets.");
    }
  }

  justReadValues(rsbOutScopeObstacle, rsbOutScopeGround, rsbPeriod, vm.count("print"));


  return EXIT_SUCCESS;
}
