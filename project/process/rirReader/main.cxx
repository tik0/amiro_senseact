//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Reads and generalizes the proximity ring sensor data of
//               AMiRo
//============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

#include <stdint.h>  // int32
#include <string>

#include <ControllerAreaNetwork.h>

using namespace std;
using namespace muroxConverter;

#define SENSOR_COUNT 8

/* Start offsets for AMiRo */
static int GROUND_OFFSETS[] = {2450, 2450, 2450, 2450, 2450, 2450, 2450, 2450};
static int AIR_OFFSETS[] = {2290, 2290, 2290, 2290, 2290, 2290, 2290, 2290};

string irObstacleOffsetFile = "/home/root/initial/irConfig.conf";
string irAirOffsetFile = "/home/root/initial/irEmpty.conf";

string rsbOutScopeOriginal = "/rir_prox/original";
string rsbOutScopeObstacle = "/rir_prox/obstacle";
string rsbOutScopeGround = "/rir_prox/ground";
string rsbInScopeNewConfig = "/rir_prox/newconfig";
string rsbOutScopeNewConfigRes = "/rir_prox/newconfigResponse";

string COMMAND_DONE = "DONE";
string COMMAND_ERROR = "ERROR";


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

bool loadIrConfig(std::string configPath, bool isObstacleConfig) {
  // set offset type
  string setOffsetType;
  if (isObstacleConfig) {
    setOffsetType = "ground";
  } else {
    setOffsetType = "air";
  }

  INFO_MSG("Read '" << configPath << "' (for " << setOffsetType << " offsets):");

  // try to open file
  char input[100];
  bool allDone = false;
  FILE *irConfig = fopen(configPath.c_str(), "r");

  // check if file could be loaded
  if (irConfig) {

    // read file
    fgets(input, 100, irConfig);
    vector<std::string> parts;
    splitString(std::string(input), parts, "\t");

    // check if there are values for each sensor
    if (parts.size() != SENSOR_COUNT) {
      ERROR_MSG("There are " << SENSOR_COUNT << " offsets needed, but " << parts.size() << " offsets found in '" << configPath << "'!");
    } else {

      // check if each value is an integer value
      int testOffsets[SENSOR_COUNT];
      allDone = true;
      for (int part=0; part<parts.size(); part++) {
        testOffsets[part] = atoi(std::string(parts[part]).c_str());
        string errorMsg = "";
        string inputPart = std::to_string(testOffsets[part]);
        if (testOffsets[part] <= 0) {
          allDone = false;
          errorMsg = " (error value!)";
          inputPart = parts[part];
        }
        INFO_MSG(" " << (part+1) << ") " << inputPart << errorMsg);
      }
      if (allDone) {
        // set new offsets
        for (int part=0; part<parts.size(); part++) {
          if (isObstacleConfig) {
            GROUND_OFFSETS[part] = testOffsets[part];
          } else {
            AIR_OFFSETS[part] = testOffsets[part];
          }
        } 
        return true;
      } else {
        ERROR_MSG("The offsets aren't integer values!");
      }
    }
  } else {
    ERROR_MSG("Coudn't load file '" << configPath << "'!");
  }
  WARNING_MSG("Continuing using old offsets!");
  return false;
}

void justReadValues(std::string rsbOutScopeObstacle, std::string rsbOutScopeGround, uint32_t rsbPeriod, bool print) {

  rsb::Factory& factory = rsb::Factory::getInstance();

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converter);
  
  // Prepare RSB informer
  rsb::Informer< std::vector<int> >::Ptr informerOriginalValues = factory.createInformer< std::vector<int> > (rsbOutScopeOriginal);
  rsb::Informer< std::vector<int> >::Ptr informerObstacleValues = factory.createInformer< std::vector<int> > (rsbOutScopeObstacle);
  rsb::Informer< std::vector<int> >::Ptr informerGroundValues = factory.createInformer< std::vector<int> > (rsbOutScopeGround);
  rsb::Informer<std::string>::Ptr informerCommandResponse = factory.createInformer<std::string> (rsbOutScopeNewConfigRes);

  // Create and start the command listener
  rsb::ListenerPtr listener = factory.createListener(rsbInScopeNewConfig);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
  listener->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(commandQueue)));

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
          INFO_MSG((int)sensorIdx << ": " << proximityRingValue[sensorIdx] << " - " << obstacleValues[sensorIdx] << " - " << groundValuesFac[sensorIdx]);
        }
      }

      // Datastructure for the RSB messages
      boost::shared_ptr< std::vector<int> > vecDataOriginal = boost::shared_ptr<std::vector<int> >(new std::vector<int>(proximityRingValue.begin(),proximityRingValue.end()));
      boost::shared_ptr< std::vector<int> > vecDataObstacle = boost::shared_ptr<std::vector<int> >(new std::vector<int>(obstacleValues.begin(),obstacleValues.end()));
      boost::shared_ptr< std::vector<int> > vecDataGround = boost::shared_ptr<std::vector<int> >(new std::vector<int>(groundValuesFac.begin(),groundValuesFac.end()));

      // Send proximity data
      informerOriginalValues->publish(vecDataOriginal);
      boost::this_thread::sleep(boost::posix_time::milliseconds(2));
      informerObstacleValues->publish(vecDataObstacle);
      boost::this_thread::sleep(boost::posix_time::milliseconds(2));
      informerGroundValues->publish(vecDataGround);
    }

    if (!commandQueue->empty()) {
      std::string newConfig = *commandQueue->pop().get();
      INFO_MSG("New Config Path received: " << newConfig);
      bool done = loadIrConfig(newConfig, true);
      if (done) {
        boost::shared_ptr<std::string> StringPtr(new std::string(COMMAND_DONE));
        informerCommandResponse->publish(StringPtr);
      } else {
        boost::shared_ptr<std::string> StringPtr(new std::string(COMMAND_ERROR));
        informerCommandResponse->publish(StringPtr);
      }
    }

    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(rsbPeriod) );
  }

}



int main(int argc, char **argv) {

  // Handle program options
  namespace po = boost::program_options;
  uint32_t rsbPeriod = 0;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscopeOriginal,o", po::value<std::string> (&rsbOutScopeOriginal), "Scope for sending original proximity values.")
    ("outscopeObstacle,e", po::value<std::string> (&rsbOutScopeObstacle), "Scope for sending generalized proximity values for the obstacle model.")
    ("outscopeGround,g", po::value<std::string> (&rsbOutScopeGround), "Scope for sending generalized proximity values the egde model.")
    ("inscopeCommand,i", po::value<std::string> (&rsbInScopeNewConfig), "Scope for receiving new configuration path command.")
    ("outscopeResponse,r", po::value<std::string> (&rsbOutScopeNewConfigRes), "Scope for sending command response.")
    ("period,t", po::value<uint32_t> (&rsbPeriod), "Update interval in milliseconds (0 for maximum rate).")
    ("print,p", "Prints read proximity values in the console.")
    ("loadOffsetFile,l", po::value<std::string> (&irObstacleOffsetFile), "File name for the obstacle offsets (on deafult: /root/initial/irConfig.conf).");

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

  // print parameter input
  INFO_MSG("Scopes for sending proximity sensor values:");
  INFO_MSG(" - original: " << rsbOutScopeOriginal);
  INFO_MSG(" - obstacle: " << rsbOutScopeObstacle);
  INFO_MSG(" - edge:     " << rsbOutScopeGround);
  INFO_MSG("");
  INFO_MSG("Actual offset configuration files:");
  INFO_MSG(" - ground offsets: " << irObstacleOffsetFile);
  INFO_MSG(" - air offsets:    " << irAirOffsetFile);
  INFO_MSG("");
  INFO_MSG("Scopes for setting new ground offsets:");
  INFO_MSG(" - command scope:  " << rsbInScopeNewConfig);
  INFO_MSG(" - response scope: " << rsbOutScopeNewConfigRes);
  INFO_MSG("");

  // loading configurations
  loadIrConfig(irObstacleOffsetFile, true);
  loadIrConfig(irAirOffsetFile, false);
  
  // loop
  justReadValues(rsbOutScopeObstacle, rsbOutScopeGround, rsbPeriod, vm.count("print"));

  return EXIT_SUCCESS;
}
