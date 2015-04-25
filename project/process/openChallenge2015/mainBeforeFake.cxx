// Messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// CAN
#include <ControllerAreaNetwork.h>
#include <Types.h>
#include <Constants.h>
types::position homingPosition;
ControllerAreaNetwork myCAN;

// For running system comands
#include <stdlib.h>

// For using kbhit
#include <unistd.h>

// For checking character pressed in the console
#include <kbhit.hpp>

// For using vector
#include <vector>

// For using string
#include <string>

// For program options
#include <boost/program_options.hpp>

// RSB
#include <converter/matConverter/matConverter.hpp>
#include <converter/vecIntConverter/main.hpp>
#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>



using namespace boost;
using namespace std;
using std::vector;
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// State machine states
#define NUM_STATES 14
enum states                          { idle , init , exploration , explorationFinish , objectHoming , objectDetection , objectDetectionPending , objectDeliveryHoming , objectDeliveryPushing , objectDeliveryFinish , objectSeperation , objectSeperationPending , objectHomingPending , objectDeliveryFail };
std::string statesString[NUM_STATES] {"idle","init","exploration","explorationFinish","objectHoming","objectDetection","objectDetectionPending","objectDeliveryHoming","objectDeliveryPushing","objectDeliveryFinish","objectSeperation","objectSeperationPending","objectHomingPending","objectDeliveryFail"};


// Objects && Object detection communication
#define NUM_OBJECTS 6
enum objects { object1 , object2 , object3 , object4 , object5 , object6 };
std::string objectsString[NUM_OBJECTS] = {"object1","object2","object3","object4","object5","object6"};

// Object detection
// We send a command "COMP" string  for start comparing or "ESC" for exiting the program
const std::string sObjectDetCmdScope("/objectDetection/command");
// We assume object IDs as string from "1" to "6", after sending the "comp" command
const std::string sObjectDetAnswerScope("/objectDetection/detected");

// Exploration
// We send a command "start" string  for start the exploration
const std::string sExplorationCmdScope("/exploration");
// We assume a "finish" string, which indicates, that the exploration has run clean
const std::string sExplorationAnswerScope("/explorationState");

// Delivery
// TODO
const std::string sDeliveryCmdScope("/objectDelivery");
// TODO
const std::string sDeliveryAnswerScope("/objectDeliveryState");


int processSM(void);

std::string g_sInScopeTobi = "/tobiamiroNUMBER";
std::string g_sOutScopeStateTobi = "/amiroNUMBERtobi";
std::string g_sOutScopeState = "/state";
std::string g_sRemoteServerPort = "4823";
std::string g_sRemoteServer = "localhost";

int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("spread,s", po::value < std::string > (&g_sRemoteServer), "IP of remote spread server.")
    ("spreadPort", po::value < std::string > (&g_sRemoteServerPort), "Port of remote spread server.")
    ("outscopeStateTobi", po::value < std::string > (&g_sOutScopeStateTobi), "Scope for sending the current state to tobi.")
    ("outscopeState", po::value < std::string > (&g_sOutScopeState), "Scope for sending the current state internaly.")
    ("inscopeTobi,i", po::value < std::string > (&g_sInScopeTobi), "Scope for recieving Tobis messages.");

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

  // use camera stream for testing
  return processSM();
}

int processSM(void) {

    // Create the factory
  rsb::Factory &factory = rsb::getFactory();

    //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
    ///////////////////////////////////////////////////////////////////////////////
        // Get the global participant config as a template
        rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
              {
                // Get the options for socket transport, because we want to change them
                rsc::runtime::Properties tmpProp  = tmpPartConf.mutableTransport("socket").getOptions();

                // disable socket transport
                std::string enabled = "0";
                tmpProp["enabled"] = boost::any(enabled);

                // Write the socket tranport properties back to the participant config
                tmpPartConf.mutableTransport("socket").setOptions(tmpProp);
              }
              {
                // Get the options for spread transport, because we want to change them
                rsc::runtime::Properties tmpPropSpread = tmpPartConf.mutableTransport("spread").getOptions();

                // enable socket transport
                std::string enabled = "1";
                tmpPropSpread["enabled"] = boost::any(enabled);

                // the port of the server
                tmpPropSpread["port"] = boost::any(g_sRemoteServerPort);

                // Change the server
                tmpPropSpread["host"] = boost::any(g_sRemoteServer);

                // Write the tranport properties back to the participant config
                tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);

                std:: cout << tmpPropSpread << std::endl;
              }
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

    /////////////////// LOCAL SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Object Detection: Listener and Informer
    rsb::ListenerPtr listenerObjectDetAnswerScope = factory.createListener(sObjectDetAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueObjectDetAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerObjectDetAnswerScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueObjectDetAnswerScope)));

    rsb::Informer< std::string >::Ptr informerObjectDetCmdScope = factory.createInformer< std::string > (sObjectDetCmdScope);

    // Exploration: Listener and Informer
    rsb::ListenerPtr listenerExplorationScope = factory.createListener(sExplorationAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueExplorationAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerExplorationScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueExplorationAnswerScope)));

    rsb::Informer< std::string >::Ptr informerExplorationScope = factory.createInformer< std::string > (sExplorationCmdScope);

    // Object seperation and delivery: Listener and Informer
    rsb::ListenerPtr listenerDeliveryScope = factory.createListener(sDeliveryAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueDeliveryAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerDeliveryScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueDeliveryAnswerScope)));

    rsb::Informer< std::string >::Ptr informerDeliveryScope = factory.createInformer< std::string > (sDeliveryCmdScope);

    /////////////////// REMOTE SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Prepare RSB informer and listener
    rsb::Informer< std::string >::Ptr informerRemoteState[NUM_STATES];
    rsb::Informer< std::string >::Ptr informerRemoteObject[NUM_OBJECTS];
    rsb::Informer< std::string >::Ptr informerRemoteObjectFinish[NUM_OBJECTS];

    // Create the listener
    rsb::ListenerPtr listenerRemoteTobiState;
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueRemoteTobiState(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));


    try {
      for (int idx = 0; idx < NUM_STATES; ++idx) {
        std::string sOutScopeStateTobiTmp(g_sOutScopeStateTobi);
        informerRemoteState[idx] = factory.createInformer< std::string > (sOutScopeStateTobiTmp.append("/").append(statesString[idx]), tmpPartConf);
      }
      for (int idx = 0; idx < NUM_OBJECTS; ++idx) {
        std::string sOutScopeStateTobiTmp(g_sOutScopeStateTobi);
        std::string sOutScopeStateTobiObjectFinishTmp(g_sOutScopeStateTobi);
        informerRemoteObject[idx] = factory.createInformer< std::string >       (sOutScopeStateTobiTmp.append("/").append(objectsString[idx]), tmpPartConf);
        informerRemoteObjectFinish[idx] = factory.createInformer< std::string > (sOutScopeStateTobiObjectFinishTmp.append("/").append(objectsString[idx]).append("finish"), tmpPartConf);
      }

      // Create the listener for the standby task
      listenerRemoteTobiState = factory.createListener(g_sInScopeTobi, tmpPartConf);
      listenerRemoteTobiState->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueRemoteTobiState)));
    }
    catch(std::exception& e) {
      ERROR_MSG("Remote connection not established");
      return -1;
    }

    // When we reached this point, everything should be fine


  // Just a void for sending nothing
  boost::shared_ptr< void > publishVoid(new int(0));

  // Local variables
  bool bGotStateFromTobi = false;  // True if a state from tobi was received
  bool bGotObject = false; // If an object was detected correctly
  bool bGotFalseObject = false; // If an object was not detected correctly
  bool bGotExplorationFinish = false; // Indicates, if the exploration has finished
  bool bGotHomingFinish = false; // Indicates if the homing to an object has finished
  bool bGotDeliveryFinish = false; // Indicates, if the homing has finished
  bool bGotDeliveryFail = false; // Indicates, if the delivery faile

  int  numSeperatedObjects = -1; // Number of objects seperated in the map
  bool bGotNumberOfSeperatedObjects = false; // Indicate, if the seperation succeeded

  int objectDetectionCounter = -1; // Equals numSeperatedObjects, if all seperated objects were detected
  std::vector<int> objectsDetected; // This is a lookuptable, where the index is the seperated object, and the content the detected object
  int objectDelivery = -1; // Number of object, which needs to be delivered IF FOUND

  double objDistance = 0.0, objAngular = 0.0;

  // Variables for the keypress
  // Check for keypress in winow
  int KB_codeCV = 0;
  // Check for keypress in terminal
  int KB_code = 0;

  // Strings send from tobi
  std::string sStateFromTobi = "";
  states amiroState = idle;
  boost::shared_ptr<std::string> statePublish(new std::string);
  boost::shared_ptr<std::string> tmpDummy(new std::string);

  // Process the behaviour
  while (true) {

    // Get messages from tobi
    if (!queueRemoteTobiState->empty()) {
      INFO_MSG("Received a state from Tobi")
      bGotStateFromTobi = true;
      sStateFromTobi = *queueRemoteTobiState->pop();

      // Check if the content was an object request
      for (int idx = 0; idx < NUM_OBJECTS; ++idx) {
        if ( sStateFromTobi.compare(objectsString[idx]) == 0) {
          for (int detectedObjIdx = 0; detectedObjIdx < objectsDetected.size(); ++detectedObjIdx) {
            if (idx == objectsDetected.at(detectedObjIdx)) {
              objectDelivery = idx;
              amiroState = objectDeliveryHoming;
              // HACK
              amiroState = objectDeliveryPushing;
              // HACK Finish
              break;
            }
          }
        }
      }

      INFO_MSG("Tobi state: " << sStateFromTobi)
      // Clear the queu
      queueRemoteTobiState->clear();
    } else {
      bGotStateFromTobi = false;
    }

    // Get messages from object detection
    if (!queueObjectDetAnswerScope->empty()) {
      INFO_MSG("Received an object from object detection")
      std::string object(*queueObjectDetAnswerScope->pop());
      if (object.compare("null") != 0) {
        objectsDetected.push_back(atoi(object.c_str())-1);
        bGotObject = true;
      } else {
        bGotFalseObject = true;
      }
    } else {
      bGotObject = false;
      bGotFalseObject = false;
    }

    // Get messages from object detection
    if (!queueExplorationAnswerScope->empty()) {
      INFO_MSG("Received a message from exploration")
      std::string msg(*queueExplorationAnswerScope->pop());
      if (msg.compare("finish") == 0) {
        INFO_MSG("Exploration success")
        bGotExplorationFinish = true;
      } else if (msg.compare("false") == 0){
        ERROR_MSG("Exploration failed")
      }
    } else {
      bGotExplorationFinish = false;
    }

    // Get messages from object seperation and delivery
    if (!queueDeliveryAnswerScope->empty()) {
      INFO_MSG("Received an message from object seperation and delivery")
      std::string msg(*queueDeliveryAnswerScope->pop());
      if (msg.find("seperated") != std::string::npos) {
        // We assume the number of objects in the first character
        numSeperatedObjects = atoi(msg.substr(0,1).c_str());
        INFO_MSG("Got " << numSeperatedObjects << " objects from seperation")
        objectDetectionCounter = 0;
        bGotNumberOfSeperatedObjects = true;
      } else if (msg.compare("homingFinish") == 0) {
        bGotHomingFinish = true;
      } else if (msg.compare("deliverFinish") == 0) {
        bGotDeliveryFinish = true;
      } else if (msg.compare("deliverFinish") == 0) {
        bGotDeliveryFail = false;
      }
    } else {
      bGotNumberOfSeperatedObjects = false;
      bGotHomingFinish = false;
      bGotDeliveryFinish = false;
      bGotDeliveryFail = false;
    }

    // Check for keypress in terminal
    KB_code = 0;
    if (kbhit()) {
      KB_code = getchar();
      INFO_MSG("KB_code = " << KB_code)
      // Do an object detection if "d" button was pressed
      if (KB_code == 1000) {
        INFO_MSG("Do object detection")
        amiroState = objectDetection;
      }
    }

    INFO_MSG("STATE: " << statesString[amiroState] )
    switch (amiroState) {
      case idle:
        if(bGotStateFromTobi && sStateFromTobi.compare("init") == 0) {
          amiroState = init;
        }
        break;
      case init:
        // Clear all variables
        objectsDetected.clear();
        bGotStateFromTobi = false;
        bGotObject = false;
        bGotFalseObject = false;
        bGotExplorationFinish = false;
        bGotDeliveryFail = false;
        objectDelivery = -1;
        // Start the exploration
        *tmpDummy = "start";
        informerExplorationScope->publish(tmpDummy);
        amiroState = exploration;
        break;
      case exploration:
        if (bGotExplorationFinish) {
          amiroState = explorationFinish;
        }
        break;
      case explorationFinish:
        amiroState = objectSeperation;
        break;
      case objectSeperation:
        *tmpDummy = "start";
        informerDeliveryScope->publish(tmpDummy);
        amiroState = objectSeperationPending;
        break;
      case objectSeperationPending:
        if (bGotNumberOfSeperatedObjects) {
          amiroState = objectHoming;
        }
        break;
      case objectHoming:
        ++objectDetectionCounter;
        INFO_MSG("objectDetectionCounter: " << objectDetectionCounter)
        *tmpDummy = std::to_string(objectDetectionCounter);
        tmpDummy->append("homing");
        informerDeliveryScope->publish(tmpDummy);
        amiroState = objectHomingPending;
        break;
      case objectHomingPending:
        if (bGotHomingFinish) {
          amiroState = objectDetection;
        }
        break;
      case objectDetection:
        *tmpDummy = "COMP";
        informerObjectDetCmdScope->publish(tmpDummy);
        amiroState = objectDetectionPending;
        break;
      case objectDetectionPending:
        if (bGotObject && objectDetectionCounter < numSeperatedObjects) {
          amiroState = objectHoming;
        } else if (objectDetectionCounter == numSeperatedObjects) {
           INFO_MSG("All objects detected, just idle arround")
           amiroState = idle;
        } else if (bGotFalseObject) {
          // Just go to the next object
          amiroState = objectHoming;
        }
        break;
      case objectDeliveryHoming:
        // TODO Do we need this?
        *tmpDummy = std::to_string(objectDelivery);
        tmpDummy->append("deliver");
        informerDeliveryScope->publish(tmpDummy);
        amiroState = objectDeliveryPushing;
        break;
      case objectDeliveryPushing:
        if (bGotDeliveryFinish) {
          amiroState = objectDeliveryFinish;
        } else if (bGotDeliveryFail) {
          amiroState = objectDeliveryFail;
        }
        break;
      case objectDeliveryFinish:
        informerRemoteObjectFinish[objectDelivery]->publish(statePublish);
        break;
      case objectDeliveryFail:
        amiroState = idle;
        break;
      default:
        WARNING_MSG("DEFAULT")
    }

    *statePublish = statesString[amiroState];
    INFO_MSG("SENDING STATE SCOPE: " << g_sOutScopeStateTobi << "/" << *statePublish << " ; Content: " << *statePublish)

    // Send every detected object in every iteration
    for (int idx = 0; idx < objectsDetected.size(); ++idx) {
      informerRemoteObject[objectsDetected.at(idx)]->publish(statePublish);
    }

    informerRemoteState[amiroState]->publish(statePublish);
    // informerAmiroState->publish(statePublish);
    boost::this_thread::sleep(boost::posix_time::seconds(1));

  }
  return 0;
}
