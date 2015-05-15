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

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <types/LocatedLaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>



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

void objectDeliveryCtrl(double distance, double angular) {

}


void objectHomingCtrl(double distance, double angular) {

  // get odometry data
  types::position homingPosition;
  homingPosition.x = int(distance * cos(angular) * 1e6);
  homingPosition.y = int(distance * sin(angular)  * 1e6);
  homingPosition.f_z = int(angular  * 1e6);
//  homingPosition = myCAN.getOdometry();

  // calculate angle from robot position (without respect to orientation) to origin (in rad)
  double angleOdo = atan2((double)homingPosition.y, (double)homingPosition.x) + angular;
  if (angleOdo > 2*M_PI) {
    angleOdo -= 2*M_PI;
  }

  // calculate difference angle (in urad) to turn towards origin
  int diffAngle = (int)(angleOdo*1e6)-homingPosition.f_z;

  // calculate angle (in urad) to turn to origin orientation after reaching origin
  int backAngle = -homingPosition.f_z-diffAngle;
  while (backAngle < 0) {
    backAngle += 2*M_PI*1e6;
  }
  while (backAngle > M_PI*1e6) {
    backAngle -= 2*M_PI*1e6;
  }

  // calculate distance between origin and robot position (in um)
  int diffDist = (int) sqrt((double)homingPosition.y*(double)homingPosition.y + (double)homingPosition.x*(double)homingPosition.x);

  // reset position struct
  homingPosition.y = 0;
/*
  // turn towards origin
  homingPosition.x = 0;
  homingPosition.f_z = diffAngle;
  moveToTargetPosition(homingPosition, 1);
  // drive to origin
  homingPosition.x = diffDist;
  homingPosition.f_z = 0;
  moveToTargetPosition(homingPosition, 1);
        // turn towards origin orientation
  homingPosition.x = 0;
  homingPosition.f_z = backAngle;
  moveToTargetPosition(homingPosition, 1);
*/
        int speed;
  // turn towards origin
  if (diffAngle < 0) {
    speed = -1e6;
  } else {
    speed = 1e6;
  }
  myCAN.setTargetSpeed(0,speed);
        usleep(abs(int(0.99f*diffAngle)));
        myCAN.setTargetSpeed(0,0);
  // drive to origin
  if (diffDist < 0) {
    speed = -100e3;
  } else {
    speed = 100e3;
  }
  myCAN.setTargetSpeed(speed,0);
        usleep(abs(diffDist*11));
        myCAN.setTargetSpeed(0,0);
        // turn towards origin orientation
//  if (backAngle < 0) {
//    speed = -1e6;
//  } else {
//    speed = 1e6;
//  }
//  myCAN.setTargetSpeed(0,speed);
//        usleep(abs(int(0.99f*backAngle)));
//        myCAN.setTargetSpeed(0,0);
}


int processSM(void);

//std::string g_sInScope_Stream = "/image";
//std::string g_sInScope_HeadingStop = "/heading/stop";
//std::string g_sOutScope_Steering = "/proc/steering/raw";
//std::string g_sOutScope_RemoteHoming = "/TableTop/BB/homing";
//std::string g_sOutScope_RemoteHomingFinish = "/TableTop/BB/homingFinish";
//std::string g_sOutScope_RemoteFinish = "/TableTop/BB/finish";
//std::string g_sInScope_RemoteStandby = "/TableTop/BB/standby";
//std::string g_sInScope_RemoteStart = "/TableTop/BB/start";
//std::string g_sInScope_RemoteCleanup = "/tabletop/cleanup";

std::string g_sInScopeTobi = "/tobiamiroNUMBER";
std::string g_sOutScopeStateTobi = "/amiroNUMBERtobi";
std::string g_sOutScopeState = "/state";
std::string g_sRemoteServerPort = "4823";
std::string g_sRemoteServer = "localhost";

double tableDepth = 0.7; // cm
double startPosition = 0.10; /*cm start position*/
double endPosition = 0.20;

// Object detection
double minimalAngle = 0;
double minimalValue = 99999;

int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
//    ("outscopeSteering,os", po::value < std::string > (&g_sOutScope_Steering), "Scope for sending steering commands.")
//    ("inscopeHeadingStop,ih", po::value < std::string > (&g_sInScope_HeadingStop), "Scope for receiving a Heading-Stop.")
//    ("inscopeStream,is", po::value < std::string > (&g_sInScope_Stream), "Scope for receiving input images.")
    ("spread,s", po::value < std::string > (&g_sRemoteServer), "IP of remote spread server.")
    ("spreadPort", po::value < std::string > (&g_sRemoteServerPort), "Port of remote spread server.")
    ("outscopeStateTobi", po::value < std::string > (&g_sOutScopeStateTobi), "Scope for sending the current state to tobi.")
    ("outscopeState", po::value < std::string > (&g_sOutScopeState), "Scope for sending the current state internaly.")
    ("inscopeTobi,i", po::value < std::string > (&g_sInScopeTobi), "Scope for recieving Tobis messages.")
    ("tableDepth", po::value < double > (&tableDepth), "Table depth.")
    ("startPosition", po::value < double > (&tableDepth), "Start Position.")
    ("endPosition", po::value < double > (&endPosition), "End Position.");


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
  // Register
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);
  // Prepare RSB listener for incomming lidar scans
    rsb::ListenerPtr lidarListener = factory.createListener("/lidar"/*lidarInScope*/);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
    lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
    // Prepare RSB async listener for odometry messages
    rsb::ListenerPtr listener = factory.createListener("/odom"/*odomInScope*/);
//    listener->addHandler(HandlerPtr(new DataFunctionHandler<rst::geometry::Pose> (&storeOdomData)));





    // Drive back
//    objectHomingCtrl(-0.1,  minimalAngle);

//    return 0;


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

      if (sStateFromTobi.compare("object3") == 0) {
        amiroState = objectDeliveryPushing;
      }
      // Check if the content was an object request
      for (int idx = 0; idx < NUM_OBJECTS; ++idx) {
        if ( sStateFromTobi.compare(objectsString[idx]) == 0) {  // if ( sStateFromTobi.compare(objectsString[idx]) != 0) {
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

        // FAKE Exploration
        // Get angular and ditance of closest object
        for (int lidarIdx = 0; lidarIdx < 10; lidarIdx++) {
        // while(true) {
          minimalValue = 99999;
          minimalAngle = 0;
          boost::shared_ptr< rst::vision::LocatedLaserScan > data = lidarQueue->pop();
          int minimalIdx = 0;
          for (int idx = 0; idx < data->scan_values_size()-1; idx++) {
              double a = data->scan_values(idx);

              if (minimalValue > a && a > data->scan_values_min()) {
                WARNING_MSG(" " << a)
                minimalValue = a;
                minimalIdx = idx;
              }

            }
          minimalAngle = - double(data->scan_angle_start()) - double(data->scan_angle_end() - data->scan_angle_start()) / double((data->scan_values_size() - 1)) * double(minimalIdx);

          ERROR_MSG("start " << data->scan_angle_start() << " ; data->scan_angle_end() " << data->scan_angle_end() << " ; idx " << minimalIdx << " ; size " << data->scan_values_size() - 1)
          ERROR_MSG("v " << minimalValue << " ; a " << minimalAngle)

        }

        minimalValue = minimalValue - 0.16;  // Stop 16 cm before the object
        types::position homingPosition;
        homingPosition.x = int(minimalValue * cos(minimalAngle) * 1e6);
        homingPosition.y = int(minimalValue * sin(minimalAngle)  * 1e6);
        homingPosition.f_z = int(minimalAngle  * 1e6);

        ERROR_MSG("x " << homingPosition.x << " ; y " << homingPosition.y << " ; fz " << homingPosition.f_z )

        // myCAN.setTargetPosition(homingPosition, 5000/*ms*/);
        objectHomingCtrl(minimalValue, minimalAngle);

        // Fake fnish




        bGotExplorationFinish = true;
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
        // HACK
        bGotNumberOfSeperatedObjects = true;
        numSeperatedObjects = 1;
        objectDetectionCounter = 1;
//        objectHomingCtrl(/*distance*/, /*angle*/);
        amiroState = objectDetection;
//        if (bGotNumberOfSeperatedObjects) {
//          amiroState = objectHoming;
//        }
        // HACK FINISH
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
        // HACK
//        objectDeliveryCtrl(/*dist*/, /*angle*/);
        // Pushing
        // myCAN.setTargetPosition(homingPosition, 5000/*ms*/);
      {
        double distanceToDeliver = (tableDepth - startPosition - endPosition ) / cos(minimalAngle) - minimalValue;
        objectHomingCtrl(distanceToDeliver, 0);
      }
        amiroState = objectDeliveryFinish;
        break;
        // HACKFinish
        if (bGotDeliveryFinish) {
          amiroState = objectDeliveryFinish;
        } else if (bGotDeliveryFail) {
          amiroState = objectDeliveryFail;
        }
        break;
      case objectDeliveryFinish:
        // FAKE Drive back
        myCAN.setTargetSpeed(- 200 * 1e3, 0);
        sleep(1);
        myCAN.setTargetSpeed(0, 0);
        myCAN.setTargetSpeed(0, 0);
        // FAKE FINISH
        // HACK so that nils listenes
        while (true) {
//          informerRemoteObjectFinish[objectDelivery]->publish(statePublish);
          informerRemoteObjectFinish[2]->publish(statePublish);
          boost::this_thread::sleep(boost::posix_time::seconds(1));
        }
        // HACK FINISH
        amiroState = idle;
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