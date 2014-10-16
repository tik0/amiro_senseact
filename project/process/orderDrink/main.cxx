//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Drives an 8 with the AMiRo and send drink orders to a 
//               foreign spread server
//============================================================================

// Messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

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
// #include <converter/matConverter/matConverter.hpp>
#include <converter/vecIntConverter/main.hpp>
#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>

#include <Constants.h>
#include <Types.h>
#include <ControllerAreaNetwork.h>
#include <Color.h>

using namespace boost;
using namespace std;
using std::vector;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;
using namespace amiro::constants;

// START: Template matching
/// Global Variables

/// Function Headers
// void MatchingMethod(int match_method, cv::Mat &mTemplate, cv::Mat &mImage );
// END: Template Matching


int process(void);

void light(int R, int G, int B, int L, rsb::Informer< std::vector<int> >::Ptr &informerLights) {
  shared_ptr<std::vector<int> > lights( new std::vector<int>(25,200) );
      for (int idx = 0; idx < 8; idx++) {
        lights->at(idx * 3) = R;
        lights->at(idx * 3 + 1) = G;
        lights->at(idx * 3 + 2) = B;
      }
      lights->at(24) = L;
      informerLights->publish(lights);
}

std::string g_sOutScope_Steering = "/motor";
std::string g_sOutScope_Lights = "/lights";
std::string g_sInScope_RingProx = "/prox";
std::string g_sInScope_FloorProx = "/proxFloor";

std::string g_sRemoteRSBServer = "192.168.2.2";
std::string g_sRemoteRSBPort = "4816";
std::string g_sOutScope_Drink1 = "/tabletop/drink/1";
std::string g_sOutScope_Drink2 = "/tabletop/drink/2";
std::string g_sInScope_RemoteStandby = "/tabletop/start";

int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscopeSteering,S", po::value < std::string > (&g_sOutScope_Steering), "Scope for sending steering commands")
    ("outscopeLights,L", po::value < std::string > (&g_sOutScope_Lights), "Scope for sending light commands")
    ("outscopeRemoteDrink1,1", po::value < std::string > (&g_sOutScope_Drink1), "Scope of sending drink 1 message with angular phi")
    ("outscopeRemoteDrink2,2", po::value < std::string > (&g_sOutScope_Drink2), "Scope of sending drink 2 message with angular phi")
    ("remoteServer,s", po::value < std::string > (&g_sRemoteRSBServer), "IP of remote server")
    ("remotePort,p", po::value < std::string > (&g_sRemoteRSBPort), "Port of remote server")
    ("inscopeRingProx,r", po::value < std::string > (&g_sInScope_RingProx), "Scope for receving local ring proximity values")
    ("inscopeFloorProx,f", po::value < std::string > (&g_sInScope_FloorProx), "Scope for receving local floor proximity values")
    ("inscopeRemoteStart,i", po::value < std::string > (&g_sInScope_RemoteStandby), "Scope for receving the remote start signal");

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

  // Process
  return process();
}
ControllerAreaNetwork CAN;
int process(void) {
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // Register new converter for std::vector<int>
    boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
    converterRepository<std::string>()->registerConverter(converterVecInt);

    // Create the factory
    rsb::Factory &factory = rsb::Factory::getInstance();

    //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
    ///////////////////////////////////////////////////////////////////////////////
        // Get the global participant config as a template
        rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
              {
                // Get the options for socket transport, because we want to change them
                rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();

                // enable socket transport
                std::string enabled = "0";
                tmpPropSocket["enabled"] = boost::any(enabled);

                // this node is the server, all other nodes clients server = 0 !!!
                std::string server = "0";
                tmpPropSocket["server"] = boost::any(server);

                // Change the config
                tmpPropSocket["host"] = boost::any(g_sRemoteRSBServer);

                // Write the socket tranport properties back to the participant config
                tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
              }
              {
                // Get the options for spread transport, because we want to change them
                rsc::runtime::Properties tmpPropSpread = tmpPartConf.mutableTransport("spread").getOptions();

                std:: cout << tmpPropSpread << std::endl;

                // enable socket transport
                std::string enabled = "1";
                tmpPropSpread["enabled"] = boost::any(enabled);

                // the port of the server
                std::string port = g_sRemoteRSBPort;
                tmpPropSpread["port"] = boost::any(port);

                // Change the config
                tmpPropSpread["host"] = boost::any(g_sRemoteRSBServer);

                std:: cout << tmpPropSpread << std::endl;

                // Write the socket tranport properties back to the participant config
                tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
              }
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

    /////////////////// LOCAL SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Listener for the proximity ring
    rsb::ListenerPtr listenerProxRing = factory.createListener(g_sInScope_RingProx);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr< std::vector< int > > > > proxRingQueue(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr< std::vector< int > > >(1));
    listenerProxRing->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler< std::vector< int > >(proxRingQueue)));
    
    // Listener for the proximity floor
    rsb::ListenerPtr listenerProxFloor = factory.createListener(g_sInScope_FloorProx);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr< std::vector< int > > > > proxFloorQueue(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr< std::vector< int > > >(1));
    listenerProxFloor->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler< std::vector< int > >(proxFloorQueue)));

    // Informer for the steering commands
    rsb::Informer< std::vector<int> >::Ptr informerSteering = factory.createInformer< std::vector<int> > (g_sOutScope_Steering);
    
    // Informer for the light commands
    rsb::Informer< std::vector<int> >::Ptr informerLights = factory.createInformer< std::vector<int> > (g_sOutScope_Lights);

    /////////////////// REMOTE SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Prepare RSB informer and listener
    rsb::Informer< int64_t >::Ptr informerRemoteDrink1;
    rsb::Informer< int64_t >::Ptr informerRemoteDrink2;

    // Create the listener
    rsb::ListenerPtr listenerRemoteStandby;
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<void> > > queueRemoteStandby(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<void> >(1));

    // Create the informer and listener
    try {
      informerRemoteDrink1 = factory.createInformer< int64_t > (g_sOutScope_Drink1, tmpPartConf);
      informerRemoteDrink2 = factory.createInformer< int64_t > (g_sOutScope_Drink2, tmpPartConf);

      // Create the listener for the standby task
      listenerRemoteStandby = factory.createListener(g_sInScope_RemoteStandby, tmpPartConf);
      listenerRemoteStandby->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<void>(queueRemoteStandby)));

    }
    catch(std::exception& e) {
      light(255,0,0,200,informerLights);
      sleep(1);
      light(255,0,0,0,informerLights);
      ERROR_MSG("Remote connection not established");
      return -1;
    }

   // When we reached this point, everything should be fine
    light(0,255,0,200,informerLights);
    sleep(1);
    light(255,0,0,0,informerLights);
    
   
    // Just a value for sending the angular
    boost::shared_ptr< int64_t > publishPhi(new int64_t(0));

  // Local variables
  std::vector<int> proxRing(8,0);
  std::vector<int> proxFloor(4,0);
  
  bool started = false;
  boost::shared_ptr< std::vector<int> > vecSteering(new std::vector<int> (2, 0));

  const int vNorm = 50000;
  const int wTurn = 400000;
  
  const int maximumBlackValue = 15000;
  int distanceToLine = 0; // Distance in µm
  
  enum STATES {IDLE, START, FIRST_TURN, DRIVE, TURN, DRINK1, DRINK2};
  enum LINE_TOUCHED {LEFT, RIGHT};
  char *StateTypes[] ={"IDLE", "START", "FIRST_TURN", "DRIVE", "TURN", "DRINK1", "DRINK2"};
  STATES state = IDLE;
  LINE_TOUCHED lineTouched = LEFT;
  
  // Reset the odometry
  types::position robotPositionNull;
  robotPositionNull.x = 0;
  robotPositionNull.y = 0;
  robotPositionNull.f_z = 0;
  CAN.setOdometry(robotPositionNull);
  CAN.setOdometry(robotPositionNull);
  CAN.setOdometry(robotPositionNull);
  
  for (;;) {
    // Sleep for a while
      boost::this_thread::sleep( boost::posix_time::milliseconds(50) );
//     DEBUG_MSG( "STATE: " << StateTypes[state] )
    // Process the listener
    if (!queueRemoteStandby->empty() && !started) {
//       INFO_MSG("Received a START signal")
      started = true;
      state = START;
    }
    
    if (!proxFloorQueue->empty()) {
//       DEBUG_MSG("Received proximity floor values")
      proxFloor = *proxFloorQueue->tryPop().get();
//       DEBUG_MSG("val: " << proxFloor[0] << " " << proxFloor[1] << " " << proxFloor[2] << " " << proxFloor[3] )
    }
    
    if (!proxRingQueue->empty()) {
//       DEBUG_MSG("Received proximity ring values")
      proxRing = *proxRingQueue->tryPop().get();
//       DEBUG_MSG("val: " << proxRing[0] << " " << proxRing[1] << " " << proxRing[2] << " " << proxRing[3] )
    }
    
    types::position robotPosition = CAN.getOdometry();
    INFO_MSG("Odom: " << robotPosition.x << " " << robotPosition.y << " " << robotPosition.f_z)
    
    switch (state) {
      case IDLE:
//         state = START;
        break;
      case START: {
        // Drive straight until line
//         proxFloor[0]
        if ( proxFloor[PROX_WR] <= maximumBlackValue || proxFloor[PROX_WL] <= maximumBlackValue) {
          vecSteering->at(0) = 0;
          vecSteering->at(1) = 0;
          informerSteering->publish(vecSteering);
          // Save the distance to the line
          types::position robotPosition = CAN.getOdometry();
          distanceToLine = robotPosition.x;
          
          state = DRIVE;
        } else {
          vecSteering->at(0) = vNorm;
          vecSteering->at(1) = 0;
          informerSteering->publish(vecSteering);
        }
        break;
      }
      case FIRST_TURN: {
        // Do the first 90° turn to the right
        if ( proxFloor[PROX_FR] <= maximumBlackValue || proxFloor[PROX_FL] <= maximumBlackValue) {
          vecSteering->at(0) = 0;
          vecSteering->at(1) = 0;
          informerSteering->publish(vecSteering);
          state = DRIVE;
        } else {
          vecSteering->at(0) = 0;
          vecSteering->at(1) = wTurn;
          informerSteering->publish(vecSteering);
        }
        break;
      }
      case DRIVE: {
        
        // Drive straight until end of line
        int rpmFuzzyCtrl[2];
//         lineFollowing(proxFloor[PROX_FL], proxFloor[PROX_FR], rpmFuzzyCtrl);
        if ( proxFloor[PROX_WR] <= maximumBlackValue && lineTouched == LEFT ) {
          lineTouched = RIGHT;
        } else if ( proxFloor[PROX_WL] <= maximumBlackValue && lineTouched == RIGHT ) {
          lineTouched = LEFT;
        }
        
        
        if ( lineTouched == LEFT ) {
          WARNING_MSG( proxFloor[PROX_WR] << " LEFT" )
          vecSteering->at(0) = vNorm;
          vecSteering->at(1) = -wTurn;
          informerSteering->publish(vecSteering);
        } else if ( lineTouched == RIGHT ) {
          WARNING_MSG( proxFloor[PROX_WR] << " RIGHT" )
          vecSteering->at(0) = vNorm;
          vecSteering->at(1) = wTurn;
          informerSteering->publish(vecSteering);
        }
        
        
        
          if (proxRing[0] > 0x1000u || proxRing[1] > 0x1000u) {
                  state = DRINK1;
                  for (int ledIdx = 0; ledIdx < 8; ++ledIdx) {
                    light(0,255,255,200,informerLights);
                  }
                  // STOP
                  vecSteering->at(0) = 0;
                  vecSteering->at(1) = 0;
                  informerSteering->publish(vecSteering);
                  // Publish the order of drink 2
                  *publishPhi = int64_t(atan2(double(robotPosition.y) * 1e-6, double(robotPosition.x) * 1e-6) * 1e6);
                  DEBUG_MSG(*publishPhi)
                  informerRemoteDrink1->publish(publishPhi);
                  informerRemoteDrink1->publish(publishPhi);
                  informerRemoteDrink1->publish(publishPhi);
                  //timeout
                  INFO_MSG("start sleep");
                  sleep(5);
                  INFO_MSG("end sleep");
                  for (int ledIdx = 0; ledIdx < 8; ++ledIdx) {
                    light(255,255,255,200,informerLights);
                  }
          } else if (proxRing[6] > 0x1000u || proxRing[7] > 0x1000u) {
                  state = DRINK2;
                  for (int ledIdx = 0; ledIdx < 8; ++ledIdx) {
                    light(255,255,0,200,informerLights);
                  }
                  // STOP
                  vecSteering->at(0) = 0;
                  vecSteering->at(1) = 0;
                  informerSteering->publish(vecSteering);
                  // Publish the order of drink 2
                  *publishPhi = int64_t(atan2(double(robotPosition.y) * 1e-6, double(robotPosition.x) * 1e-6) * 1e6);
                  DEBUG_MSG(*publishPhi)
                  informerRemoteDrink2->publish(publishPhi);
                  informerRemoteDrink2->publish(publishPhi);
                  informerRemoteDrink2->publish(publishPhi);
                  //timeout
                  INFO_MSG("start sleep");
                  sleep(5);
                  INFO_MSG("end sleep");
                  for (int ledIdx = 0; ledIdx < 8; ++ledIdx) {
                    light(255,255,255,200,informerLights);
                  }
          } else if (proxRing[2] > 0x1000u && proxRing[3] > 0x1000u
                  && proxRing[4] > 0x1000u && proxRing[5] > 0x1000u) {
                  // ToDo Collision avoidance
                  state = START;
          }
        
        // Reaching end of black line, and going to spin
//         if (proxFloor[PROX_FR] >= maximumBlackValue && proxFloor[PROX_FL] >= maximumBlackValue) {
//           vecSteering->at(0) = 0;
//           vecSteering->at(1) = 0;
//           informerSteering->publish(vecSteering);
//           state = TURN;
//         }
        break;
      }
      case TURN: {
        // Do a 180° turn and drive back
        if ( proxFloor[PROX_FR] <= maximumBlackValue || proxFloor[PROX_FL] <= maximumBlackValue) {
          vecSteering->at(0) = 0;
          vecSteering->at(1) = 0;
          informerSteering->publish(vecSteering);
          state = DRIVE;
        } else {
          vecSteering->at(0) = 0;
          vecSteering->at(1) = wTurn;
          informerSteering->publish(vecSteering);
        }
        break;
      }
      case DRINK1: {
        CAN.setTargetSpeed(0, 0); // STOP
        
        break;
      }
      case DRINK2: {
        CAN.setTargetSpeed(0, 0); // STOP
        break;
      }
    }
  }
  return 0;
}
