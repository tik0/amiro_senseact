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
#define NUM_STATES 19
enum states {
	init,
	explorationWait,
	exploration,
	blobDetectionWait,
	blobDetection,
	localPlannerWait,
	localPlanner,
	objectDetectionWait,
	objectDetection,
	initDoneWait,
	objectDeliveryStart,
	objectDeliveryWait,
	objectDelivery,
	objectDeliveryFinish,
	objectTransportStart,
	objectTransportWait,
	objectTransport,
	objectTransportFinish,
	finishAnswerer
};

states amiroState = init;

std::string statesString[NUM_STATES] {
	"init",
	"explorationWait",
	"exploration",
	"blobDetectionWait",
	"blobDetection",
	"localPlannerWait",
	"localPlanner",
	"objectDetectionWait",
	"objectDetection",
	"initDoneWait",
	"objectDeliveryStart",
	"objectDeliveryWait",
	"objectDelivery",
	"objectDeliveryFinish",
	"objectTransportStart",
	"objectTransportWait",
	"objectTransport",
	"objectTransportFinish",
	"finishAnswerer"
};
	



// Objects && Object detection communication
#define NUM_OBJECTS 6
enum objects { object1 , object2 , object3 , object4 , object5 , object6 };
std::string objectsString[NUM_OBJECTS] = {"object1","object2","object3","object4","object5","object6"};

// Exploration
std::string sExplorationCmdScope("/exploration/answer");
std::string sExplorationAnswerScope("/exploration/command");

// Blob detection
std::string sBlobCmdScope("/blobDetection/answer");
std::string sBlobAnswerScope("/blobDetection/command");

// Delivery
std::string sDeliveryCmdScope("/objectDelivery/answer");
std::string sDeliveryAnswerScope("/objectDelivery/command");

// Object transport
std::string sTransportCmdScope("/objectTransport/answer");
std::string sTransportAnswerScope("/objectTransport/command");

// Object detection
std::string sObjectDetCmdScope("/objectDetection/answer");
std::string sObjectDetAnswerScope("/objectDetection/command");

// Local planner
std::string sLocalPlannerCmdScope("/localplanner/answer");
std::string sLocalPlannerAnswerScope("/localplanner/command");

// Communication with ToBI
std::string sOutScopeTobi = "/tobiamiro";
std::string sInScopeTobi = "/amiro";
std::string sInScopeTobi2nd = "tobi";

// Output for debugging
std::string sOutScopeState = "/amiroState";

// Sensor scopes
std::string sInScopeLidar = "/lidar";
std::string sInScopeOdometry = "/odo";


// RSB content
std::string outputRSBOutsideInitDone = "initdone";
std::string outputRSBOutsideDelivery = "delivered";
std::string outputRSBOutsideTransport = "transported";
std::string inputRSBOutsideInit = "init";
std::string inputRSBOutsideDelivery = "deliver";
std::string inputRSBOutsideTransport = "transport";
std::string outputRSBExploration = "start";
std::string inputRSBExploration = "finish";
std::string outputRSBBlobDetection = "start";
std::string inputRSBBlobDetection = "finish";
std::string outputRSBObjectDetection = "start";
std::string inputRSBObjectDetection = "finish";
std::string outputRSBLocalPlanner = "start";
std::string inputRSBLocalPlanner = "finish";
std::string outputRSBDelivery = "start";
std::string inputRSBDelivery = "finish";
std::string outputRSBTransport = "start";
std::string inputRSBTransport = "finish";

// RSB input recognizer
bool rsbInputOutsideInit = false;
bool rsbInputOutsideDeliver = false;
bool rsbInputOutsideTransport = false;
bool rsbInputExploration = false;
bool rsbInputBlobDetection = false;
bool rsbInputObjectDetection = false;
bool rsbInputDelivery = false;
bool rsbInputTransport = false;
bool rsbInputLocalPlanner = false;

int processSM(void);

int robotID = 0;

int objectCount = 2;

std::string sRemoteServerPort = "4823";
std::string sRemoteServer = "localhost";

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
        ("spread,s", po::value < std::string > (&sRemoteServer), "IP of remote spread server.")
        ("spreadPort,p", po::value < std::string > (&sRemoteServerPort), "Port of remote spread server.")
        ("outscopeTobi,o", po::value < std::string > (&sOutScopeTobi), "Scope for sending the current state to tobi.")
        ("outscopeState,s", po::value < std::string > (&sOutScopeState), "Scope for sending the current state internaly.")
        ("inscopeTobi,i", po::value < std::string > (&sInScopeTobi), "Scope for recieving Tobis messages.")
        ("robotID,d", po::value < int > (&robotID), "Robot ID.")
        ("tableDepth", po::value < double > (&tableDepth), "Table depth.")
        ("startPosition", po::value < double > (&tableDepth), "Start Position.")
        ("endPosition", po::value < double > (&endPosition), "End Position.");


    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

    // first, process the help option
    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }

    // afterwards, let program options handle argument errors
    po::notify(vm);

    // prepare scopes for ToBI-AMIRo communication
    sOutScopeTobi.append(std::to_string(robotID));
    sInScopeTobi.append(std::to_string(robotID)).append(sInScopeTobi2nd);

    // print all scopes
    INFO_MSG("List of all RSB scopes:");
    INFO_MSG(" - ToBI to AMiRo:   " << sInScopeTobi);
    INFO_MSG(" - AMiRo to ToBI:   " << sOutScopeTobi);
    INFO_MSG(" - State debugging: " << sOutScopeState);
    INFO_MSG(" - Exploration cmd: " << sExplorationCmdScope);
    INFO_MSG(" - Exploration ans: " << sExplorationAnswerScope);
    INFO_MSG(" - BlobDetect cmd:  " << sBlobCmdScope);
    INFO_MSG(" - BlobDetect ans:  " << sBlobAnswerScope);
    INFO_MSG(" - Detection cmd:   " << sObjectDetCmdScope);
    INFO_MSG(" - Detection ans:   " << sObjectDetAnswerScope);
    INFO_MSG(" - Delivery cmd:    " << sDeliveryCmdScope);
    INFO_MSG(" - Delivery ans:    " << sDeliveryAnswerScope);
    INFO_MSG(" - Transport cmd:   " << sTransportCmdScope);
    INFO_MSG(" - Transport ans:   " << sTransportAnswerScope);

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
                tmpPropSpread["port"] = boost::any(sRemoteServerPort);

                // Change the server
                tmpPropSpread["host"] = boost::any(sRemoteServer);

                // Write the tranport properties back to the participant config
                tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);

                INFO_MSG("Remote Spread Configuration done: " << tmpPropSpread);
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

    rsb::Informer< std::string >::Ptr informerObjectDetScope = factory.createInformer< std::string > (sObjectDetCmdScope);

    // Local Planner: Listener and Informer
    rsb::ListenerPtr listenerLocalPlannerAnswerScope = factory.createListener(sLocalPlannerAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueLocalPlannerAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerLocalPlannerAnswerScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueLocalPlannerAnswerScope)));

    rsb::Informer< std::string >::Ptr informerLocalPlannerScope = factory.createInformer< std::string > (sLocalPlannerCmdScope);

    // Exploration: Listener and Informer
    rsb::ListenerPtr listenerExplorationScope = factory.createListener(sExplorationAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueExplorationAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerExplorationScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueExplorationAnswerScope)));

    rsb::Informer< std::string >::Ptr informerExplorationScope = factory.createInformer< std::string > (sExplorationCmdScope);

    // Blob detection: Listener and Informer
    rsb::ListenerPtr listenerBlobScope = factory.createListener(sBlobAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueBlobAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerBlobScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueBlobAnswerScope)));

    rsb::Informer< std::string >::Ptr informerBlobScope = factory.createInformer< std::string > (sBlobCmdScope);

    // Object seperation and delivery: Listener and Informer
    rsb::ListenerPtr listenerDeliveryScope = factory.createListener(sDeliveryAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueDeliveryAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerDeliveryScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueDeliveryAnswerScope)));

    rsb::Informer< std::string >::Ptr informerDeliveryScope = factory.createInformer< std::string > (sDeliveryCmdScope);

    // Object transport: Listener and Informer
    rsb::ListenerPtr listenerTransportScope = factory.createListener(sTransportAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueTransportAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerTransportScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueTransportAnswerScope)));

    rsb::Informer< std::string >::Ptr informerTransportScope = factory.createInformer< std::string > (sTransportCmdScope);

    /////////////////// REMOTE SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    rsb::ListenerPtr listenerOutsideScope;
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueOutsideScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    rsb::Informer< std::string >::Ptr informerOutsideScope;

    try {
        listenerOutsideScope = factory.createListener(sInScopeTobi);
        listenerOutsideScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueOutsideScope)));

        informerOutsideScope = factory.createInformer< std::string > (sOutScopeTobi);
    }
    catch(std::exception& e) {
        ERROR_MSG("Remote connection not established");
        return -1;
    }

    INFO_MSG("All RSB connections built. Starting statemachine now.");

    boost::shared_ptr<std::string> stringPublisher(new std::string);

    // run through statemachine
    bool runningStatemachine = true;
    while(runningStatemachine) {

        // Check RSB input first
        std::string sRSBInput = "";
        rsbInputOutsideInit = false;
        rsbInputExploration = false;
        rsbInputBlobDetection = false;
        rsbInputObjectDetection = false;
        rsbInputDelivery = false;
        rsbInputTransport = false;
        rsbInputLocalPlanner = false;

        // Check input from outside
        if (!queueOutsideScope->empty()) {
            sRSBInput = *queueOutsideScope->pop();
            if (sRSBInput.compare(outputRSBOutsideInitDone) == 0) {
                rsbInputOutsideInit = true;
            } else if (sRSBInput.compare(outputRSBOutsideDelivery) == 0) {
                rsbInputOutsideDeliver = true;
            } else if (sRSBInput.compare(outputRSBOutsideTransport) == 0) {
                rsbInputOutsideTransport = true;
            }
        }
        if (!queueExplorationAnswerScope->empty()) {
            sRSBInput = *queueExplorationAnswerScope->pop();
            if (sRSBInput.compare(outputRSBExploration) == 0) {
                rsbInputExploration = true;
            }
        }
        if (!queueObjectDetAnswerScope->empty()) {
            sRSBInput = *queueObjectDetAnswerScope->pop();
            if (sRSBInput.compare(outputRSBObjectDetection) == 0) {
                rsbInputObjectDetection = true;
            }
        }
        if (!queueLocalPlannerAnswerScope->empty()) {
            sRSBInput = *queueLocalPlannerAnswerScope->pop();
            if (sRSBInput.compare(outputRSBLocalPlanner) == 0) {
                rsbInputLocalPlanner = true;
            }
        }
        if (!queueBlobAnswerScope->empty()) {
            sRSBInput = *queueBlobAnswerScope->pop();
            if (sRSBInput.compare(outputRSBBlobDetection) == 0) {
                rsbInputBlobDetection = true;
            }
        }
        if (!queueDeliveryAnswerScope->empty()) {
            sRSBInput = *queueDeliveryAnswerScope->pop();
            if (sRSBInput.compare(outputRSBDelivery) == 0) {
                rsbInputDelivery = true;
            }
        }
        if (!queueTransportAnswerScope->empty()) {
            sRSBInput = *queueTransportAnswerScope->pop();
            if (sRSBInput.compare(outputRSBTransport) == 0) {
                rsbInputTransport = true;
            }
        }

        INFO_MSG("STATE: " << statesString[amiroState]);
        switch (amiroState) {
            case init:
                *stringPublisher = inputRSBOutsideInit;
                informerOutsideScope->publish(stringPublisher);
                amiroState = exploration;
                break;
            case explorationWait:
                if (rsbInputExploration) {
                    amiroState = exploration;
                }
                break;
            case exploration:
                INFO_MSG("EXPLORING");
                sleep(3);
                *stringPublisher = inputRSBExploration;
                informerExplorationScope->publish(stringPublisher);
                amiroState = blobDetectionWait;
                break;
            case blobDetectionWait:
                if (rsbInputBlobDetection) {
                    amiroState = blobDetection;
                }
                break;
            case blobDetection:
                INFO_MSG("BLOBBING");
                sleep(3);
                *stringPublisher = inputRSBBlobDetection;
                informerBlobScope->publish(stringPublisher);
                amiroState = localPlannerWait;
                break;
            case localPlannerWait:
                if (objectCount > 0) {
                    if (rsbInputLocalPlanner) {
                        amiroState = localPlanner;
                    }
                } else {
                    amiroState = initDoneWait;
                }
                break;
            case localPlanner:
                INFO_MSG("DRIVING");
                sleep(3);
                *stringPublisher = inputRSBLocalPlanner;
                informerLocalPlannerScope->publish(stringPublisher);
                amiroState = objectDetectionWait;
                break;
            case objectDetectionWait:
                if (rsbInputObjectDetection) {
                    amiroState = objectDetection;
                }
                break;
            case objectDetection:
                INFO_MSG("DETECTING");
                sleep(3);
                objectCount--;
                *stringPublisher = inputRSBObjectDetection;
                informerObjectDetScope->publish(stringPublisher);
                amiroState = localPlannerWait;
                break;
            case initDoneWait:
                if (rsbInputOutsideInit) {
                    amiroState = objectDeliveryStart;
                }
                break;
            case objectDeliveryStart:
                INFO_MSG("WAITING");
                sleep(3);
                *stringPublisher = inputRSBOutsideDelivery;
                informerOutsideScope->publish(stringPublisher);
                amiroState = objectDeliveryWait;
                break;
            case objectDeliveryWait:
                if (rsbInputDelivery) {
                    amiroState = objectDelivery;
                }
                break;
            case objectDelivery:
                INFO_MSG("DELIVERING");
                sleep(3);
                *stringPublisher = inputRSBDelivery;
                informerDeliveryScope->publish(stringPublisher);
                amiroState = objectDeliveryFinish;
                break;
            case objectDeliveryFinish:
                if (rsbInputOutsideDeliver) {
                    amiroState = objectTransportStart;
                }
                break;
            case objectTransportStart:
                INFO_MSG("WAITING");
                sleep(3);
                *stringPublisher = inputRSBOutsideTransport;
                informerOutsideScope->publish(stringPublisher);
                amiroState = objectTransportWait;
                break;
            case objectTransportWait:
                if (rsbInputTransport) {
                    amiroState = objectTransport;
                }
                break;
            case objectTransport:
                INFO_MSG("TRANSPORTING");
                sleep(3);
                *stringPublisher = inputRSBTransport;
                informerTransportScope->publish(stringPublisher);
                amiroState = objectTransportFinish;
                break;
            case objectTransportFinish:
                if (rsbInputOutsideTransport) {
                    amiroState = finishAnswerer;
                }
                break;
            case finishAnswerer:
                INFO_MSG("Statemachine of Answerer finished.");
		return 0;
            default:
                ERROR_MSG("Unknown state in statemachine!");
                return -1;
        }

        usleep(500000);

    }

    INFO_MSG("Statemachine has been closed.");

    return 0;
}
