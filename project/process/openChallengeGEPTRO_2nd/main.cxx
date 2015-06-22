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
#include <rst/geometry/Pose.pb.h>



using namespace boost;
using namespace std;
using std::vector;
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// State machine states
#define NUM_STATES 13
enum states {
	idle,
	init,
	explorationStart,
	exploration,
	blobDetectionStart,
	blobDetection,
	objectDetectionMain,
	initDone,
	waiting,
	objectDeliveryStart,
	objectDelivery,
	objectTransportStart,
	objectTransport
};

states amiroState = idle;
states amiroStateL = init;

std::string statesString[NUM_STATES] {
	"idle",
	"initialization",
	"starting exploration",
	"exploration",
	"starting blob detection",
	"blob detection",
	"object detection",
	"initialization done",
	"waiting",
	"starting object delivery",
	"object delivery",
	"starting object transport",
	"object transport"
};

// object detection states
#define NUM_STATES_OD 3
enum states_od {
	localPlannerStart,
	localPlanner,
	objectDetection
};

states_od objectDetectionState = localPlannerStart;
states_od objectDetectionStateL = localPlanner;

std::string statesODString[NUM_STATES] {
	"starting local planner",
	"local planner",
	"detecting"
};
	



// Objects && Object detection communication
#define NUM_OBJECTS 6
enum objects { object1 , object2 , object3 , object4 , object5 , object6 };
std::string objectsString[NUM_OBJECTS] = {"object1","object2","object3","object4","object5","object6"};

// RSB informer
rsb::Informer<std::string>::Ptr informerObjectDetScope;
rsb::Informer<std::string>::Ptr informerLocalPlannerScope;
rsb::Informer<std::string>::Ptr informerExplorationScope;
rsb::Informer<std::string>::Ptr informerBlobScope;
rsb::Informer<std::string>::Ptr informerDeliveryScope;
rsb::Informer<std::string>::Ptr informerTransportScope;

// Exploration
std::string sExplorationCmdScope("/exploration/command");
std::string sExplorationAnswerScope("/exploration/answer");

// Blob detection
std::string sBlobCmdScope("/blobDetection/command");
std::string sBlobAnswerScope("/blobDetection/answer");

// Delivery
std::string sDeliveryCmdScope("/objectDelivery/command");
std::string sDeliveryAnswerScope("/objectDelivery/answer");

// Object transport
std::string sTransportCmdScope("/objectTransport/command");
std::string sTransportAnswerScope("/objectTransport/answer");

// Object detection
std::string sObjectDetCmdScope("/objectDetection/command");
std::string sObjectDetAnswerScope("/objectDetection/detected");

// Local planner
std::string sLocalPlannerCmdScope("/localplanner/command");
std::string sLocalPlannerAnswerScope("/localplanner/answer");

// Communication with ToBI
std::string sInScopeTobi = "/tobiamiro";
std::string sOutScopeTobi = "/amiro";
std::string sOutScopeTobi2nd = "tobi";

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
std::string outputRSBObjectDetection = "COMP";
std::string inputRSBObjectDetection = "finish";
std::string outputRSBLocalPlanner = "start";
std::string inputRSBLocalPlanner = "finish";
std::string outputRSBDelivery = "start";
std::string inputRSBDelivery = "finish";
std::string outputRSBTransport = "start";
std::string inputRSBTransport = "finish";

// string publisher
boost::shared_ptr<std::string> stringPublisher(new std::string);

// RSB input recognizer
bool rsbInputOutsideInit = false;
bool rsbInputOutsideDeliver = false;
bool rsbInputOutsideTransport = false;
bool rsbInputExploration = false;
bool rsbInputBlobDetection = false;
bool rsbInputDelivery = false;
bool rsbInputTransport = false;
bool rsbInputObjectDetection = false;
bool rsbInputLocalPlanner = false;

int objectCount = 0;
std::string objectDetectionAnswer = "";

int robotID = 0;
std::string colorInit = "";
bool blinkerRight = false;

std::string sRemoteServerPort = "4823";
std::string sRemoteServer = "localhost";

bool testWithAnswerer = false;

// functions
bool readInitInput(std::string inputData);
void setLightcolor(void);
void idleBlink(void);
int processSM(void);
int ssmObjectDetection(void);

int main(int argc, char **argv) {
    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
//    ("outscopeSteering,os", po::value < std::string > (&g_sOutScope_Steering), "Scope for sending steering commands.")
//    ("inscopeHeadingStop,ih", po::value < std::string > (&g_sInScope_HeadingStop), "Scope for receiving a Heading-Stop.")
//    ("inscopeStream,is", po::value < std::string > (&g_sInScope_Stream), "Scope for receiving input images.")
        ("spread,s", po::value < std::string > (&sRemoteServer), "IP of remote spread server.")
        ("spreadPort,p", po::value < std::string > (&sRemoteServerPort), "Port of remote spread server.")
        ("outscopeTobi,o", po::value < std::string > (&sOutScopeTobi), "Scope for sending the current state to tobi.")
        ("outscopeState,s", po::value < std::string > (&sOutScopeState), "Scope for sending the current state internaly.")
        ("inscopeTobi,i", po::value < std::string > (&sInScopeTobi), "Scope for recieving Tobis messages.")
        ("robotID,d", po::value < int > (&robotID), "Robot ID.")
        ("testWithAnswerer", "Prepares some constants for test with answerer.");


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

    testWithAnswerer = vm.count("testWithAnswerer");

    // prepare scopes for ToBI-AMIRo communication
    sInScopeTobi.append(std::to_string(robotID));
    sOutScopeTobi.append(std::to_string(robotID)).append(sOutScopeTobi2nd);

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

    informerObjectDetScope = factory.createInformer< std::string > (sObjectDetCmdScope);

    // Local Planner: Listener and Informer
    rsb::ListenerPtr listenerLocalPlannerAnswerScope = factory.createListener(sLocalPlannerAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueLocalPlannerAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerLocalPlannerAnswerScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueLocalPlannerAnswerScope)));

    informerLocalPlannerScope = factory.createInformer< std::string > (sLocalPlannerCmdScope);

    // Exploration: Listener and Informer
    rsb::ListenerPtr listenerExplorationScope = factory.createListener(sExplorationAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueExplorationAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerExplorationScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueExplorationAnswerScope)));

    informerExplorationScope = factory.createInformer< std::string > (sExplorationCmdScope);

    // Blob detection: Listener and Informer
    rsb::ListenerPtr listenerBlobScope = factory.createListener(sBlobAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueBlobAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerBlobScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueBlobAnswerScope)));

    informerBlobScope = factory.createInformer< std::string > (sBlobCmdScope);

    // Object seperation and delivery: Listener and Informer
    rsb::ListenerPtr listenerDeliveryScope = factory.createListener(sDeliveryAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueDeliveryAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerDeliveryScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueDeliveryAnswerScope)));

    informerDeliveryScope = factory.createInformer< std::string > (sDeliveryCmdScope);

    // Object transport: Listener and Informer
    rsb::ListenerPtr listenerTransportScope = factory.createListener(sTransportAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueTransportAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerTransportScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueTransportAnswerScope)));

    informerTransportScope = factory.createInformer< std::string > (sTransportCmdScope);

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

    // run through statemachine
    bool runningStatemachine = true;
    while(runningStatemachine) {

        // Check RSB input first
        std::string sRSBInput = "";
        rsbInputOutsideInit = false;
        rsbInputOutsideDeliver = false;
        rsbInputOutsideTransport = false;
        rsbInputExploration = false;
        rsbInputBlobDetection = false;
        rsbInputObjectDetection = false;
        rsbInputDelivery = false;
        rsbInputTransport = false;
        rsbInputLocalPlanner = false;

        // Check input from outside
        if (!queueOutsideScope->empty()) {
            sRSBInput = *queueOutsideScope->pop();
            if (readInitInput(sRSBInput)) {
                rsbInputOutsideInit = true;
            } else if (sRSBInput.compare(inputRSBOutsideDelivery) == 0) {
                rsbInputOutsideDeliver = true;
            } else if (sRSBInput.compare(inputRSBOutsideTransport) == 0) {
                rsbInputOutsideTransport = true;
            }   
        }
        if (!queueExplorationAnswerScope->empty()) {
            sRSBInput = *queueExplorationAnswerScope->pop();
            if (sRSBInput.compare(inputRSBExploration) == 0) {
                rsbInputExploration = true;
            }
        }
        if (!queueObjectDetAnswerScope->empty()) {
            objectDetectionAnswer = *queueObjectDetAnswerScope->pop();
            rsbInputObjectDetection = true;
//            if (sRSBInput.compare(inputRSBObjectDetection) == 0) {
//                rsbInputObjectDetection = true;
//            }
        }
        if (!queueLocalPlannerAnswerScope->empty()) {
            sRSBInput = *queueLocalPlannerAnswerScope->pop();
            if (sRSBInput.compare(inputRSBLocalPlanner) == 0) {
                rsbInputLocalPlanner = true;
            }
        }
        if (!queueBlobAnswerScope->empty()) {
            sRSBInput = *queueBlobAnswerScope->pop();
            if (sRSBInput.compare(inputRSBBlobDetection) == 0) {
                rsbInputBlobDetection = true;
            }
        }
        if (!queueDeliveryAnswerScope->empty()) {
            sRSBInput = *queueDeliveryAnswerScope->pop();
            if (sRSBInput.compare(inputRSBDelivery) == 0) {
                rsbInputDelivery = true;
            }
        }
        if (!queueTransportAnswerScope->empty()) {
            sRSBInput = *queueTransportAnswerScope->pop();
            if (sRSBInput.compare(inputRSBTransport) == 0) {
                rsbInputTransport = true;
            }
        }

        // print actual state
        if (amiroState == objectDetectionMain && objectDetectionState != objectDetectionStateL) {
            INFO_MSG("STATE: " << statesString[amiroState] << " -> " << statesODString[objectDetectionState]);
            objectDetectionStateL = objectDetectionState;
        } else if (amiroState != amiroStateL) {
            INFO_MSG("STATE: " << statesString[amiroState]);
        }
        amiroStateL = amiroState;

        // check states
        switch (amiroState) {
            case idle:
                idleBlink();
                if (rsbInputOutsideInit) {
                    amiroState = init;
                }
                break;
            case init:
                // TODO initalization parts
                setLightcolor();
                *stringPublisher = inputRSBOutsideInit;
                informerOutsideScope->publish(stringPublisher);
                amiroState = explorationStart;
                break;
            case explorationStart:
                *stringPublisher = outputRSBExploration;
                informerExplorationScope->publish(stringPublisher);
                amiroState = exploration;
                break;
            case exploration:
                if (rsbInputExploration) {
                    //informerOutsideScope->publish(???);
                    amiroState = blobDetectionStart;
                }
                break;
            case blobDetectionStart:
                *stringPublisher = outputRSBBlobDetection;
                informerBlobScope->publish(stringPublisher);
                amiroState = blobDetection;
                break;
            case blobDetection:
                if (rsbInputBlobDetection) {
                    //informerOutsideScope->publish(???);
                    amiroState = objectDetectionMain;
                    if (testWithAnswerer) {
                        objectCount = 1;
                    }
                }
                break;
            case objectDetectionMain:
                if (objectCount > 0) {
                    ssmObjectDetection();
                } else {
                    //informerOutsideScope->publish(???);
                    amiroState = initDone;
                }
                break;
            case initDone:
                *stringPublisher = outputRSBOutsideInitDone;
                informerOutsideScope->publish(stringPublisher);
                amiroState = waiting;
                break;
            case waiting:
                if (rsbInputOutsideDeliver) {
                    amiroState = objectDeliveryStart;
                } else if (rsbInputOutsideTransport) {
                    amiroState = objectTransportStart;
                }
                break;
            case objectDeliveryStart:
                *stringPublisher = outputRSBDelivery;
                informerDeliveryScope->publish(stringPublisher);
                amiroState = objectDelivery;
                break;
            case objectDelivery:
                if (rsbInputDelivery) {
                    *stringPublisher = outputRSBOutsideDelivery;
                    informerOutsideScope->publish(stringPublisher);
                    amiroState = waiting;
                }
                break;
            case objectTransportStart:
                *stringPublisher = outputRSBTransport;
                informerTransportScope->publish(stringPublisher);
                amiroState = objectTransport;
                break;
            case objectTransport:
                if (rsbInputTransport) {
                    *stringPublisher = outputRSBOutsideTransport;
                    informerOutsideScope->publish(stringPublisher);
                    amiroState = waiting;
                }
                break;
            default:
                ERROR_MSG("Unknown state in statemachine!");
                return -1;
        }

        usleep(500000);

    }

    INFO_MSG("Statemachine has been closed.");

    return 0;
}

bool readInitInput(std::string inputData) {
//    INFO_MSG("Input is '" << inputData << "' and expected is '" << inputRSBOutsideInit << "' and colors");
    int expectedLength = inputRSBOutsideInit.size();
    if (inputData.size() < expectedLength) {
//        WARNING_MSG("Input size is " << inputData.size() << ", but expected " << expectedLength);
        return false;
    }
    std::string justInit;
    justInit.append(inputData, 0, expectedLength);
//    INFO_MSG("First " << expectedLength << " chars: " << justInit);
    if (justInit.compare(inputRSBOutsideInit) == 0) {
        colorInit = "";
    	colorInit.append(inputData, expectedLength, inputData.size()-expectedLength);
//        INFO_MSG("Recognized colors: " << colorInit);
        return true;
    } else {
//        WARNING_MSG("'" << inputRSBOutsideInit << "' couldn't be found in '" << justInit << "'");
        return false;
    }
}

void setLightcolor(void) {
    amiro::Color color;
    if (robotID < colorInit.size()) {
        switch (colorInit[robotID]) {
            case 'r':
//                INFO_MSG("Chosen color is red.");
                color = amiro::Color::RED;
                break;
            case 'b':
//                INFO_MSG("Chosen color is blue.");
                color = amiro::Color::BLUE;
                break;
            case 'g':
//                INFO_MSG("Chosen color is green.");
                color = amiro::Color::GREEN;
                break;
            case 'y':
//                INFO_MSG("Chosen color is yellow.");
                color = amiro::Color::YELLOW;
                break;
            default:
                WARNING_MSG("Color '" << colorInit[robotID] << "' is unknown!");
                color = amiro::Color::WHITE;
        }
    } else if (colorInit.size() > 0) {
        WARNING_MSG("For id " << robotID << " isn't any color defined, only for ids until " << colorInit.size()-1);
        color = amiro::Color::WHITE;
    } else {
        WARNING_MSG("There aren't any colors defined.");
        color = amiro::Color::WHITE;
    }
    for (int ledIdx=0; ledIdx<8; ledIdx++) {
        myCAN.setLightColor(ledIdx, color);
    }
}

void idleBlink(void) {
    blinkerRight = !blinkerRight;
    for (int ledIdx=0; ledIdx<8; ledIdx++) {
        if (blinkerRight && ledIdx < 4 || !blinkerRight && ledIdx >= 4) {
            myCAN.setLightColor(ledIdx, amiro::Color(amiro::Color::WHITE));
        } else {
            myCAN.setLightColor(ledIdx, amiro::Color(amiro::Color::BLACK));
        }
    }
}

int ssmObjectDetection(void) {
    switch (objectDetectionState) {
        case localPlannerStart:
            *stringPublisher = outputRSBLocalPlanner;
            informerLocalPlannerScope->publish(stringPublisher);
            objectDetectionState = localPlanner;
            break;
        case localPlanner:
            if (rsbInputLocalPlanner) {
                *stringPublisher = outputRSBObjectDetection;
                informerObjectDetScope->publish(stringPublisher);
                objectDetectionState = objectDetection;
            }
            break;
        case objectDetection:
            if (rsbInputObjectDetection) {
                // TODO check if object has been detected and reduce #object afterwards
                if (objectDetectionAnswer.compare("null") == 0) {
                    WARNING_MSG(" -> Doesn't know the object.");
                } else {
                    INFO_MSG(" -> Object " << objectDetectionAnswer << " found!");
                    objectCount--;
                }
                if (testWithAnswerer) {
                    objectCount--;
                }
                objectDetectionState = localPlannerStart;
            }
            break;
        default:
            ERROR_MSG("Unknown state in statemachine!");
            return -1;
    }
    if (objectDetectionState == localPlanner) {
        return 1;
    } else {
        return 0;
    }
}

