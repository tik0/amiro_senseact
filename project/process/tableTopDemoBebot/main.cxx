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

// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace boost;
using namespace std;
using std::vector;
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// If the program compiles for the ARM architecture
// then throw away all image output
#ifdef __arm__
  #define NO_SHOW_
#endif

// The tableTopDetection by Leon Ziegler
#include "tableTopDetection.hpp"

// For template matching on the horizont
#include "templateMatching.hpp"

// START: Template matching
/// Global Variables

/// Function Headers
// void MatchingMethod(int match_method, cv::Mat &mTemplate, cv::Mat &mImage );
// END: Template Matching


// The location of the object and its variance
Point2f objectMean(0.f, 0.f);
Point2f objectVar(0.f, 0.f);

// PARAMETERS for processStrem()
// the color value difference from the seed point
const int regionGrowingThreshold = 50;
// the color value difference for considering a pixel as foreground
const int differenceImageThreshold = 40;
// the minimum size of a found object relative to the image width
const float minWidthThreshold = 0.05;
// number of frames an object must remain at the same location
const int stableFrames = 15;
int processStream(void);

std::string g_sInScope_Stream = "/image";
std::string g_sInScope_HeadingStop = "/heading/stop";
std::string g_sOutScope_Steering = "/proc/steering/raw";
std::string g_sRemoteSocketServer = "192.168.2.2";
std::string g_sOutScope_RemoteHoming = "/TableTop/BB/homing";
std::string g_sOutScope_RemoteHomingFinish = "/TableTop/BB/homingFinish";
std::string g_sOutScope_RemoteFinish = "/TableTop/BB/finish";
std::string g_sInScope_RemoteStandby = "/TableTop/BB/standby";
std::string g_sInScope_RemoteStart = "/TableTop/BB/start";
std::string g_sInScope_RemoteCleanup = "/tabletop/cleanup";


int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscopeSteering,os", po::value < std::string > (&g_sOutScope_Steering), "Scope for sending steering commands.")
    ("inscopeHeadingStop,ih", po::value < std::string > (&g_sInScope_HeadingStop), "Scope for receiving a Heading-Stop.")
    ("inscopeStream,is", po::value < std::string > (&g_sInScope_Stream), "Scope for receiving input images.")
    ("remoteSocketServer,s", po::value < std::string > (&g_sRemoteSocketServer), "IP of remote socket server.")
    ("outscopeRemoteHoming,orh", po::value < std::string > (&g_sOutScope_RemoteHoming), "Scope for sending homing-state to the remote server.")
    ("outscopeRemoteHomingFinish,orhf", po::value < std::string > (&g_sOutScope_RemoteHomingFinish), "Scope for sending homing-finish-state to the remote server.")
    ("outscopeRemoteFinish,orf", po::value < std::string > (&g_sOutScope_RemoteFinish), "Scope for sending finish-state to the remote server.")
    ("inscopeRemoteStandby,irs", po::value < std::string > (&g_sInScope_RemoteStandby), "Scope for recieving standby-state to the remote server.")
    ("inscopeRemoteCleanup,irs", po::value < std::string > (&g_sInScope_RemoteCleanup), "Scope for recieving the cleanup signal.")
    ("inscopeRemoteStart,irst", po::value < std::string > (&g_sInScope_RemoteStart), "Scope for recieving the start signal.");

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

  #ifndef NO_SHOW_
    // Preview Windows
    namedWindow("reference mask", WINDOW_AUTOSIZE);
    namedWindow("result", WINDOW_AUTOSIZE);
    namedWindow("cam", WINDOW_AUTOSIZE);
  #endif
  #ifdef __arm__
        ::system("/home/root/leds_trigger_none.sh");
        ::system("/home/root/leds_brightness.sh 0");
  #endif

  // use camera stream for testing
  return processStream();
}

int processStream(void) {
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // Register new converter for cv::Mat
    boost::shared_ptr<MatConverter> converter(new MatConverter());
    converterRepository<std::string>()->registerConverter(converter);
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
                // ToDo Change the host for the real server
  //         std::string host = "192.168.0.200";
                tmpPropSocket["host"] = boost::any(g_sRemoteSocketServer);

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
                std::string port = "4816";
                tmpPropSpread["port"] = boost::any(port);

                // Change the config
                // ToDo Change the host for the real server
  //            std::string host = "192.168.0.200";
                tmpPropSpread["host"] = boost::any(g_sRemoteSocketServer);

                std:: cout << tmpPropSpread << std::endl;

                // Write the socket tranport properties back to the participant config
                tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
              }
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

    /////////////////// LOCAL SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Listener for the images
    rsb::ListenerPtr listenerImages = factory.createListener(g_sInScope_Stream);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<cv::Mat> > > imageQueue(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<cv::Mat> >(1));
    listenerImages->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<cv::Mat>(imageQueue)));

    // Informer for the steering commands
    rsb::Informer< std::vector<int> >::Ptr informerSteering = factory.createInformer< std::vector<int> > (g_sOutScope_Steering);

    // Listener for the the stopping of the heading
    rsb::ListenerPtr listenerStopHeading = factory.createListener(g_sInScope_HeadingStop);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<void> > > queueStopHeading(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<void> >(1));
    listenerStopHeading->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<void>(queueStopHeading)));

    /////////////////// REMOTE SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Prepare RSB informer and listener
    rsb::Informer< void >::Ptr informerRemoteHoming;
    rsb::Informer< void >::Ptr informerRemoteHomingFinish;
    rsb::Informer< void >::Ptr informerRemoteFinish;
    // Create the listener
    rsb::ListenerPtr listenerRemoteStandby;
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<void> > > queueRemoteStandby(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<void> >(1));

    rsb::ListenerPtr listenerRemoteStart;
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<void> > > queueRemoteStart(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<void> >(1));

    rsb::ListenerPtr listenerRemoteCleanup;
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<void> > > queueRemoteCleanup(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<void> >(1));

    try {
      informerRemoteHoming = factory.createInformer< void > (g_sOutScope_RemoteHoming, tmpPartConf);
      informerRemoteHomingFinish = factory.createInformer< void > (g_sOutScope_RemoteHomingFinish, tmpPartConf);
      informerRemoteFinish = factory.createInformer< void > (g_sOutScope_RemoteFinish, tmpPartConf);

      // Create the listener for the standby task
      listenerRemoteStandby = factory.createListener(g_sInScope_RemoteStandby, tmpPartConf);
      listenerRemoteStandby->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<void>(queueRemoteStandby)));

      // Create the listener for the start signal
      listenerRemoteStart = factory.createListener(g_sInScope_RemoteStart, tmpPartConf);
      listenerRemoteStart->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<void>(queueRemoteStart)));

      // Create the listener for the cleanup signal
      listenerRemoteCleanup = factory.createListener(g_sInScope_RemoteCleanup, tmpPartConf);
      listenerRemoteCleanup->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<void>(queueRemoteCleanup)));
    }
    catch(std::exception& e) {
      ERROR_MSG("Remote connection not established");
#ifdef __arm__
      ::system("/home/root/leds_trigger_none.sh");
      ::system("/home/root/leds_brightness.sh 0");
      ::system("/home/root/leds_red_brightness.sh 512");
#endif
      return -1;
    }

    // When we reached this point, everything should be fine
#ifdef __arm__
    ::system("/home/root/leds_trigger_none.sh");
    ::system("/home/root/leds_brightness.sh 0");
    ::system("/home/root/leds_green_brightness.sh 512");
    boost::this_thread::sleep(boost::posix_time::seconds(2));
    ::system("/home/root/leds_brightness.sh 0");
#endif

    // Variables for sending
    // Allocate the new steering vector (two elements with 0 initialized)
    boost::shared_ptr< std::vector<int> > vecSteering(new std::vector<int> (2, 0));

    // Just a void for sending nothing
    boost::shared_ptr< void > publishVoid(new int(0));

  // Local variables

  // get reference image
  cv::Mat reference;
  cv::Mat referenceMask;
  // The input image which is updated every time an image is recieved
  cv::Mat mInput;
  cv::Mat mLastInput;
  // The object for homing which is detected by processImage()
  cv::Mat mObject;
  // The rectangular for the object
  cv::Rect rectObject;

  // State transitions and flags
  bool bInit = false;  // Is true, if there was a start signal
  bool bMotionDetected = false;  // Is true, if motion is in the frame
  bool bMotionGone = false;  // Is true, if "bMotionDetected" is true, and when there is no more motion in the frame
  bool bGotReferenceMask = false;  // Is true, if the reference mask was created
  bool bStart = false;  // Is true, if Tobi send a start signal
  bool bTobiStandBy = false;  // Is trie, if Tobi sends a "standby" message
  bool bHeadingStop = false;  // Is true, if the robot recieved a message from the frontal obstacle detection
  bool bFinish = false;  // Is true if the overall task is finish
  bool bIsHeading = false;  // Is true, if the robot is heading an object
  bool bImageRecieved = false;  // Binary identifier if a new image lies in mInput
  bool bCleanup = false;  // True if cleanup signal recieved
  bool bGotCleanup = false;  // True if the step for the cleanup signal was processed (blinking blue)

  // Variables for the keypress
  // Check for keypress in winow
  int KB_codeCV = 0;
  // Check for keypress in terminal
  int KB_code = 0;


  // Process the behaviour
  while (true) {
    // Process the listener
    if (!imageQueue->empty()) {
      mLastInput = mInput.clone();
      mInput = *imageQueue->tryPop().get();
      bImageRecieved = true;
#ifndef NO_SHOW_
      imshow("Recieved Image", mInput);
#endif
    } else {
      bImageRecieved = false;
    }

    if (!queueStopHeading->empty()) {
      INFO_MSG("Received a HEADING STOP from sensor fusion")
      bHeadingStop = true;
      // Clear the queu
      queueStopHeading->clear();
      INFO_MSG("Heading stop")
    } else {
      bHeadingStop = false;
    }

    if (!queueRemoteStandby->empty()) {
      INFO_MSG("Received a STANDYBY from Tobi")
      bTobiStandBy = true;
      // Clear the queu
      queueRemoteStandby->clear();
      INFO_MSG("Standby from Tobi")
    }

    if (!queueRemoteStart->empty()) {
      bStart = true;
      // Clear the queu
      queueRemoteStart->clear();
      INFO_MSG("Start from Tobi")
    } else {
      bStart = false;
    }

    if (!queueRemoteCleanup->empty() ) {
      // Clear the queu
      queueRemoteCleanup->clear();
      // Only do the cleanup, if it is in the start state
      if (bGotReferenceMask) {
        bCleanup = true;
        INFO_MSG("Cleanup signal recieved")
        DEBUG_MSG(bGotReferenceMask << bCleanup <<  !bGotCleanup)
      }
      
    } else {
//      bCleanup = false;
    }

    // Check for keypress in winow
    KB_codeCV = waitKey(1);

    // Check for keypress in terminal
    KB_code = 0;
    if (kbhit()) {
      KB_code = getchar();
      INFO_MSG("KB_code = " << KB_code)
    }

    // Process the keys/states
//     if(KB_codeCV == KB_1 || KB_code == KB_1 || g_State == STATE_CREATE_REFERENCE_MASK) {
      if (KB_codeCV == KB_0 || KB_code == KB_0 || (bStart && !bInit)) {
      INFO_MSG("Start")
      bInit = true;
#ifdef __arm__
        ::system("/home/root/leds_green_brightness.sh 512");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ::system("/home/root/leds_green_brightness.sh 0");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
#endif
    } else if (KB_codeCV == KB_1 || KB_code == KB_1 || (bInit && bImageRecieved && !bGotReferenceMask)) {
      INFO_MSG("Create new reference mask")
//       bInit = false;
      mInput.copyTo(reference);
      referenceMask = tableTopDetection::createReferenceMask(reference, regionGrowingThreshold);
      bGotReferenceMask = true;
#ifndef NO_SHOW_
      imshow("reference mask", referenceMask);
#endif
//     } else if(KB_codeCV == KB_2 || KB_code == KB_2 || (bGotReferenceMask && bImageRecieved && !bMotionGone)) {
//       INFO_MSG("Process the difference image to get a motion trigger")
//       cv::Mat mDiff(mInput.rows, mInput.cols, mInput.type());
//       // calculate difference image
//       cv::absdiff(mInput, mLastInput, mDiff);
//       // Get the grayscale image
//       cvtColor(mDiff, mDiff, CV_RGB2GRAY);
//       // Apply the mask
//       cv::bitwise_and(mDiff, referenceMask, mDiff);
//       // Get the difference
//       cv::Scalar vecSumDiff = cv::sum(mDiff);
//       // Get the size of the mask
//       cv::Scalar vecSumRef = cv::sum(referenceMask);
//       // Calculate the normalized difference (only channel 1, because it is a grey scale image)
//       float fNormDiff = vecSumDiff[0] / vecSumRef[0];
//       INFO_MSG("Normalized Sum-Difference of the Image: " << fNormDiff)
// #ifndef NO_SHOW_
//       imshow("Diff of consecutive images", mDiff);
// #endif
//       if (fNormDiff > 0.02f) {
//         bMotionDetected = true;
//       } else if (bMotionDetected) {
//         bMotionGone = true;
// #ifdef __arm__
//         ::system("/home/root/leds_blue_brightness.sh 512");
//         boost::this_thread::sleep(boost::posix_time::seconds(1));
//         ::system("/home/root/leds_blue_brightness.sh 0");
// #endif
//       }
//     } else if(KB_codeCV == KB_3 || KB_code == KB_3 || (bMotionGone && bImageRecieved && !tableTopDetection::bIsStable)) {
    } else if (KB_codeCV == KB_2 || KB_code == KB_2 || (bGotReferenceMask && bCleanup && !bGotCleanup)) {
      INFO_MSG("Start the cleanup")
      bGotCleanup = true;
#ifdef __arm__
        ::system("/home/root/leds_blue_brightness.sh 512");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ::system("/home/root/leds_blue_brightness.sh 0");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
#endif
//        exit(0);
    } else if (KB_codeCV == KB_3 || KB_code == KB_3 || (bGotCleanup && !tableTopDetection::bIsStable)) {
      INFO_MSG("Process Image to get new object")
      tableTopDetection::processImage(mInput, reference, referenceMask, differenceImageThreshold,
        minWidthThreshold, stableFrames, rectObject);
      mInput(rectObject).copyTo(mObject);
#ifdef __arm__
      if (tableTopDetection::bIsStable) {
        ::system("/home/root/leds_green_brightness.sh 512");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ::system("/home/root/leds_green_brightness.sh 0");
      }
#endif
    } else if ((KB_codeCV == KB_4 || KB_code == KB_4) || (tableTopDetection::bIsStable && bImageRecieved && (!bHeadingStop && !bTobiStandBy && !bFinish))) {
//       } else if ((KB_codeCV == KB_4 || KB_code == KB_4)) {
      INFO_MSG("Match the template")
      // Correlate only the left feature for testing
//       "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED"
      cv::Rect rMatch = templateMatching::templateMatching(0, mObject, mInput);
      mInput(rMatch).copyTo(mObject);

      // Heading
      objectMean = cv::Point2f(rMatch.x + rMatch.width / 2.f, rMatch.y + rMatch.height / 2.f);

//       float halfViewAngle_deg = 28.0f;
      // If the object is on the left/right side of
      // the frame, steer left/right, ...
      if (objectMean.x - mInput.cols / 2 >= 0)
        vecSteering->at(1) = -20;  // Degree per second
      else
        vecSteering->at(1) = 20;  // Degree per second
      // ... but if the object is more or less in the middle
      // then don't steer.
      if (abs(objectMean.x - mInput.cols / 2) < mInput.cols / 4)
        vecSteering->at(1) = 0;
      // Just go forward while homing
      vecSteering->at(0) = 30;  // Milimeter per second

      INFO_MSG("Homing the object at: " <<  objectMean)
      // Publish the steering command
      informerSteering->publish(vecSteering);
      // Publish the state
      informerRemoteHoming->publish(publishVoid);
      // Set a flag, that the bebot is heading
      bIsHeading = true;
    } else if (KB_codeCV == KB_5 || KB_code == KB_5 || (bHeadingStop && !bTobiStandBy && !bFinish && bIsHeading)) {
      INFO_MSG("Homing finish")
      // Publish the state
      informerRemoteHomingFinish->publish(publishVoid);
      // Publish no more movement
      vecSteering->at(1) = 0;
      vecSteering->at(0) = 0;
      informerSteering->publish(vecSteering);
      // Wait a second, we dont have to hasitate
      boost::this_thread::sleep(boost::posix_time::seconds(1));
#ifdef __arm__
      ::system("/home/root/leds_trigger_heartbeat.sh");
#endif
    } else if (KB_codeCV == KB_6 || KB_code == KB_6 || (bTobiStandBy && !bFinish)) {
      INFO_MSG("Go Back")
      // Publish the state
      vecSteering->at(1) = 0;
      vecSteering->at(0) = -30;
#ifdef __arm__
      ::system("/home/root/leds_trigger_none.sh");
      ::system("/home/root/leds_brightness.sh 0");
      ::system("/home/root/leds_red_brightness.sh 512");
#endif
      // go back for 30 cm * 6 s/cm = 18 cm
      for (int idx = 0; idx < 6; idx++) {
        informerSteering->publish(vecSteering);
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      // Set velocity to 0
      vecSteering->at(0) = 0;
      informerSteering->publish(vecSteering);

      bFinish = true;
    } else if (KB_codeCV == KB_7 || KB_code == KB_7 || bFinish) {
      INFO_MSG("Finish ")
      // Publish the state
      informerRemoteFinish->publish(publishVoid);
#ifdef __arm__
      ::system("/home/root/leds_trigger_none.sh");
      ::system("/home/root/leds_brightness.sh 0");
      ::system("/home/root/leds_green_brightness.sh 512");
#endif
    }
  }
  return 0;
}
