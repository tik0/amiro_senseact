/*
 * Converter from GigEVision frames to RSB
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <math.h>

// Stemmer Imaging API
#include <iCVCDriver.h>
#include <iCVCImg.h>
#include <iCVGenApi.h>
#include <CVCError.h>
#include <iCVCUtilities.h>

// openCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <converter/iplImageConverter/IplImageConverter.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

using namespace std;
using namespace cv;
using namespace boost;
using namespace rsb;
using namespace rst::converters::opencv;
using namespace rsb::converter;

/**
 * Variables
 */


static const std::size_t DRIVERPATHSIZE = 256;
const std::size_t colorChannels = 1;

// Camera object of Stemmer Camera API
IMG hCamera;

// Image from the camera
cv::Mat frame;


/**
 * RSB setup
 */

// Create an informer that is capable of sending events
rsb::Informer<IplImage>::Ptr informer;

// Scope for sending the data
std::string rsbOutScope = "/gigevision/cam";


/**
 * Prototypes
 */

// After configuration is done in main, this function loops for ever
static void processEverything(void);
// Processing the camera hCamera and write the image to the variable frame
void process();
// Do multi marker tracking on the variable frame. First display it, then send the data via RSB
void tracking();
// Handle program options
static void programOptions(int argc, char **argv);


// Video writer

cv::VideoWriter recorder;
bool doRecord = false;
bool isRecording = false;
std::size_t numFramesRecorded = 0;
std::size_t numFrames = 0;
std::string recordFilename = "video.avi";
std::size_t fpsRecord = 30;

// Camera parameter
static cvbint64_t camStreamBytesPerSecond = 12400000;
static cvbint64_t camPixelFormat = 17301505;
static cvbint64_t camWidth = 2048;
static cvbint64_t camHeight = 1088;
static cvbint64_t camOffsetX = 0;
static cvbint64_t camOffsetY = 0;
static cvbint64_t camExposureTimeAbs = 16000;
static cvbint64_t camExposureAuto = 0;
static double camGain = 0.00;
static cvbint64_t camGainAuto = 0;
static double camBlackLevel = 4.00;
static double camGamma = 1.00;

void initVideoRecorder(cv::Size frameSize)
{
  if(!isRecording)
  {
    numFramesRecorded = 0;
    recorder = cv::VideoWriter(recordFilename, CV_FOURCC('D','I','V','X'), fpsRecord, frameSize);
  }

  isRecording = true;
}

void recordVideo(cv::Mat frame)
{
  if(isRecording)
  {
    ++numFramesRecorded;
//     std::cout << "frame:" << numFramesRecorded << std::endl;
    recorder.write(frame);
  }
}

void stopRecordVideo()
{
  if((isRecording && numFramesRecorded >= numFrames) && (numFrames != 0))
  {
    isRecording = false;
    numFramesRecorded = 0;
    recorder.release();
    std::cout << "finish video recording!" << std::endl;
  }
}


// reads a feature of type T via CVGenApi
template<typename T>
void genicam_read(const std::string nodeName, const T value)
{
  cout << "Get "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if(result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if(result >= 0)
    {
      if(typeid(value) == typeid(double)) {
          double val = 0.0;
           result = NGetAsFloat(hNode, val);
           cout << val << " (double)" << endl;
      } else if(typeid(value) == typeid(cvbbool_t)) {
          cvbbool_t val = false;
           result = NGetAsBoolean(hNode, val);
           cout << val << " (bool)" << endl;
      } else if(typeid(value) == typeid(cvbint64_t)) {
            cvbint64_t val = 0;
           result = NGetAsInteger(hNode, val);
           cout << val << " (int)" << endl;
      } else if(typeid(value) == typeid(string)) { //String is not working because compiler think its double ? run time issue
            char* val;
            size_t len;
           result = NGetAsString(hNode, val, len);
           try {
           cout << string(val,len) << " (string)" << endl;
           } catch(std::logic_error ex) {
               cout << "error at accessing string " << nodeName << endl;
           }
           } else
        result = -1;

      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
  }
}


// access a feature via CVGenApi
template<typename T>
void genicam_access(const std::string nodeName, const T value)
{
    genicam_read(nodeName, value);
  cout << "Set "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if(result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if(result >= 0)
    {
      if(typeid(value) == typeid(double)) {
           void* temp =  (void*)&value;
           double tempstr = *(double*)temp;
           result = NSetAsFloat(hNode, tempstr);
      } else if(typeid(value) == typeid(cvbbool_t)) {
           void* temp =  (void*)&value;
           cvbbool_t tempstr = *(cvbbool_t*)temp;
           result = NSetAsBoolean(hNode, tempstr);

      } else if(typeid(value) == typeid(cvbint64_t)) {
           void* temp =  (void*)&value;
           cvbint64_t tempstr = *(cvbint64_t*)temp;
           result = NSetAsInteger(hNode, tempstr);
      } else if(typeid(value) == typeid(string)) { //String is not working because compiler think its double ? run time issue
           void* temp =  (void*)&value;
           string tempstr = *(string*)temp;
           result = NSetAsString(hNode, tempstr.c_str());
       } else
        result = -1;

      if(result >= 0)
      {
        cout << "Node value set to "<< value << endl;
      }
      else
      {
        cout << "Node value error: " << CVC_ERROR_FROM_HRES(result) << endl;
      }

      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
  }
}




int main(int argc, char **argv) {

  programOptions(argc, argv);

  // Init RSB
  // First get a factory instance that is used to create RSB domain objects
  rsb::Factory& factory = rsb::getFactory();

  // Register new converter for the image
  boost::shared_ptr<IplImageConverter> converter(new IplImageConverter());
  converterRepository<std::string>()->registerConverter(converter);

  // Create an informer that is capable of sending events containing string data on the given scope.
  informer = factory.createInformer<IplImage> (rsbOutScope);

  char driverPath[DRIVERPATHSIZE] = { 0 };

  // Get Driver from arguments
  TranslateFileName("%CVB%/drivers/GenICam.vin", driverPath, DRIVERPATHSIZE);
  LoadImageFile(driverPath, hCamera);

  // Load the camera via Stemmer API
  cvbbool_t success = LoadImageFile(driverPath, hCamera);
  if (!success) {
    std::cout << "Error loading " << driverPath << " driver!" << std::endl;
    return 1;
  }
  std::cout << "Load " << driverPath << " successful." << std::endl;

  // Height & Width of the camera image
  long IMGheight = ImageHeight(hCamera);
  long IMGwidth = ImageWidth(hCamera);

  // Init the video recorder
  cv::Size frameSize(IMGwidth, IMGheight);
  initVideoRecorder(frameSize);

  // Allocation of the BGR framedata
  frame.create(IMGheight, IMGwidth, CV_8UC1);

  std::cout << "Frame Height: " << IMGheight << std::endl << "Frame Width:  " << IMGwidth << std::endl;

  // Start grab with ring buffer
  cvbres_t result = G2Grab(hCamera);

 
      // access camera config
  if(CanNodeMapHandle(hCamera))
  {
    genicam_access(std::string("StreamBytesPerSecond"), camStreamBytesPerSecond);
    genicam_access(std::string("PixelFormat"), camPixelFormat);
    genicam_access(std::string("Width"),camWidth);
    genicam_access(std::string("Height"), camHeight);
    genicam_access(std::string("OffsetX"), camOffsetX);
    genicam_access(std::string("OffsetY"), camOffsetY);
    genicam_access(std::string("ExposureTimeAbs"), camExposureTimeAbs);
    genicam_access(std::string("ExposureAuto"), camExposureAuto);
    genicam_access(std::string("Gain"), camGain);
    genicam_access(std::string("GainAuto"), camGainAuto);
    genicam_access(std::string("BlackLevel"), camBlackLevel);
    genicam_access(std::string("Gamma"), camGamma);
  }

  
  
  // Start with the grabbing
  for (;;) {
    processEverything();
  }

  std::cout << "Stop acquisition" << std::endl;

  // stop the grab (kill = true: wait for ongoing frame acquisition to stop)
  result = G2Freeze(hCamera, true);

  // free camera
  ReleaseObject(hCamera);

  return 0;
}

static void programOptions(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending the robot localizations.")
    ("record,r", po::value < bool > (&doRecord), "Set value to 1 to record the video")
    ("recordFilename,f", po::value < std::string > (&recordFilename), "Filename of the video. Standard value: video.avi")
    ("numFrames,n", po::value < size_t > (&numFrames), "Numbers of frames to record (0 for no framenumber constraints). Standard value: 0")
    ("fpsRecord", po::value < size_t > (&fpsRecord), "Frames per second for replay. Standard value: camAcquisitionFrameRateAbs")
    ("camStreamBytesPerSecond", po::value < cvbint64_t> (&camStreamBytesPerSecond), "Camera option.")
    ("camPixelFormat", po::value < cvbint64_t > (&camPixelFormat), "Camera option.")
    ("camWidth", po::value < cvbint64_t> (&camWidth), "Camera option.")
    ("camHeight", po::value <cvbint64_t > (&camHeight), "Camera option.")
    ("camOffsetX", po::value < cvbint64_t> (&camOffsetX), "Camera option.")
    ("camOffsetY", po::value < cvbint64_t> (&camOffsetY), "Camera option.")
    ("camExposureTimeAbs", po::value <cvbint64_t > (&camExposureTimeAbs), "Camera option.")
    ("camExposureAuto", po::value < cvbint64_t> (&camExposureAuto), "Camera option.")
    ("camGain", po::value <double > (&camGain), "Camera option.")
    ("camGainAuto", po::value < cvbint64_t> (&camGainAuto), "Camera option.")
    ("camBlackLevel", po::value < double> (&camBlackLevel), "Camera option.")
    ("camGamma", po::value < double> (&camGamma), "Camera option.");

  
  
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
}

static void processEverything(void) {
  // Wait for next image to be acquired
  // (returns immediately if unprocessed image are in the ring buffer)
  cvbres_t result = G2Wait(hCamera);
  if (result < 0) {
    std::cout << " Error with G2Wait";
  } else {
    // Grab the image and copy it to the variable frame
    process();
    // Do image sending via RSB

    // Record
    recordVideo(frame);
    stopRecordVideo();

    // Send the data
    boost::shared_ptr<IplImage> iplImage(new IplImage(frame));
    informer->publish(iplImage);
  }
}

void process() {
  // Height & Width of the camera image
  long IMGheight = ImageHeight(hCamera);
  long IMGwidth = ImageWidth(hCamera);

  // Getting the address of the camera source image
  void *lpBaseAddress = 0;
  intptr_t lXInc;
  intptr_t lYInc;
  cvbbool_t status = GetLinearAccess(hCamera, 0, &lpBaseAddress, &lXInc,
                                     &lYInc);
  if (!status) {
    std::cout << "Error GetLinearAccess" << std::endl;
    //throw ("Error accessing camera image");
  }

  // Copy the frame out of the ring buffer
  if (lpBaseAddress != 0) {
    memcpy(&frame.at<char>(0), (char*) lpBaseAddress, IMGwidth * IMGheight * colorChannels);
  } else {
    std::cout << "BaseAddress is ZERO" << std::endl;
    //throw "baseAddress is ZERO";
  }
}
