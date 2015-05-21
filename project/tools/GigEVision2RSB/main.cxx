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

int main(int argc, char **argv) {

  programOptions(argc, argv);

  // Init RSB
  // First get a factory instance that is used to create RSB domain objects
  rsb::Factory& factory = rsb::getFactory();

  // Register new converter for the image
  shared_ptr<IplImageConverter> converter(new IplImageConverter());
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
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending the images.")
    ("record,r", po::value < bool > (&doRecord), "Set value to 1 to record the video")
    ("recordFilename,f", po::value < std::string > (&recordFilename), "Filename of the video. Standard value: video.avi")
    ("numFrames,n", po::value < size_t > (&numFrames), "Numbers of frames to record (0 for no framenumber constraints). Standard value: 0")
    ("fpsRecord", po::value < size_t > (&fpsRecord), "Frames per second for replay. Standard value: 30");

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
    shared_ptr<IplImage> iplImage(new IplImage(frame));
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
