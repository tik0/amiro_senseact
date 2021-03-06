/*
 * This program does the tracking by first accuiring the
 * BGR (BlueGreenRed) image from the camer via Stemmer Imaging API.
 * Then the frame is copied to an OpenCV Mat conatainer.
 * On this copied frame, the ARToolKit does the tracking of the
 * trained marker. The ID and the location of recognised marker
 * is then send via RSB
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

// GL
#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glut.h>
#else
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#endif

// ARToolKit
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include "object.h"

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

// Proto types
#include <types/twbTracking.pb.h>

using namespace std;
using namespace cv;
using namespace boost;



/**
 * Variables
 */


static const std::size_t DRIVERPATHSIZE = 256;
const std::size_t colorChannels = 3;

// Camera object of Stemmer Camera API
IMG hCamera;

// Image from the camera
// It is filled by Stemmer Camera API and then given
// to ARToolKit
cv::Mat frame;


/**
 * ARToolKit setup
 */

// Marker list file of the robots
char *model_name = "Data/object_data";
ObjectData_T *object;
int objectnum;

#ifdef _WIN32
char *vconf = "Data/WDM_camera_flipV.xml";
#else
char *vconf = "";
#endif

// Threshold value (between 0-255) of arDetectMarker to be used to convert the
// input image into a binary image. (DO NOT CHANGE THIS VALUE. ONLY BY PROGRAM OPTIONS)
int thresh = 100;

// Threshold value (between 0.0-1.0) of the confidence factor for the recognized
// patterns. If the confidence factor is above this threashold, the marker is
// signed as recognized, otherwise not. (DO NOT CHANGE THIS VALUE. ONLY BY PROGRAM OPTIONS)
double confidenceFactor = 0.7;

// Publishing of tracking frames
static size_t publishCounter = 1;

// Only publish the marker list, if a marker has been detected
// otherwise, even emtpy lists will be published
static bool onlyPublishIfMarkerDetected = false;

// Camera parameter file
char *cparam_name = "Data/camera_parafinal.dat";
ARParam cparam;


/**
 * RSB setup
 */

// Create an informer that is capable of sending events
rsb::Informer<twbTracking::proto::Pose2DList>::Ptr informer;

// Scope for sending the data
std::string rsbOutScope = "/murox/roboterlocation";

int counter = 0;


/**
 * Prototypes
 */

// After configuration is done in main, this function loops for ever
static void processEverything(void);
// Processing the camera hCamera and write the image to the variable frame
void process();
// Do multi marker tracking on the variable frame. First display it, then send the data via RSB
void tracking();
// Init the AR Toolkit
static void initARToolkit(int xsize, int ysize);
// Cleanup AR Toolkit
static void cleanup(void);
// Handle program options
static void programOptions(int argc, char **argv);


// Video writer
static cv::VideoWriter recorder;
static bool doRecord = false;
static bool isRecording = false;
static std::size_t numFramesRecorded = 0;
static std::size_t numFrames = 0;
static std::string recordFilename = "video.avi";
static std::size_t fpsRecord = 0;

// Camera parameter
static double camGainAbs = 12.0f;
static cvbint64_t camExposureTimeRaw = 16000;
static cvbbool_t camReverseX = false;
static cvbbool_t camNoiseCorrection = false;
static cvbbool_t camDefectCorrection = false;
static cvbbool_t camNegativeImage = false;
static cvbint64_t camAcquisitionFrameRateAbs = 15;

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
        //std::cout << "frame:" << numFramesRecorded << std::endl;
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


// access a feature via CVGenApi
template<typename T>
void genicam_access(const std::string nodeName, const T value)
{
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
      if(std::is_same<T,double>::value)
        result = NSetAsFloat(hNode, value);
      else if(std::is_same<T,cvbbool_t>::value)
        result = NSetAsBoolean(hNode, value);
      else if(std::is_same<T,cvbint64_t>::value)
        result = NSetAsInteger(hNode, value);
//       else if(std::is_same<T,string>::value)
//         result = NSetAsString(hNode, const_cast<std::string>(value).c_str());
      else
        result = -1;

      if(result >= 0)
      {
        cout << "Node value set" << endl;
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

  // Register new converter for the pose list
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> >
      converter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Create an informer that is capable of sending events containing string data on the given scope.
  informer = factory.createInformer<twbTracking::proto::Pose2DList> (rsbOutScope);

  // Init glut (give it only one argument, because the arguments are already parsed by programOptions())
  int argcGlut = 1;
  glutInit(&argcGlut, argv);

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
  if (doRecord) {
    cv::Size frameSize(IMGwidth, IMGheight);
    initVideoRecorder(frameSize);
  }

  // Allocation of the BGR framedata
  frame.create(IMGheight, IMGwidth, CV_8UC3);

  // Init ARToolkit
  initARToolkit(IMGwidth, IMGheight);

  std::cout << "Frame Height: " << IMGheight << std::endl << "Frame Width:  " << IMGheight << std::endl;

  // Start grab with ring buffer
  cvbres_t result = G2Grab(hCamera);
  
    // access camera config
  if(CanNodeMapHandle(hCamera))
  {
    genicam_access(std::string("GainAbs"), camGainAbs);  // double
    genicam_access(std::string("ReverseX"), camReverseX);  // bool
    genicam_access(std::string("ExposureTimeRaw"), camExposureTimeRaw);  // int
    genicam_access(std::string("NoiseCorrection"), camNoiseCorrection);  // bool
    genicam_access(std::string("DefectCorrection"), camDefectCorrection);  // bool
    genicam_access(std::string("NegativeImage"), camNegativeImage);  // bool
    genicam_access(std::string("AcquisitionFrameRateAbs"), camAcquisitionFrameRateAbs);  // int
  }

  // Start with the tracking
  argMainLoop( NULL, NULL, processEverything);

  std::cout << "Stop acquisition" << std::endl;

  // stop the grab (kill = true: wait for ongoing frame acquisition to stop)
  result = G2Freeze(hCamera, true);

  // free camera
  ReleaseObject(hCamera);

  // free AR Toolkit
  cleanup();

  return 0;
}

static void programOptions(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending the robot localizations.")
    ("tresh,t", po::value < int > (&thresh), "Threshold value (between 0-255) of to be used to convert the input image into a binary image. Standard value: 100")
    ("confidenceFactor,c", po::value < double > (&confidenceFactor), "Threshold value (between 0.0-1.0) of the confidence factor for the recognized patterns. Standard value: 0.7")
    ("record,r", po::value < bool > (&doRecord), "Set value to 1 to record the video")
    ("recordFilename,f", po::value < std::string > (&recordFilename), "Filename of the video. Standard value: video.avi")
    ("numFrames,n", po::value < size_t > (&numFrames), "Numbers of frames to record (0 for no framenumber constraints). Standard value: 0")
    ("fpsRecord", po::value < size_t > (&fpsRecord), "Frames per second for replay. Standard value: camAcquisitionFrameRateAbs")
    ("publishCounter", po::value < size_t > (&publishCounter), "Publish the tracking result ever n'th frame. Standard value: 1")
    ("onlyPublishIfMarkerDetected", po::value < bool > (&onlyPublishIfMarkerDetected), "Publish only if marker has been detected. Standard value: 0")
    ("camGainAbs", po::value < double > (&camGainAbs), "Camera gain. Standard value: 12")
    ("camReverseX", po::value < cvbbool_t > (&camReverseX), "Reverse the x axis. Standard value: false")
    ("camExposureTimeRaw", po::value < cvbint64_t > (&camExposureTimeRaw), "Exposure time in µs. Standard value: 16000")
    ("camNoiseCorrection", po::value < cvbbool_t > (&camNoiseCorrection), "Noise correction feature. Standard value: false")
    ("camDefectCorrection", po::value < cvbbool_t > (&camDefectCorrection), "Defect correction feature. Standard value: false")
    ("camNegativeImage", po::value < cvbbool_t > (&camNegativeImage), "Invert the image. Standard value: false")
    ("camAcquisitionFrameRateAbs", po::value < cvbint64_t > (&camAcquisitionFrameRateAbs), "Frames per second. Standard value: 15");
    
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

  if (fpsRecord == 0)
    fpsRecord = size_t(camAcquisitionFrameRateAbs);
}

static void initARToolkit(int xsize, int ysize) {
  ARParam wparam;

  /* set the initial camera parameters */
  if (arParamLoad(cparam_name, 1, &wparam) < 0) {
    std::cout << "Camera parameter load error" << std::endl;
    exit(0);
  }
  arParamChangeSize(&wparam, xsize, ysize, &cparam);
  arInitCparam(&cparam);
  arParamDisp(&cparam);

  /* load in the object data - trained markers and associated bitmap files */
  if ((object = read_ObjData(model_name, &objectnum)) == NULL)
    exit(0);

  /* open the graphics window */
  argInit(&cparam, 1.0, 0, 0, 0, 0);
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
    // Do the tracking on the variable frame and send the locations via RSB
    tracking();
  }
}

static void cleanup(void) {
  arVideoCapStop();
  arVideoClose();
  argCleanup();
}

void tracking() {
  
  ++counter;

  rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);

  // Record the frame
  recordVideo(frame);
  stopRecordVideo();

  if(counter >= publishCounter) {
    std::cout << "---ID\t x\t y\t f---" << std::endl;
  }

  ARUint8 *dataPtr;
  ARMarkerInfo *marker_info;
  int marker_num;
  int idxMarker2Object;
  double rotat[2];
  double xt;
  double yt;
  double zt;
  double grad;
  double cosa, sina, atana;
  bool markerDetected = false;
  std::string robotCoordinates = "";

  // Grab a video frame
  dataPtr = (ARUint8 *) frame.data;

  // Draw the video
  argDrawMode2D();
  argDispImage(dataPtr, 0, 0);

  // Detect all the marker in the video frame
  if (arDetectMarker(dataPtr, thresh, &marker_info, &marker_num) < 0) {
    cleanup();
    exit(0);
  }

  // Check the patterns on the recognized markers and assigne an object from
  // the model_name file to them
  if (marker_num > 0) {
    for (size_t idxObject = 0; idxObject < objectnum /* Number of objects in model_name file */; idxObject++) {
      idxMarker2Object = -1;
      for (size_t idxMarker = 0; idxMarker < marker_num; idxMarker++) {
        // std::cout << "Objekt i:"  << object[i].id << " Marker j:" << marker_info[j].id << std::endl;
        if (object[idxObject].id == marker_info[idxMarker].id) {
          /* you've found a pattern, so a blue square is drawn around the object*/
          glColor3f(0.0, 0.0, 1.0);
          argDrawSquare(marker_info[idxMarker].vertex, 0, 0);
          if (idxMarker2Object == -1)
            idxMarker2Object = idxMarker;
          else /* make sure you have the best pattern (highest confidence factor) */
          if (marker_info[idxMarker2Object].cf < marker_info[idxMarker].cf)
            idxMarker2Object = idxMarker;
        }
      }

      // If the detected square has a low confidence factor regarding an
      // marker from the list, then it will not be precessed
      if (marker_info[idxMarker2Object].cf < confidenceFactor) {
        object[idxObject].visible = 0;
        continue;
      } else {
        markerDetected = true;
      }

      // If the confidence factor is high enough, the square around the object
      // becomes yellow, and the transformation will be calculated
      glColor3f(1.0, 1.0, 0.0);
      argDrawSquare(marker_info[idxMarker2Object].vertex, 0, 0);
      /* calculate the transform for each marker */
      if (object[idxObject].visible == 0) {
        arGetTransMat(&marker_info[idxMarker2Object], object[idxObject].marker_center,
                      object[idxObject].marker_width, object[idxObject].trans);
      } else {
        arGetTransMatCont(&marker_info[idxMarker2Object], object[idxObject].trans,
                          object[idxObject].marker_center, object[idxObject].marker_width,
                          object[idxObject].trans);
      }

      // Calculate the position out of the detected marker
      rotat[0] = object[idxObject].trans[0][0];
      rotat[1] = object[idxObject].trans[0][1];

      cosa = acos(rotat[0]);
      sina = asin(rotat[1]);
      if (sina < 0) {
        atana = M_PI * 2 - cosa;
      } else {
        atana = cosa;
      }
      grad = atana * 360 / (M_PI * 2);
      object[idxObject].marker_grad = grad;
      object[idxObject].marker_pos[0] = marker_info[idxMarker2Object].pos[0];
      object[idxObject].marker_pos[1] = marker_info[idxMarker2Object].pos[1];
      object[idxObject].visible = 1;

      // TODO Why are they incremented by 0.5?
      object[idxObject].marker_pos[0] = object[idxObject].marker_pos[0] + 0.5;
      object[idxObject].marker_pos[1] = object[idxObject].marker_pos[1] + 0.5;
      object[idxObject].marker_grad = object[idxObject].marker_grad + 0.5;

     if(counter >= publishCounter)
     {
      // Print out the found position
      std::cout
          << "   "
          << object[idxObject].id
          << "\t "
          << (int) object[idxObject].marker_pos[0]
          << "\t "
          << 1000 - (int) object[idxObject].marker_pos[1]
          << "\t "
          << 360 - (int) object[idxObject].marker_grad
          << std::endl;

        // Add a new robot to the list
        twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
        pose2D->set_x(float(object[idxObject].marker_pos[0]));
        pose2D->set_y(float(1000 - object[idxObject].marker_pos[1]));
        pose2D->set_orientation(float(360 - object[idxObject].marker_grad));
        pose2D->set_id(object[idxObject].id);
      }
    }
  }

  if((!onlyPublishIfMarkerDetected || markerDetected) && counter >= publishCounter) {
    // Send the data
    informer->publish(pose2DList);
    counter = 0;
  }

  /*swap the graphics buffers*/
  argSwapBuffers();
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

  // Copy the BGR frame out of the ring buffer
  if (lpBaseAddress != 0) {
    memcpy(&frame.at<char>(0), (char*) lpBaseAddress, IMGwidth * IMGheight * colorChannels);
  } else {
    std::cout << "BaseAddress is ZERO" << std::endl;
    //throw "baseAddress is ZERO";
  }
}
