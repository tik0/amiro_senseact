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

// OTHER
#include "ArgHandler.hpp"

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
      } else if(typeid(value) == typeid(string)) { 
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



void genicam_read_int(const std::string nodeName)
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
        cvbint64_t val = 0;
           result = NGetAsInteger(hNode, val);
           cout << val << " (int)" << endl;
  

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

void genicam_read_double(const std::string nodeName)
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
          double val = 0.0;
           result = NGetAsFloat(hNode, val);
           cout << val << " (double)" << endl;
  

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

void genicam_read_bool(const std::string nodeName)
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
   cvbbool_t val = false;
           result = NGetAsBoolean(hNode, val);
           cout << val << " (bool)" << endl;
  

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

void genicam_read_str(const std::string nodeName)
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
            char* val;
            size_t len;
           result = NGetAsString(hNode, val, len);
           try {
           cout << string(val,len) << " (string)" << endl;
           } catch(std::logic_error ex) {
               cout << "error at accessing string " << nodeName << endl;
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
// access a feature via CVGenApi
template<typename T>
void genicam_access( std::string nodeName, T value)
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
void genicam_access_int( std::string nodeName, int value)
{
    genicam_read_int(nodeName);
  cout << "Set int "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if(result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if(result >= 0)
    {

           result = NSetAsInteger(hNode, value);


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
void genicam_access_double( std::string nodeName, double value)
{
    genicam_read_double(nodeName);
  cout << "Set double "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if(result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if(result >= 0)
    {

           result = NSetAsFloat(hNode, value);


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
void genicam_access_bool( std::string nodeName, bool value)
{
    genicam_read_bool(nodeName);
  cout << "Set bool "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if(result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if(result >= 0)
    {

           result = NSetAsBoolean(hNode, value);


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
void genicam_access_str( std::string nodeName, string value)
{
    genicam_read_str(nodeName);
  cout << "Set str "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if(result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if(result >= 0)
    {

           result = NSetAsString(hNode, value.c_str());


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


    ArgHandler argHandle;


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
      vector<pair<string,string>> args = argHandle.getValuesByPrefix("cam");
      
      for(vector<pair<string,string>>::iterator it = args.begin();it != args.end();++it) {
          //cout << genicam_read(it->first,string("")) << endl;
          if(argHandle.isBoolean(it->second)) {
          genicam_access_bool(it->first,argHandle.toBoolean(it->second));
          }
          if(argHandle.isInteger(it->second)) {
          genicam_access_int(it->first,argHandle.toInteger(it->second));
          }
          if(argHandle.isDouble(it->second)) {
          genicam_access_double(it->first,argHandle.toDouble(it->second));
          }
          if(argHandle.isString(it->second)) {
          genicam_access_str(it->first,it->second);
          }
      }
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

    argHandle.processArgs(argc,argv);
    
    argHandle.coutArgs();
    
    if(argHandle.hasIdentifier("outscope")) {
        rsbOutScope = argHandle.getValueAsString("outscope");
        cout << "outscope is now " << rsbOutScope << endl;
    }
    if(argHandle.hasIdentifier("record")) {
        doRecord = argHandle.toBoolean(argHandle.getValueAsString("record"));
        cout << "record is now " << doRecord << endl;

    }
    if(argHandle.hasIdentifier("recordFilename")) {
        recordFilename = argHandle.getValueAsString("recordFilename");
        cout << "recordFilename is now " << recordFilename << endl;

    }
    if(argHandle.hasIdentifier("numFrames")) {
        numFrames = argHandle.toInteger(argHandle.getValueAsString("numFrames"));
                cout << "numFrames is now " << numFrames << endl;
    }
    if(argHandle.hasIdentifier("fpsRecord")) {
        fpsRecord = argHandle.toInteger(argHandle.getValueAsString("fpsRecord"));
        cout << "fpsRecord is now " << fpsRecord << endl;

    }

    
    
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
