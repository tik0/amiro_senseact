#include <iostream>
#include <iomanip>
#include <vector>
#include <iCVCDriver.h>
#include <iCVCImg.h>
#include <iCVGenApi.h>
#include <CVCError.h>
#include <iCVCUtilities.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>

using namespace std;
using namespace cv;

static const size_t DRIVERPATHSIZE = 256;

// see implementation below on how to access a camera via GenICam GenApi
static void genicam_access(IMG hCamera);

// entry function
int main(int argc, char* argv[])
{  
  // load the camera
  char driverPath[DRIVERPATHSIZE] = { 0 };
  TranslateFileName("%CVB%/drivers/GenICam.vin", driverPath, DRIVERPATHSIZE);
  IMG hCamera = NULL;
  
  bool success = LoadImageFile(driverPath, hCamera); 
  if(!success)
  {
    cout << "Error loading " << driverPath << " driver!" << endl;
    return 1;
  }
  cout << "Load " << driverPath << " successful." << endl;
   
  // access camera config
  if(CanNodeMapHandle(hCamera))
  {
    genicam_access(hCamera);
  }
   
  // start grab with ring buffer
  cout << "start acquiring images" << endl;
  
  int IMGheight = ImageHeight(hCamera);
  int IMGwidth  = ImageWidth(hCamera); 
  
  cvbres_t result = G2Grab(hCamera); 
  
  struct timeval start, end;
  int counter = 0;
  
  if(result >= 0)
  {
    gettimeofday(&start, NULL);
    // acquiring images
    while(true)
    {
      // wait for next image to be acquired
      // (returns immediately if unprocessed images are in the ring buffer)
      result = G2Wait(hCamera);
      if(result < 0)
      {
        cout << setw(3) << counter << " Error with G2Wait: " << CVC_ERROR_FROM_HRES(result) << endl;
      }
      else
      {
        // do image processing
        void *pBase = NULL;
        intptr_t xInc = 0;
        intptr_t yInc = 0;
        GetLinearAccess(hCamera, 0, &pBase, &xInc, &yInc);
        //cout << setw(3) << counter << " Acquired image @" << hex << pBase << endl;
        
        
        //show live image
        Mat rawImage(IMGheight, IMGwidth, CV_8UC3, (char*) pBase);
        //Size size(960, 540);
        //resize(rawImage, smallImage, size);
        cvtColor(rawImage, rawImage, CV_BGR2RGB);
        imshow("GE-CAM", rawImage);
        waitKey(1);
        
        //calc frames per seconds
        counter++;
        if (counter == 30) {
            gettimeofday(&end, NULL);
            cout << setw(3) << " FPS: " << 1000/(((end.tv_sec - start.tv_sec)*1000 + (end.tv_usec - start.tv_usec) / 1000)/30) << " | " << IMGheight << "x" << IMGwidth << endl;
            counter = 0;
            gettimeofday(&start, NULL);
        }
        
      }
    } 

    cout << "Stop acquisition" << endl;
    // stop the grab (kill = false: wait for ongoing frame acquisition to stop)
    result = G2Freeze(hCamera, true);
  }
  else
  {
    cout << "Error starting acquisition!" << endl;
  }

  // free camera
  cout << "Free camera" << endl;
  ReleaseObject(hCamera); 

  return 0;
}

// access a feature via CVGenApi
void genicam_access(IMG hCamera)
{
  //cout << "WIDTH: ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if(result >= 0)
  {
    // get width feature node
    NODE hWidth = NULL;
    result = NMGetNode(hNodeMap, "Width", hWidth);
    if(result >= 0)
    {
      // value camera dependent
      cvbint64_t widthValue = 0;
      result = NGetAsInteger(hWidth, widthValue);
      // set values via NSetAsInteger(hWidht, widthValue);
      if(result >= 0)
      {
        //cout << "WIDTH  : " << widthValue << endl;
          cout << "camera ready"  << endl;
      }
      else
      {
        cout << "Read value error: " << CVC_ERROR_FROM_HRES(result) << endl;
      }

      ReleaseObject(hWidth);
    }
    else
    {
      cout << "Get node error: " << CVC_ERROR_FROM_HRES(result) << endl;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Get nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
  }
}
