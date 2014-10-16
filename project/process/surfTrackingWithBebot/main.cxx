// Messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// Programoptions
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
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// If the program compiles for the ARM architecture
// then throw away all image output
#ifdef __arm__
#define NO_SHOW_
#endif

// For checking character pressed in the console
#include <kbhit.hpp>

// Object detection with SURF
#include "objectDetection.hpp"

// The tableTopDetection by Leon Ziegler
#include "tableTopDetection.hpp"

// For template matching on the horizont
#include "templateMatching.hpp"

// START: Template matching
/// Global Variables

// The input image which is updated every time an image is recieved
cv::Mat mInput;
// The object for homing which is detected by processImage()
cv::Mat mObject;
// The rectangular for the object
cv::Rect rectObject;

/// Function Headers
void MatchingMethod( int match_method, cv::Mat &mTemplate, cv::Mat &mImage  );
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
const int stableFrames = 10;
int processStream( void );

int main(int argc, char **argv) {

#ifndef NO_SHOW_
	// Preview Windows
	namedWindow("reference mask", WINDOW_AUTOSIZE);
	namedWindow("result", WINDOW_AUTOSIZE);
	namedWindow("cam", WINDOW_AUTOSIZE);
#endif

	// use camera stream for testing
	 return processStream();
	 
}

int processStream( void ) {

	  ////////////////////////////////////////////////////////////////////////////////////////////////////
	  // Register our converter within the collection of converters for
	  // the string wire-type (which is used for arrays of octets in
	  // C++).
	  boost::shared_ptr<MatConverter> converter(new MatConverter());
	  converterRepository<std::string>()->registerConverter(converter);


	  rsb::Factory &factory = rsb::Factory::getInstance();

	  // Create and start the listener
	  rsb::ListenerPtr listener = factory.createListener("/image");
	  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<cv::Mat> > > imageQueue(
			      new rsc::threading::SynchronizedQueue<boost::shared_ptr<cv::Mat> >(1));

	  listener->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<cv::Mat>(imageQueue)));
	  
	  
	  // Register new converter for std::vector<int>
	  shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	  converterRepository<std::string>()->registerConverter(converterVecInt);

	  // Prepare RSB informer
	  rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > ("/proc/steering/raw");

	  // Calculate the new steering (two elements with 0 initialized)
	  boost::shared_ptr< std::vector<int> > vecSteering(new std::vector<int> (2,0));
	  ////////////////////////////////

	// get reference image
// 	cap >> input;
	  mInput = imageQueue->pop().get()->clone();
// 	  mInput = imageQueue->pop().get()->copyTo(mInput);
	Mat reference;
	mInput.copyTo(reference);
// 	
	Mat referenceMask = tableTopDetection::createReferenceMask(reference, regionGrowingThreshold);
#ifndef NO_SHOW_
	imshow("reference mask", referenceMask);
#endif

	while (true) {

		// get image to test
// 		cap >> mInput;
// 		mInput = *imageQueue->pop().get();
		mInput = *imageQueue->pop().get();
#ifndef NO_SHOW_
		imshow("cam", mInput);
#endif
		INFO_MSG( "Received image" )
		
		// Check for keypress in winow
		int key = waitKey(30);
		
		// Check for keypress in terminal
		int KB_code = 0;
		if(kbhit()){
		  KB_code = getchar();
		  INFO_MSG( "KB_code = " << KB_code )
		}
		  
		// Process the keys
		if(key == KB_1 || KB_code == KB_1) {
		  INFO_MSG( "Create new refernce mask" )
		  mInput.copyTo(reference);
		  Mat referenceMask = tableTopDetection::createReferenceMask(reference, regionGrowingThreshold);
#ifndef NO_SHOW_
		  imshow("reference mask", referenceMask);
#endif
		}
		if(key == KB_2 || KB_code == KB_2) {
		  INFO_MSG( "Process Image to get new object" )
		  tableTopDetection::processImage(mInput, reference, referenceMask, differenceImageThreshold,
				minWidthThreshold, stableFrames, rectObject);
		  mInput(rectObject).copyTo(mObject);
		}
		if (key == KB_3 || KB_code == KB_3) {
		  INFO_MSG( "Do SURF detection" )
		  objectMean = cv::Point2f(0.f, 0.f);
		  objectVar = cv::Point2f(0.f, 0.f);
		  objectDetection::detection( mObject,mInput , objectMean, objectVar);
		  // Heading
		  INFO_MSG( "Object mean (x,y): " << objectMean )
		  INFO_MSG( "Object std.dev. (x,y): " << cv::Point2f(sqrt(objectVar.x), sqrt(objectVar.y)) )
		  float halfViewAngle_deg = 28.0f;
		  vecSteering->at(0) = 30;
		  if (objectMean.x - mInput.cols / 2 > 0)
		    vecSteering->at(1) = -20;
		  else
		    vecSteering->at(1) = 20;
		  if (abs(objectMean.x - mInput.cols / 2) < mInput.cols / 4)
		    vecSteering->at(1) = 0;
		  
		  vecSteering->at(1) = 0;
		  informer_vec->publish(vecSteering);
		  
		}
		if(key == KB_Q || KB_code == KB_Q) {
		  INFO_MSG( "Create teamplate for template matching: Press Q for matching" )
		  templateMatching::horizontTemplateCreating( rectObject.tl().x, /*templateHight*/ 6, /*templateWidth*/ 40, mInput );
		}
		if (key == KB_W || KB_code == KB_W) {
		  INFO_MSG( "Match the template" )
		  // Correlate only the left feature for testing
// 		  "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED"
		  templateMatching::horizontTemplateMatching( /*Method*/ 0, mInput );
		}
	}

	return 0;
}