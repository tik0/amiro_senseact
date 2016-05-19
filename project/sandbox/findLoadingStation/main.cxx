//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : -
//============================================================================

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>

// For checking character pressed in the console
#include <kbhit.hpp>

// protocol defines
std::string COMMAND_QUIT = "ESC";
std::string COMMAND_FIND = "FIND";
std::string COMMAND_LOAD = "LOAD";
std::string COMMAND_THRESHOLD = "threshold_";


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
// using namespace muroxConverter;
using namespace rsb::converter;
using namespace cv;


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

#include <jpeglib.h>

static std::string g_sImageScope = "/objectDetection/image";
static std::string g_sOutScope = "/objectDetection/detected";
static std::string g_sInScope = "/objectDetection/command";
static int g_iDevice = 0;
static unsigned int g_uiQuality = 85;

// init flags
bool sendingPic = false;
bool debugging = false;


int binaryThreshold = 100;





// load objects
Mat loadImage(string imageName) {
	INFO_MSG("Try to load image " << imageName);
	Mat frame = imread(imageName, 1);
	if (frame.rows > 0 && frame.cols > 0) {
		INFO_MSG("Image " << imageName << " loaded.");
	} else {
		WARNING_MSG("Image " << imageName << " could not be loaded!");
	}
	return frame;
}


Mat doBlobDetection(Mat frame, SimpleBlobDetector detector) {
	Mat grayImage, grayImage3C;

	// Get image channels
	Mat chans[3];
	split(frame, chans);

	// Convert image
	cvtColor(frame, grayImage, CV_BGR2GRAY);

//	threshold(chans[2], binImage, binaryThreshold, 255, 0);
//	INFO_MSG(" -> Binary image created: " << binImage.size().width << "/" << binImage.size().height);

	cvtColor(grayImage, grayImage3C, CV_GRAY2BGR);

	std::vector<KeyPoint> keypoints;
	detector.detect(grayImage3C, keypoints);
	drawKeypoints(grayImage3C, keypoints, grayImage3C, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	INFO_MSG(" -> Blob Detection: " << keypoints.size() << " blobs detected.");

	// Search for best pairs
	int bestPartner[keypoints.size()];
	for (int i=0; i<keypoints.size(); i++) {
		bestPartner[i] = -1;
	}
	for (int curPoint=0; curPoint<keypoints.size(); curPoint++) {
		for (int comp=0; comp<keypoints.size(); comp++) {
			if (comp == curPoint) continue; 
			if (bestPartner[curPoint] == -1) {
				bestPartner[curPoint] = comp;
			} else {
				float maxSize = max(keypoints[curPoint].size, keypoints[comp].size);
				float sizeDiff = keypoints[curPoint].size - keypoints[comp].size;
				if (sizeDiff < 0.0) sizeDiff *= -1.0;
				float actDistDiff = keypoints[curPoint].pt.y - keypoints[bestPartner[curPoint]].pt.y;
				if (actDistDiff < 0.0) actDistDiff *= -1.0;
				float compDistDiff = keypoints[curPoint].pt.y - keypoints[comp].pt.y;
				if (compDistDiff < 0.0) compDistDiff *= -1.0;

				if (sizeDiff/maxSize < 0.15 && compDistDiff < actDistDiff) {
					bestPartner[curPoint] = comp;
				}
			}
		}
	}

	std::vector<int> partner1;
	std::vector<int> partner2;
	// Mark partners to pairs, which fit with each other
	for (int curPoint=0; curPoint<keypoints.size(); curPoint++) {
		if (bestPartner[bestPartner[curPoint]] == curPoint && bestPartner[curPoint] > curPoint) {
			Point pt1 = keypoints[curPoint].pt;
			float size1 = keypoints[curPoint].size;
			Point pt2 = keypoints[bestPartner[curPoint]].pt;
			float size2 = keypoints[bestPartner[curPoint]].size;
			INFO_MSG(" -> Partners " << curPoint << "-" << bestPartner[curPoint] << " (" << pt1.x << "/" << pt1.y << " - " << pt2.x << "/" << pt2.y << ", " << size1 << " - " << size2 << ")");
			line(grayImage3C, pt1, pt2, Scalar(0,255,0), 2);

			partner1.push_back(curPoint);
			partner2.push_back(bestPartner[curPoint]);
		}
	}

	// Check for two pairs, which fit best
	int bestPairs[partner1.size()];
	for (int i=0; i<partner1.size(); i++) {
		bestPairs[i] = -1;
	}
	for (int curPair=0; curPair<partner1.size(); curPair++) {
		for (int comp=0; comp<partner1.size(); comp++) {
			if (curPair == comp) continue;
			int xMin = min(keypoints[partner1[curPair]].pt.x, keypoints[partner2[curPair]].pt.x);
			int xMax = max(keypoints[partner1[curPair]].pt.x, keypoints[partner2[curPair]].pt.x);
			int xComp1 = keypoints[partner1[comp]].pt.x;
			int xComp2 = keypoints[partner2[comp]].pt.x;
			if (xComp1 > xMin && xComp1 < xMax && xComp2 > xMin && xComp2 < xMax) {
				if (bestPairs[curPair] < 0) {
					bestPairs[curPair] = comp;
				} else {
					WARNING_MSG("For pair " << curPair << " the pair " << comp << " is also interesting!");
				}
			}
		}
	}
	for (int i=0; i<partner1.size(); i++) {
		if (bestPairs[i] >= 0) {
			int j = bestPairs[i];
			int xMin = min(min(keypoints[partner1[i]].pt.x-keypoints[partner1[i]].size, keypoints[partner2[i]].pt.x-keypoints[partner2[i]].size), min(keypoints[partner1[j]].pt.x-keypoints[partner1[j]].size, keypoints[partner2[j]].pt.x-keypoints[partner2[j]].size));
			int xMax = max(max(keypoints[partner1[i]].pt.x+keypoints[partner1[i]].size, keypoints[partner2[i]].pt.x+keypoints[partner2[i]].size), max(keypoints[partner1[j]].pt.x+keypoints[partner1[j]].size, keypoints[partner2[j]].pt.x+keypoints[partner2[j]].size));
			int yMin = min(min(keypoints[partner1[i]].pt.y-keypoints[partner1[i]].size, keypoints[partner2[i]].pt.y-keypoints[partner2[i]].size), min(keypoints[partner1[j]].pt.y-keypoints[partner1[j]].size, keypoints[partner2[j]].pt.y-keypoints[partner2[j]].size));
			int yMax = max(max(keypoints[partner1[i]].pt.y+keypoints[partner1[i]].size, keypoints[partner2[i]].pt.y+keypoints[partner2[i]].size), max(keypoints[partner1[j]].pt.y+keypoints[partner1[j]].size, keypoints[partner2[j]].pt.y+keypoints[partner2[j]].size));

			Point pt1(xMin, yMin);
			Point pt2(xMin, yMax);
			Point pt3(xMax, yMax);
			Point pt4(xMax, yMin);
			line(grayImage3C, pt1, pt2, Scalar(0,255,255), 2);
			line(grayImage3C, pt2, pt3, Scalar(0,255,255), 2);
			line(grayImage3C, pt3, pt4, Scalar(0,255,255), 2);
			line(grayImage3C, pt4, pt1, Scalar(0,255,255), 2);
		}
	}	

	return grayImage3C;
}



// main function
int main(int argc, char **argv) {  
  
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for sending images")
			("inscope,i", po::value < std::string > (&g_sInScope),"Scope for receiving commands")
			("device,d", po::value < int > (&g_iDevice),"Number of device")
			("quality,q", po::value < unsigned int > (&g_uiQuality),"Quality of JPEG compression [0 .. 100]")
			("sending,s", "Sends the taken snapshot over RSB.")
			("debug", "Activates debugging which includes generated pictures and additional console information.")
			("printPic", "Prints a notice if a new picture has been taken.");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(),vm);

	// first, process the help option
	if (vm.count("help")) {
		std::cout << options << "\n";
		exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	sendingPic = vm.count("sending");
	debugging = vm.count("debug");

	INFO_MSG("Output scope: " << g_sOutScope);
	INFO_MSG("Command scope: " << g_sInScope);
	if (sendingPic) {
		INFO_MSG("Picture scope: " << g_sImageScope);
	}
	INFO_MSG("Device: " << g_iDevice);
	INFO_MSG("JPEG Quality: " << g_uiQuality);
	INFO_MSG("");

	////////////////////////////////////////////////////////////////////////////////////////////////////
	rsb::Factory &factory = rsb::Factory::getInstance();

	// Create the informer
	Informer<std::string>::Ptr imageInformer = getFactory().createInformer<std::string> (Scope(g_sImageScope));
	Informer<std::string>::Ptr detectedInformer = getFactory().createInformer<std::string> (Scope(g_sOutScope));

	// Create and start the command listener
	rsb::ListenerPtr listener = factory.createListener(g_sInScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

	listener->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(commandQueue)));
	////////////////////////////////////////////////////////////////////////////////////////////////////

	// Creating the cam object
	cv::VideoCapture cam;
	// Open the device /dev/video<g_iDevice>
	if ( cam.open(g_iDevice) ) {
		INFO_MSG("Camera Device activated:");

		Mat frame, grayImage, binImage, sendImage;

		cam >> frame;
		Size fSize = frame.size();
		INFO_MSG(" - Image Size: " << fSize.width << "/" << fSize.height << " pixels");

		/* Sending Image Parts:
		 * +--------+--------+
		 * | Part 1 | Part 2 |
		 * +--------+--------+
		 * | Part 3 | Part 4 |
		 * +--------+--------+
		 */
		sendImage = Mat(fSize.height, fSize.width*2, CV_8UC3);
		Mat part1(sendImage, Rect(0, 0, fSize.width, fSize.height));
		Mat part2(sendImage, Rect(fSize.width, 0, fSize.width, fSize.height));


		// Initialize Blob Detector (with parameters)
		SimpleBlobDetector::Params params;
		params.minThreshold = 50;
		params.maxThreshold = 200;
		params.filterByArea = true;
		params.minArea = 15;
		params.maxArea = frame.rows*frame.cols;
		params.filterByCircularity = true;
		params.minCircularity = M_PI/6.0;
		params.filterByConvexity = false;
		params.minConvexity = 0.87;
		params.filterByInertia = false;
		params.minInertiaRatio = 0.01;
		SimpleBlobDetector detector(params);

		INFO_MSG("");
		INFO_MSG("Starting camera loop.");
		INFO_MSG("");
		// Process the cam forever
		for (; ;) {

			// Get image
			cam >> frame;
			frame.copyTo(part1);

			// Get command
			if (!commandQueue->empty()) {
				std::string command = *commandQueue->pop().get();
				// check for quit command
				if (command == COMMAND_QUIT) {
					INFO_MSG("Quit application.");
					break;
				// check for find command
				} else if (command == COMMAND_FIND) {
					INFO_MSG("Find loading station");
					Mat blobbed = doBlobDetection(frame, detector);
					blobbed.copyTo(part2);
				} else if (command.substr(0, COMMAND_THRESHOLD.length()) == COMMAND_THRESHOLD) {
					std::string newThresholdS = command.substr(COMMAND_THRESHOLD.length());
					INFO_MSG("New threshold: " << newThresholdS);
					int thresholdCheck = atoi(newThresholdS.c_str());
					if (thresholdCheck > 0 && thresholdCheck < 255) {
						binaryThreshold = thresholdCheck;
					} else {
						WARNING_MSG(" => Threshold is not in ]0,255[!");
					}
					
				// otherwise it is an unknown command
				} else {
					INFO_MSG("Unknown command.");
				}
			} else {
				usleep(100000);
			}

			if (sendingPic) {
				// Compress image
				vector<uchar> buf;
				vector<int> compression_params;
				compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
				compression_params.push_back(g_uiQuality);

				imencode(".jpg", sendImage, buf, compression_params);

				// Send the data.
				shared_ptr<std::string> frameJpg(new std::string(buf.begin(), buf.end()));
				imageInformer->publish(frameJpg);
			}
	      
		}
	}

	// Free the cam
	cam.release();

	return 0;
}
