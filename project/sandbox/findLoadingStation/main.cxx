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
// Include own converter
#include <converter/vecIntConverter/main.hpp>

// For checking character pressed in the console
#include <kbhit.hpp>

// protocol defines
std::string COMMAND_QUIT = "ESC";
std::string COMMAND_FIND = "FIND";
std::string COMMAND_LOAD = "LOAD";
std::string COMMAND_THRESHOLD = "threshold_";


using namespace boost;
using namespace std;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;
using namespace cv;

typedef struct StationData_t {
	int imageWidth;
	int imageHeight;
	float camAngleWidth;
	float camAngleHeight;
	int stationWidth;
	int stationHeight;
	int stationCenterWidth;
	int stationCenterHeight;
};

std::vector<StationData_t> stations;

// blob characteristic constants
#define STATION_CONTACT_SIZE_MIN 40.0
#define STATION_CONTACT_SIZE_MAX 85.0
#define STATION_MARKER_DIFF_MAX 0.15
#define STATION_SIZE_DIFF_MAX 5.0
#define STATION_SIZE_DIFF_MIN 1.0


#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

#include <jpeglib.h>

static std::string g_sImageScope = "/stationDetection/image";
static std::string g_sOutScope = "/stationDetection/detected";
static std::string g_sInScope = "/stationDetection/command";
static int g_iDevice = 0;
static unsigned int g_uiQuality = 85;

static float camAngleHeightDegree = 43.60; // degree
static float camAngleWidthDegree = 53.12; // degree 
float camAngleHeight, camAngleWidth;

static float stationHeight = 0.0405; // m

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


Mat doBlobDetection(Mat frame, SimpleBlobDetector detectorRect, SimpleBlobDetector detectorCirc) {
	Mat grayImage, grayImage3C, redImage3C, resultFrame;
	Size fSize = frame.size();

	stations.clear();

//	Mat sendImage = Mat(fSize.height, fSize.width*2, CV_8UC3);
//	Mat part1(sendImage, Rect(0, 0, fSize.width, fSize.height));
//	Mat part2(sendImage, Rect(fSize.width, 0, fSize.width, fSize.height));

	frame.copyTo(resultFrame);
	Mat bigFocusFrame;
	frame.copyTo(bigFocusFrame);

	// Convert image
	cvtColor(frame, grayImage, CV_BGR2GRAY);
//	blur(grayImage, grayImage, Size(3,3), Point(-1,-1));

//	threshold(chans[2], binImage, binaryThreshold, 255, 0);
//	INFO_MSG(" -> Binary image created: " << binImage.size().width << "/" << binImage.size().height);

	cvtColor(grayImage, grayImage3C, CV_GRAY2BGR);

	std::vector<KeyPoint> keypoints;
	detectorRect.detect(grayImage3C, keypoints);
	if (debugging) drawKeypoints(resultFrame, keypoints, resultFrame, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	if (debugging) DEBUG_MSG(" -> Blob Detection: " << keypoints.size() << " blobs detected.");

	if (keypoints.size() <= 1) {
		return resultFrame;
	}

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

				if (sizeDiff/maxSize < STATION_MARKER_DIFF_MAX && compDistDiff < actDistDiff) {
					bestPartner[curPoint] = comp;
				}
			}
		}
	}

	Mat focusFrame;
	std::vector<int> partner1;
	std::vector<int> partner2;
	// Mark partners to pairs, which fit with each other
	for (int curPoint=0; curPoint<keypoints.size(); curPoint++) {
		if (bestPartner[bestPartner[curPoint]] == curPoint && bestPartner[curPoint] > curPoint) {
			Point pt1 = keypoints[curPoint].pt;
			float size1 = keypoints[curPoint].size;
			Point pt2 = keypoints[bestPartner[curPoint]].pt;
			float size2 = keypoints[bestPartner[curPoint]].size;
			if (debugging) DEBUG_MSG(" -> Partners " << curPoint << "-" << bestPartner[curPoint] << " (" << pt1.x << "/" << pt1.y << " - " << pt2.x << "/" << pt2.y << ", " << size1 << " - " << size2 << ")");
			if (debugging) line(resultFrame, pt1, pt2, Scalar(0,255,0), 2);

			partner1.push_back(curPoint);
			partner2.push_back(bestPartner[curPoint]);

			int i = partner1.size()-1;

			// Check the focus image
			int xMinRect, xMaxRect;
			if (keypoints[partner1[i]].pt.x < keypoints[partner2[i]].pt.x) {
				xMinRect = keypoints[partner1[i]].pt.x + keypoints[partner1[i]].size/2;
				xMaxRect = keypoints[partner2[i]].pt.x - keypoints[partner2[i]].size/2;
			} else {
				xMinRect = keypoints[partner2[i]].pt.x - keypoints[partner2[i]].size/2;
				xMaxRect = keypoints[partner1[i]].pt.x + keypoints[partner1[i]].size/2;
			}
			int yMinRect = min(keypoints[partner1[i]].pt.y-keypoints[partner1[i]].size, keypoints[partner2[i]].pt.y-keypoints[partner2[i]].size);
			int yMaxRect = max(keypoints[partner1[i]].pt.y, keypoints[partner2[i]].pt.y);
			if (xMinRect < 0) xMinRect = 0;
			if (yMinRect < 0) yMinRect = 0;
			if (xMaxRect >= fSize.width) xMaxRect = fSize.width-1;
			if (yMaxRect >= fSize.height) yMaxRect = fSize.height-1;

			if (xMaxRect-xMinRect > 0 && yMaxRect-yMinRect > 0) {
				if (debugging) DEBUG_MSG("    -> Rect is big enough.");
				float rectSizeDiff = (float)(xMaxRect-xMinRect)/(float)(yMaxRect-yMinRect);
				if (debugging) DEBUG_MSG("    -> Rect has size difference of " << rectSizeDiff << ".");
				if (rectSizeDiff >= STATION_SIZE_DIFF_MIN && rectSizeDiff <= STATION_SIZE_DIFF_MAX) {
					if (debugging) DEBUG_MSG("    -> Rect size difference correct.");
					Rect zoomRect(xMinRect, yMinRect, xMaxRect-xMinRect, yMaxRect-yMinRect);
					focusFrame = bigFocusFrame(zoomRect);
					resize(focusFrame, focusFrame, Size(fSize.width, fSize.height));

					// Get image channels
	//				Mat chans[3];
	//				split(focusFrame, chans);

					Mat focusFrameGray;
					cvtColor(focusFrame, focusFrameGray, CV_BGR2GRAY);

					// Get red dominated parts
	//				Mat greenblue = cv::max(chans[0], chans[1]);
	//				Mat redChan = chans[2] - greenblue;
	//				Mat maxValue(frame.rows, frame.cols, CV_8UC1, Scalar(255));
	//				redChan = maxValue - redChan;
	//				threshold(redChan, redChan, 254, 255, 0);
	//				Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(11,11), Point(1,1));
	//				dilate(redChan, redChan, dilateElement);
				//	Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(5,5), Point(1,1));
				//	erode(redChan, redChan, erodeElement);
				//	Mat dilateElement2 = getStructuringElement(MORPH_ELLIPSE, Size(3,3), Point(1,1));
				//	dilate(redChan, redChan, dilateElement2);
				//	normalize(redChan, redChan, 0, 255, NORM_MINMAX, CV_8UC1);
					cvtColor(focusFrameGray, redImage3C, CV_GRAY2BGR);

					std::vector<KeyPoint> keypointsRed;
					detectorCirc.detect(redImage3C, keypointsRed);
					drawKeypoints(redImage3C, keypointsRed, redImage3C, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

					int bigPoints = 0;
					for (int c=0; c<keypointsRed.size(); c++) {
						if (debugging) DEBUG_MSG("      -> keypoint " << c << ": " << keypointsRed[c].size);
						if (keypointsRed[c].size > STATION_CONTACT_SIZE_MIN && keypointsRed[c].size < STATION_CONTACT_SIZE_MAX) {
							bigPoints++;
						}
					}

					if (debugging) DEBUG_MSG("    -> " << bigPoints << " big points found.");

					if (bigPoints >= 2) {
						int xMin = min(keypoints[partner1[i]].pt.x-keypoints[partner1[i]].size, keypoints[partner2[i]].pt.x-keypoints[partner2[i]].size);
						int xMax = max(keypoints[partner1[i]].pt.x+keypoints[partner1[i]].size, keypoints[partner2[i]].pt.x+keypoints[partner2[i]].size);
						int yMin = min(keypoints[partner1[i]].pt.y-keypoints[partner1[i]].size, keypoints[partner2[i]].pt.y-keypoints[partner2[i]].size);
						int yMax = max(keypoints[partner1[i]].pt.y+keypoints[partner1[i]].size, keypoints[partner2[i]].pt.y+keypoints[partner2[i]].size);

						if (xMin < 0) xMin = 0;
						if (yMin < 0) yMin = 0;
						if (xMax >= fSize.width) xMax = fSize.width-1;
						if (yMax >= fSize.height) yMax = fSize.height-1;
						Point ptA(xMin, yMin);
						Point ptB(xMin, yMax);
						Point ptC(xMax, yMax);
						Point ptD(xMax, yMin);
						line(resultFrame, ptA, ptB, Scalar(0,255,255), 1);
						line(resultFrame, ptB, ptC, Scalar(0,255,255), 1);
						line(resultFrame, ptC, ptD, Scalar(0,255,255), 1);
						line(resultFrame, ptD, ptA, Scalar(0,255,255), 1);

						StationData_t newStation;
						newStation.imageWidth = fSize.width;
						newStation.imageHeight = fSize.height;
						newStation.camAngleWidth = camAngleWidth;
						newStation.camAngleHeight = camAngleHeight;
						newStation.stationWidth = xMax-xMin;
						newStation.stationHeight = yMax-yMin;
						newStation.stationCenterWidth = xMin + newStation.stationWidth/2;
						newStation.stationCenterHeight = newStation.imageHeight - (yMin + newStation.stationHeight/2);
						stations.push_back(newStation);

						//redImage3C.copyTo(part2);
					}
				}
			}
		}
	}

//	resultFrame.copyTo(part1);
	return resultFrame;
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
			("cameraAngleHeight", po::value < float > (&camAngleHeightDegree),"Horizontal angle of the camera in degrees (default: 43.60°).")
			("camerAngleWidth", po::value < float > (&camAngleWidthDegree),"Vertical angle of the camera in degrees (default: 53.12°).")
			("stationHeight", po::value < float > (&stationHeight),"Height of the station (measured at black stripes) in meters (default: 0.0405).")
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

	camAngleHeight = camAngleHeightDegree*M_PI/180.0;
	camAngleWidth = camAngleWidthDegree*M_PI/180.0;

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

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
	converterRepository<std::string>()->registerConverter(converter);

	// Create the informer
	Informer<std::string>::Ptr imageInformer = getFactory().createInformer<std::string> (Scope(g_sImageScope));
	rsb::Informer< std::vector<int> >::Ptr stationPosInformer = factory.createInformer< std::vector<int> > (g_sOutScope);

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

		sendImage = Mat(fSize.height, fSize.width*2, CV_8UC3);
		Mat part1(sendImage, Rect(0, 0, fSize.width, fSize.height));
		Mat part2(sendImage, Rect(fSize.width, 0, fSize.width, fSize.height));


		// Initialize Blob Detector for rects
		SimpleBlobDetector::Params paramsRect;
		paramsRect.minThreshold = 50;
		paramsRect.maxThreshold = 200;
		paramsRect.filterByArea = true;
		paramsRect.minArea = 15;
		paramsRect.maxArea = frame.rows*frame.cols;
		paramsRect.filterByCircularity = true;
		paramsRect.minCircularity = M_PI/6.0;
		paramsRect.filterByConvexity = false;
		paramsRect.minConvexity = 0.87;
		paramsRect.filterByInertia = false;
		paramsRect.minInertiaRatio = 0.01;
		SimpleBlobDetector detectorRect(paramsRect);

		// Initialize Blob Detector for circles
		SimpleBlobDetector::Params paramsCirc;
		paramsCirc.minThreshold = 50;
		paramsCirc.maxThreshold = 200;
		paramsCirc.filterByArea = true;
		paramsCirc.minArea = 15;
		paramsCirc.maxArea = frame.rows*frame.cols;
		paramsCirc.filterByCircularity = true;
		paramsCirc.minCircularity = 0.01;
		paramsCirc.filterByConvexity = false;
		paramsCirc.minConvexity = 0.87;
		paramsCirc.filterByInertia = false;
		paramsCirc.minInertiaRatio = 0.01;
		SimpleBlobDetector detectorCirc(paramsCirc);


		// other variables
		std::vector<int> stationPos(2,0);
		boost::shared_ptr< std::vector<int> > vecStationPos;

		INFO_MSG("");
		INFO_MSG("Starting camera loop.");
		INFO_MSG("");
		// Process the cam forever
		for (; ;) {

			// Get image
			cam >> frame;
			Mat resultFrame;
			frame.copyTo(resultFrame);
			Mat bottom(resultFrame, Rect(0, 0, fSize.width, fSize.height*2/3));
//			Mat top(resultFrame, Rect(fSize.width, 0, fSize.width, fSize.height-bottom.size().height));
			Mat blobbed = doBlobDetection(bottom, detectorRect, detectorCirc);
			if (sendingPic) {
//				blobbed.copyTo(sendImage);
				blobbed.copyTo(bottom);
				if (debugging) {
					frame.copyTo(part1);
					resultFrame.copyTo(part2);
				} else {
					resultFrame.copyTo(sendImage);
				}
			}

			INFO_MSG("Station count: " << stations.size());
			for (int i=0; i<stations.size(); i++) {
				StationData_t station = stations[i];
				float angleImage = station.camAngleWidth;
				int widthImage = station.imageWidth;
				float angle = (station.stationCenterWidth - widthImage/2.0) * angleImage/widthImage * (-180.0/M_PI);
				float heightAngleHalfed = station.camAngleHeight * station.stationHeight/station.imageHeight * 0.5;
				float stationDist = stationHeight/2.0 * sin(M_PI*0.5 - heightAngleHalfed) / sin(heightAngleHalfed) * 1.33;
				INFO_MSG(" -> " << station.stationCenterWidth << "/" << station.stationCenterHeight << " => " << angle << "°, " << stationDist << " m");
			}

			if (stations.size() > 0) {
				float angle = (stations[0].stationCenterWidth - stations[0].imageWidth/2.0) * stations[0].camAngleWidth/stations[0].imageWidth * (-1);
				float heightAngleHalfed = stations[0].camAngleHeight * stations[0].stationHeight/stations[0].imageHeight * 0.5;
				float dist = stationHeight/2.0 * sin(M_PI*0.5 - heightAngleHalfed) / sin(heightAngleHalfed) * 1.33;
				stationPos[0] = (int)(dist*1000000.0);
				stationPos[1] = (int)(angle*1000000.0);
			} else {
				stationPos[0] = 0;
				stationPos[1] = 0;
			}
			vecStationPos = boost::shared_ptr<std::vector<int> >(new std::vector<int>(stationPos.begin(),stationPos.end()));
			stationPosInformer->publish(vecStationPos);

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
