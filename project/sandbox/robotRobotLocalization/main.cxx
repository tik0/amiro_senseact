//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : -
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>
#include <rsb/util/EventQueuePushHandler.h>
#include <rsb/MetaData.h>
#include <rsb/util/QueuePushHandler.h>

// For program options
#include <boost/program_options.hpp>


#define INFO_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <stdio.h>
#include <OpenNI.h>
#include <Eigen/Dense>
#include <math.h>
#include <ctime>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>

#include <extspread/extspread.hpp>

using namespace std;
using namespace boost::chrono;
using namespace cv;
using namespace boost;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;


// scope names
std::string commandScope = "/amiro/command";
std::string CommandInscope = "/amiro/drive/status";
std::string imageInscope = "/image";
std::string imageOutscope = "/detected";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4823";
static int g_iDevice = 6;
static unsigned int g_uiQuality = 100;
static float camAngleHeightDegree = 43.60; // degree
static float camAngleWidthDegree = 53.12; // degree
float camAngleHeight, camAngleWidth;
int imageWidth = 480;
int imageHeight = 640;

float markerWidth = 0.1; // m
float markerHeightMin = 0.02; // m
float markerHeightMax = 0.03; // m

int firstUselessPics = 20;

std::string commands[] = {	"color_bS",
				"color_gS",
				"color_rS"};
int commandsCount = 3;


std::vector<float> calculatePosition(std::vector<Point> marker) {
	float xSize = float(marker[2].x - marker[1].x);
	float ySize = float(marker[1].y - marker[0].y);

	float minWidth = ySize/markerHeightMin*markerWidth;
	float maxWidth = ySize/markerHeightMax*markerWidth;
	if (xSize > maxWidth) {
		xSize = maxWidth;
	}
	float ppa = camAngleHeight/imageHeight;
	float objAngle = xSize * ppa;
	DEBUG_MSG("=> xSize=" << xSize << "p, ySize=" << ySize << "p, app=" << (ppa*180.0/M_PI) << "°, objAngle=" << (objAngle*180.0/M_PI) << "°")
	float dist = markerWidth/2.0 * sin(M_PI/2.0-objAngle/2.0) / sin(objAngle/2.0);
	float angle = camAngleWidth-marker[3].x*ppa - camAngleWidth/2.0 + objAngle/2.0;
	std::vector<float> result(2,0);
	result[0] = dist;
	result[1] = angle;
	return result;
}


cv::Mat matDiff(cv::Mat mat1, cv::Mat mat2) {
	for (int x=0; x<mat1.cols; x++) {
		for (int y=0; y<mat1.rows; y++) {
			int result = int(mat1.at<uint8_t>(y,x)) - int(mat2.at<uint8_t>(y,x));
			if (result < 0) result = 0;
			mat1.at<uint8_t>(y,x) = uint8_t(result);
		}
	}
	return mat1;
}

cv::Mat getRSBImage(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > imageQueue) {
	if (!imageQueue->empty()) imageQueue->pop();
	while (imageQueue->empty()) {
		// sleep for 10 ms
		usleep(10000);
	}
	// Get the image as string
	std::string imageJpg = *imageQueue->pop().get();
	// Copy to a vector
	std::vector<unsigned char> data(imageJpg.begin(), imageJpg.end());
	// Decode the image
	cv::Mat image = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
	return image;
}

int main(int argc, char **argv) {

	namespace po = boost::program_options;
	std::string outScope = "/lidar";
	std::size_t intervalMs = 1000;

	unsigned int dilateSize = 5;
	unsigned int erodeSize = 9;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("imageScope,i", po::value < std::string > (&imageInscope),"Inscope of images.")
			("period,p", po::value < std::size_t > (&intervalMs),"Period for publishing laser data in ms")
			("printTime,t","Prints the time")
			("device,v", po::value < int > (&g_iDevice),"Camera device ID")
			("dilate", po::value < unsigned int > (&dilateSize),"Dilation size")
			("erode", po::value < unsigned int > (&erodeSize),"Erosion size")
			("compression,c", po::value < unsigned int > (&g_uiQuality),"Compression value [0,100]");

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

	if (dilateSize % 2 == 0) dilateSize++;
	if (erodeSize % 2 == 0) erodeSize++;

	// Initialize clock
	system_clock::time_point systime1, systime2;
	milliseconds mstime;

	if (vm.count("printTime")) systime1 = system_clock::now();
	if (vm.count("printTime")) DEBUG_MSG("Starting RSB Initialization");

	camAngleHeight = camAngleHeightDegree * M_PI/180.0;
	camAngleWidth = camAngleWidthDegree * M_PI/180.0;

	// +++++ RSB Initialization +++++
	rsb::Factory &factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// +++ Listener +++

	// Create and start the listener for pictures
	rsb::ListenerPtr imageListener = factory.createListener(imageInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > imageQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
	imageListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler< std::string >(imageQueue)));

	rsb::ListenerPtr commandListener = factory.createListener(CommandInscope, extspreadconfig);
	boost::shared_ptr< rsc::threading::SynchronizedQueue< EventPtr > > commandQueue(new rsc::threading::SynchronizedQueue< EventPtr > (1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(commandQueue)));

	// +++ Informer +++

	// create informer for images
	rsb::Informer<std::string>::Ptr RGBInformer = factory.createInformer<std::string> (imageOutscope, extspreadconfig);

	// create informer for amiro commands
	rsb::Informer<std::string>::Ptr commandInformer = factory.createInformer<std::string> (commandScope, extspreadconfig);


	cv::VideoCapture cam;
  	if (!cam.open(g_iDevice)) {
		ERROR_MSG("Could not open device " << g_iDevice);
		return EXIT_FAILURE;
	}

	Mat rgbMat;
	cam >> rgbMat;
//	Mat rgbMat = getRSBImage(imageQueue);
	int height = rgbMat.rows;
	int width = rgbMat.cols;
	imageHeight = height;
	imageWidth = width;
	DEBUG_MSG("Image size: " << width << "/" << height);
	// Compress data
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(g_uiQuality);
	vector<uchar> bufRgb;

	// Initialize Blob Detector for rects
	SimpleBlobDetector::Params paramsRect;
	paramsRect.minThreshold = 50;
	paramsRect.maxThreshold = 200;
	paramsRect.filterByArea = true;
	paramsRect.minArea = 15;
	paramsRect.filterByCircularity = true;
//	paramsRect.minCircularity = M_PI/6.0;
	paramsRect.minCircularity = 0.0;
	paramsRect.filterByConvexity = false;
	paramsRect.minConvexity = 0.0;
	paramsRect.filterByInertia = false;
	paramsRect.minInertiaRatio = 0.01;
	SimpleBlobDetector detectorRect(paramsRect);

	Mat maxValue(rgbMat.rows, rgbMat.cols, CV_8UC1, Scalar(255));
	Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(dilateSize, dilateSize), Point(1,1));
	Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(erodeSize, erodeSize), Point(1,1));

	if (vm.count("printTime")) systime2 = system_clock::now();
	if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
	if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

	for (int useless=0; useless<firstUselessPics; useless++) {
		DEBUG_MSG("Waiting for " << (firstUselessPics-useless) << " images.");
//		getRSBImage(imageQueue);
		cam >> rgbMat;
	}

	boost::shared_ptr<std::string> colorCommand(new std::string(commands[0]));
	commandInformer->publish(colorCommand);
	while (commandQueue->empty()) {
		// sleep 10 ms
		usleep(10000);
	}
	commandQueue->pop(0);

	for(;;) {

		if (vm.count("printTime")) DEBUG_MSG("-----------------------------------------------")
		systime1 = system_clock::now();
		if (vm.count("printTime")) DEBUG_MSG("Calculate color dominations");

		// Create result image
		Mat resultImage = Mat(height, 5*width, CV_8UC3);
		Mat part[5];
		part[0] = Mat(resultImage, Rect(0, 0, width, height));
		part[1] = Mat(resultImage, Rect(width, 0, width, height));
		part[2] = Mat(resultImage, Rect(2*width, 0, width, height));
		part[3] = Mat(resultImage, Rect(3*width, 0, width, height));
		part[4] = Mat(resultImage, Rect(4*width, 0, width, height));

		Mat channels[3], conclusion, allChans;
		Mat matDetect(height, width, CV_32FC1, Scalar(0.0));
		double minVal, maxVal;
		float alpha, beta;
		while (!commandQueue->empty()) {
			commandQueue->pop(0);
		}
		for (int actColor=0; actColor<3; actColor++) {
			// Get current frame
			cam >> rgbMat;
//			rgbMat = getRSBImage(imageQueue);

			// Send color command
			int nextColor = actColor+1;
			if (nextColor > 2) nextColor = 0;
			boost::shared_ptr<std::string> colorCommand(new std::string(commands[nextColor]));
			commandInformer->publish(colorCommand);
			while (commandQueue->empty()) {
				// sleep 10 ms
				usleep(10000);
			}
			commandQueue->pop(0);

			// Split into bgr channels
			Mat chans[3], curChan, curChan3;
			split(rgbMat, chans);
			chans[actColor].copyTo(curChan);

			// Get channel ids of other channels
			int oc1, oc2;
			switch (actColor) {
				case 0: oc1 = 1; oc2 = 2; break;
				case 1: oc1 = 0; oc2 = 2; break;
				default: oc1 = 1; oc2 = 0; break;
			}
			// Calculate color domination
			curChan -= max(chans[oc1], chans[oc2]);
			threshold(curChan, curChan, 1, 255, 0);
//			curChan = matDiff(curChan, max(chans[oc1], chans[oc2]));

			// Rescale channel
//			minMaxLoc(curChan, &minVal, &maxVal);
//			alpha = 255.0 / (maxVal - minVal);
//			beta = 255.0 - maxVal*alpha;
//			curChan.convertTo(curChan, CV_8UC1, alpha, beta);

//			cvtColor(curChan, curChan3, CV_GRAY2BGR);
			rgbMat.copyTo(part[actColor+2]);

			// Convert to float
			curChan.convertTo(channels[actColor], CV_32FC1, 1.0, 0);
			matDetect += channels[actColor];
		}
		rgbMat.copyTo(part[0]);
		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

		// Fuse color dominations
		if (vm.count("printTime")) systime1 = system_clock::now();
		if (vm.count("printTime")) DEBUG_MSG("Domination fusion");

		// Rescale
		minMaxLoc(matDetect, &minVal, &maxVal);
		threshold(matDetect, matDetect, maxVal*2.0/3.0, maxVal, minVal);
		alpha = 255.0 / (maxVal - minVal);
		beta = 255.0 - maxVal*alpha;
		matDetect.convertTo(conclusion, CV_8UC1, alpha, beta);

		// Invert image
		conclusion = maxValue - conclusion;

		// Prepare blobs
		dilate(conclusion, conclusion, dilateElement);
		erode(conclusion, conclusion, erodeElement);

		// Convert for publishing
		cvtColor(conclusion, allChans, CV_GRAY2BGR);
		allChans.copyTo(part[1]);
		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");


		// TODO Find contours!

		// Get blobs
		if (vm.count("printTime")) systime1 = system_clock::now();
		if (vm.count("printTime")) DEBUG_MSG("Do blobdetection");
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		Mat cInv = maxValue - conclusion;
		cv::findContours(cInv, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
		std::vector< std::vector<Point> > contourEdges;
		int biggestContour = -1;
		int biggestContourSize = 0;
		for (int c=0; c<contours.size(); c++) {
			cv::drawContours(part[0], contours, c, cv::Scalar(0, 255, 255));
			cv::drawContours(part[1], contours, c, cv::Scalar(0, 255, 255));
			int xMin = cInv.cols;
			int yMin = cInv.rows;
			int xMax = -1;
			int yMax = -1;
			for (int p=0; p<contours[c].size(); p++) {
				Point point = contours[c][p];
				if (point.x < xMin) xMin = point.x;
				if (point.y < yMin) yMin = point.y;
				if (point.x > xMax) xMax = point.x;
				if (point.y > yMax) yMax = point.y;
			}
			if ((xMax-xMin)*(yMax-yMin) > biggestContourSize) {
				biggestContourSize = (xMax-xMin)*(yMax-yMin);
				biggestContour = c;
			}
			Point dl(xMin, yMin);
			Point ul(xMin, yMax);
			Point ur(xMax, yMax);
			Point dr(xMax, yMin);
			std::vector<Point> points;
			points.push_back(dl);
			points.push_back(ul);
			points.push_back(ur);
			points.push_back(dr);
			contourEdges.push_back(points);
		}
		if (biggestContour >= 0) {
			std::vector<float> objPos = calculatePosition(contourEdges[biggestContour]);
			int xPos = imageWidth-(int(objPos[1]*imageWidth/camAngleWidth)+imageWidth/2)-1;
			DEBUG_MSG("=> " << imageWidth << " - (int(" << (objPos[1]*180.0/M_PI) << " * " << imageWidth << "/" << (camAngleWidth*180.0/M_PI) << ") + " << imageWidth << "/2) - 1");
			int yPos = imageHeight/2;
			DEBUG_MSG("=> AMiRo at " << objPos[0] << "m and " << (objPos[1]*180.0/M_PI) << "° (" << xPos << "/" << yPos << ")");
			Point objPoint(xPos, yPos);
			cv::circle(part[0], objPoint, 5, cv::Scalar(0, 255, 0));
			cv::circle(part[1], objPoint, 5, cv::Scalar(0, 255, 0));

			for (int c=0; c<contourEdges.size(); c++) {
				cv::Scalar color;
				if (c==biggestContour) {
					color = cv::Scalar(0, 0, 255);
				} else {
					color = cv::Scalar(255, 0, 0);
				}
				for (int p=0; p<contourEdges[c].size(); p++) {
					Point curP = contourEdges[c][p];
					Point nextP;
					if (p==0) {
						nextP = contourEdges[c][contourEdges[c].size()-1];
					} else {
						nextP = contourEdges[c][p-1];
					}
					cv::line(part[0], curP, nextP, color);
					cv::line(part[1], curP, nextP, color);
				}
			}
		}

//		std::vector<KeyPoint> keypoints;
//		detectorRect.detect(conclusion, keypoints);
//		drawKeypoints(part[0], keypoints, part[0], Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//		drawKeypoints(part[1], keypoints, part[1], Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

		// Compress and send image
		if (vm.count("printTime")) systime1 = system_clock::now();
		if (vm.count("printTime")) DEBUG_MSG("Compressing and sending image");
		imencode(".jpg", resultImage, bufRgb, compression_params);
		shared_ptr< std::string > rgbFrameJpg(new std::string(bufRgb.begin(), bufRgb.end()));
		RGBInformer->publish(rgbFrameJpg);
		systime2 = system_clock::now();
		mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms")
		else DEBUG_MSG("Image sent (" << mstime.count() << " ms)");

		usleep(100000);
	}

	return EXIT_SUCCESS;

}
