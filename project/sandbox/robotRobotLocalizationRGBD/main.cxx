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
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#include <extspread/extspread.hpp>

#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

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
std::string trackingOutscope = "/amiro/tracking";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4823";
static int g_iDevice = 6;
static unsigned int g_uiQuality = 100;
bool continuess = false;
static float camAngleHeightDegree = 43.60; // degree
static float camAngleWidthDegree = 53.12; // degree
float camAngleHeight, camAngleWidth;
int imageWidth = 480;
int imageHeight = 640;
int scanWidth = 480;
int scanHeight = 640;

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
	
	float xCenter = xSize/2.0 + float(marker[1].x);
	float yCenter = ySize/2.0 + float(marker[0].y);
	
	float ppa = camAngleHeight/imageHeight;
	float objAngle = xSize * ppa;
	float dist = markerWidth/2.0 * sin(M_PI/2.0-objAngle/2.0) / sin(objAngle/2.0);
	float angle = xCenter*ppa - camAngleWidth/2.0;
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

void waitForColorSet(std::string colorCommand, boost::shared_ptr< rsc::threading::SynchronizedQueue< EventPtr > > commandQueue, rsb::Informer<std::string>::Ptr commandInformer) {
	// clear command queue
	while (!commandQueue->empty()) {
		commandQueue->pop();
	}
	// wait for receiving color command answer
	do {
		// send light command
		boost::shared_ptr<std::string> command(new std::string(colorCommand));
		commandInformer->publish(command);
				
		// sleep 100 ms
		usleep(100000);
	} while (commandQueue->empty());
	commandQueue->pop();
	usleep(100000);
}

void loadRgbMatFromRgbStream(Mat &xMat, VideoStream &rgbStream, VideoFrameRef &rgbImage) {
	// read image
	Status respON = rgbStream.readFrame(&rgbImage);
	if (respON != STATUS_OK) {
		ERROR_MSG("Read RGB image failed: " << OpenNI::getExtendedError());
		xMat.create(0, 0, CV_8UC1);
	}
	
	// convert to OpenCV
	const openni::RGB888Pixel* rgbBuffer = (const openni::RGB888Pixel*)rgbImage.getData();
	memcpy(xMat.data, rgbBuffer, 3*rgbImage.getHeight()*rgbImage.getWidth()*sizeof(uint8_t));
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
			("compression,c", po::value < unsigned int > (&g_uiQuality),"Compression value [0,100]")
			("continuess,s", "Flag, if every new image shall be used for calculation continuess.")
			("dontSend", "Flag, ...");

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

	bool ead = dilateSize > 0 && erodeSize > 0;
	if (dilateSize < 3) {
		dilateSize = 3;
	} else if (dilateSize % 2 == 0) {
		dilateSize++;
	}
	if (erodeSize < 3) {
		erodeSize = 3;
	} else if (erodeSize % 2 == 0) {
		erodeSize++;
	}
	
	continuess = vm.count("continuess");

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

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

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
	rsb::Informer<std::string>::Ptr RGBInformer = factory.createInformer<std::string> (imageOutscope);

	// create informer for amiro commands
	rsb::Informer<std::string>::Ptr commandInformer = factory.createInformer<std::string> (commandScope, extspreadconfig);
	
	// create informer for tracking information
	rsb::Informer< std::vector<int> >::Ptr trackingInformer = factory.createInformer< std::vector<int> > (trackingOutscope, extspreadconfig);


	VideoFrameRef rgbImage, depthImage;
	VideoStream rgbStream, depthStream;
	Device device;
	
	Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(dilateSize, dilateSize), Point(1,1));
	Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(erodeSize, erodeSize), Point(1,1));

	// Compress data
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(g_uiQuality);
	vector<uchar> bufRgb;

	Status respON = OpenNI::initialize();
	if (respON != STATUS_OK) {
		ERROR_MSG("Initialize failed: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	
	DEBUG_MSG("Open device");
	// open device and start stream
	respON = device.open(ANY_DEVICE);
	if (respON != STATUS_OK) {
		ERROR_MSG("Could not open device: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}

	// get and start rgb stream
	const SensorInfo* rgbinfo = device.getSensorInfo(SENSOR_COLOR);
	if (rgbinfo != NULL) {
		respON = rgbStream.create(device, SENSOR_COLOR);
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not create RGB stream: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
		const openni::Array<VideoMode>& modes = rgbinfo->getSupportedVideoModes();
		INFO_MSG("Modes of rgb stream:");
		for (int i=0; i<modes.getSize(); i++) {
			INFO_MSG(" " << i << ": " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << ", " << modes[i].getFps() << " fps, " << modes[i].getPixelFormat() << " format");
		}
		respON = rgbStream.setVideoMode(modes[0]);
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not set mode of RGB stream: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
	}
	respON = rgbStream.start();
	if (respON != STATUS_OK) {
		ERROR_MSG("Could not start the RGB stream: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	
	// get and start depth stream
	const SensorInfo* depthinfo = device.getSensorInfo(SENSOR_DEPTH);
	if (depthinfo != NULL) {
		respON = depthStream.create(device, SENSOR_DEPTH);
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not create depth stream: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
		const openni::Array<VideoMode>& modes = depthinfo->getSupportedVideoModes();
		INFO_MSG("Modes of depth stream:");
		for (int i=0; i<modes.getSize(); i++) {
			INFO_MSG(" " << i << ": " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << ", " << modes[i].getFps() << " fps, " << modes[i].getPixelFormat() << " format");
		}
		respON = depthStream.setVideoMode(modes[0]);
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not set mode of depth stream: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
	}
	respON = depthStream.start();
	if (respON != STATUS_OK) {
		ERROR_MSG("Could not start the depth stream: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}

	VideoStream* rgbStreamTemp = &rgbStream;
	int changedRgbStreamDummy;
	respON = OpenNI::waitForAnyStream(&rgbStreamTemp, 1, &changedRgbStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
	if (respON != STATUS_OK) {
		ERROR_MSG("Wait for RGB stream failed (timeout is "  << SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	VideoStream* depthStreamTemp = &depthStream;
	int changedDepthStreamDummy;
	respON = OpenNI::waitForAnyStream(&depthStreamTemp, 1, &changedDepthStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
	if (respON != STATUS_OK) {
		ERROR_MSG("Wait for depth stream failed (timeout is "  << SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	
	// read first image
	respON = rgbStream.readFrame(&rgbImage);
	if (respON != STATUS_OK) {
		ERROR_MSG("Read RGB image failed: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	imageHeight = rgbImage.getHeight();
	imageWidth = rgbImage.getWidth();
	
	// read first scan
	respON = depthStream.readFrame(&depthImage);
	if (respON != STATUS_OK) {
		ERROR_MSG("Read depth image failed: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	scanHeight = depthImage.getHeight();
	scanWidth = depthImage.getWidth();
	INFO_MSG("Image size " << imageWidth << "x" << imageHeight << " with laser image " << scanWidth << "x" << scanHeight);
		
	// Create result image
	Mat resultImage = Mat(2*imageHeight, 3*imageWidth, CV_8UC3);
	Mat part[6];
	part[0] = Mat(resultImage, Rect(0, 0, imageWidth, imageHeight));
	part[1] = Mat(resultImage, Rect(imageWidth, 0, imageWidth, imageHeight));
	part[2] = Mat(resultImage, Rect(2*imageWidth, 0, imageWidth, imageHeight));
	part[3] = Mat(resultImage, Rect(0, imageHeight, imageWidth, imageHeight));
	part[4] = Mat(resultImage, Rect(1*imageWidth, imageHeight, imageWidth, imageHeight));
	part[5] = Mat(resultImage, Rect(2*imageWidth, imageHeight, imageWidth, imageHeight));

	double minVal, maxVal;
	float alpha, beta;

	// Initialize OpenCV RGB Images
	Mat rgbMats[3];
	for (int mat=0; mat<3; mat++) {
		rgbMats[mat].create(imageHeight, imageWidth, CV_8UC3);
	}
	Mat depthdata(scanHeight, scanWidth, CV_16UC1);
	
	// Initialize OpenCV Mats for localization
	Mat maxValue(imageHeight, imageWidth, CV_8UC1, Scalar(255));
	Mat matDetect(imageHeight, imageWidth, CV_32FC1, Scalar(0.0));

	// pop all already received messages
	while (!commandQueue->empty()) {
		commandQueue->pop(0);
	}
	
	// start localization loop
	DEBUG_MSG("Start localizing");
	int colorCounter = -1; // if negative: first loop run
	for(;;) {
		systime1 = system_clock::now();
		if (vm.count("printTime")) DEBUG_MSG("Get robot position:");
		
		DEBUG_MSG(" - Get RGBD images:");
		if (!continuess || colorCounter < 0) {
			for (int actColor=0; actColor<3; actColor++) {
				waitForColorSet(commands[actColor], commandQueue, commandInformer);
				loadRgbMatFromRgbStream(rgbMats[2-actColor], rgbStream, rgbImage);
			}
			
		} else if (continuess) {
			waitForColorSet(commands[colorCounter], commandQueue, commandInformer);
			loadRgbMatFromRgbStream(rgbMats[2-colorCounter], rgbStream, rgbImage);
		}
		respON = depthStream.readFrame(&depthImage);
		if (respON != STATUS_OK) {
			ERROR_MSG("Read depth image failed: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
		const openni::DepthPixel* depthBuffer = (const openni::DepthPixel*)depthImage.getData();
		memcpy(depthdata.data, depthBuffer, depthImage.getHeight()*depthImage.getWidth()*sizeof(uint16_t));
		Mat depthDataImage1C;
		minMaxLoc(depthdata, &minVal, &maxVal);
		alpha = 255.0 / maxVal;
		beta = 255.0 - maxVal*alpha;
		depthdata.convertTo(depthDataImage1C, CV_8UC1, alpha, beta);
		cvtColor(depthDataImage1C, part[2], CV_GRAY2RGB);
		
		if (colorCounter < 0) {
			rgbMats[0].copyTo(part[0]);
		} else {
			rgbMats[colorCounter].copyTo(part[0]);
		}
		
		Mat channels[3], conclusion;
		matDetect *= 0.0;
		for (int actColor=0; actColor<3; actColor++) {
			// set channel into result image
			rgbMats[actColor].copyTo(part[actColor+3]);
			
			// Get channel ids of other channels
			int oc1, oc2;
			switch (actColor) {
				case 0: oc1 = 1; oc2 = 2; break;
				case 1: oc1 = 0; oc2 = 2; break;
				default: oc1 = 1; oc2 = 0; break;
			}
			
			// Split into bgr channels
			Mat actMat, chans[3], curChan, curChan3;
			rgbMats[actColor].copyTo(actMat);
			split(actMat, chans);
			for (int c=0; c<3; c++) {
				blur(chans[c], chans[c], cv::Size(3,3));
			}
			chans[actColor].copyTo(curChan);
			// Calculate color domination
			curChan -= max(chans[oc1], chans[oc2]);
			threshold(curChan, curChan, 1, 255, 0);

			// Convert to float
			curChan.convertTo(channels[actColor], CV_32FC1, 1.0, 0);
			matDetect += channels[actColor];
		}
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
		if (ead) {
			dilate(conclusion, conclusion, dilateElement);
			erode(conclusion, conclusion, erodeElement);
		}
		
		// Convert for publishing
		cvtColor(conclusion, part[1], CV_GRAY2BGR);
		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

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
			int xPos = int((objPos[1]+camAngleWidth/2.0)*float(imageWidth)/camAngleWidth);
			DEBUG_MSG("=> int(" << (objPos[1]*180.0/M_PI) << " * " << imageWidth << "/" << (camAngleWidth*180.0/M_PI) << ") + " << imageWidth << "/2");
			int yPos = imageHeight/2;
			int laserMin = 10000;
			int xPosMinLaser = -1;
			int yPosMinLaser = -1;
			for (int yl=contourEdges[biggestContour][0].y; yl<=contourEdges[biggestContour][2].y; yl++) {
				for (int xl=contourEdges[biggestContour][0].x; xl<=contourEdges[biggestContour][2].x; xl++) {
					int laserValue = depthdata.at<uint16_t>(yl, xl);
					if (laserValue > 0 && laserValue < laserMin) {
						laserMin = laserValue;
						xPosMinLaser = xl;
						yPosMinLaser = yl;
					}
				}
			}
			DEBUG_MSG("=> AMiRo at " << objPos[0] << "m (" << laserMin << "mm) and " << (objPos[1]*180.0/M_PI) << "° (" << xPos << "/" << yPos << ")");
			Point objPoint(xPos, yPos);
			cv::circle(part[0], objPoint, 5, cv::Scalar(0, 255, 0));
			cv::circle(part[1], objPoint, 5, cv::Scalar(0, 255, 0));
			if (laserMin < 10000) {
				// publish tracking point
				std::vector<int> trackingPoint(2,0);
				trackingPoint[0] = laserMin*1000; // um
				trackingPoint[1] = int(objPos[1]*1000000.0); //urad
				boost::shared_ptr< std::vector<int> > trackingPointCmd = boost::shared_ptr<std::vector<int> >(new std::vector<int>(trackingPoint.begin(),trackingPoint.end()));
				trackingInformer->publish(trackingPointCmd);
				
				Point minLaserPoint(xPosMinLaser, yPosMinLaser);
				cv::circle(part[0], minLaserPoint, 5, cv::Scalar(255, 0, 255));
				cv::circle(part[1], minLaserPoint, 5, cv::Scalar(255, 0, 255));
				cv::circle(part[2], minLaserPoint, 5, cv::Scalar(255, 0, 255));
			}

			for (int c=0; c<contourEdges.size(); c++) {
				cv::Scalar color;
				int lastImage = 2;
				if (c==biggestContour) {
					lastImage = 3;
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
					for (int i=0; i<lastImage; i++) {
						cv::line(part[i], curP, nextP, color);
					}
				}
			}
		}
		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

		// Compress and send image
		if (vm.count("printTime")) systime1 = system_clock::now();
		if (vm.count("printTime")) DEBUG_MSG("Compressing and sending image");
		imencode(".jpg", resultImage, bufRgb, compression_params);
		if (!vm.count("dontSend")) {
			shared_ptr< std::string > rgbFrameJpg(new std::string(bufRgb.begin(), bufRgb.end()));
			RGBInformer->publish(rgbFrameJpg);
		}
		systime2 = system_clock::now();
		mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms")
		else DEBUG_MSG("Image sent (" << mstime.count() << " ms)");

		// rise up color counter
		colorCounter++;
		if (colorCounter > 2) colorCounter = 0;

		usleep(100000);
	}

	// Close rgb stream and device
	DEBUG_MSG("Close device");
	rgbStream.stop();
	rgbStream.destroy();
	device.close();
	
	OpenNI::shutdown();

	return EXIT_SUCCESS;

}
