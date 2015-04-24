//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Central control of the robots map building and exploration.
//============================================================================

// std includes
#include <iostream>
using namespace std;

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// boost
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

// rsb
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/util/QueuePushHandler.h>
using namespace rsb;
using namespace rsb::patterns;

// converters
#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>
#include <ControllerAreaNetwork.h>

// camera parameter
float meterPerPixel = 1.0 / 400.0;
const float cellSize = 0.01;

// search the pose-list received from tracking for the robots id and get the corresponding pose.
cv::Point3f readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID) {
	twbTracking::proto::Pose2D pose;
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingMarkerID == data->pose(i).id()) {
			pose = data->pose(i);
			break;
		}
	}
	return cv::Point3f(pose.x() * meterPerPixel, pose.y() * meterPerPixel, pose.orientation() * M_PI / 180.0);
}

boost::shared_ptr<twbTracking::proto::Pose2DList> pathToStartPos, pushingPath;
int pathIdx = -1;
cv::Point2f destObjectPos(164, 52), nextDestObjectPos = destObjectPos, currObjectPos, startOwnPos, destOwnPos;
Mat help0, help1;
bool pathToStartPosSent = false, startPushing = true;
rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathInformer;
float objectRadius = 0;
int robotRadius = 5; // check!
rsb::Informer< std::vector<int> >::Ptr motorCmdInformer;
std::vector<int> motorCmd(3,0);
ControllerAreaNetwork myCAN;

void calcAndPublishNextPathSegment(boost::shared_ptr<bool> pathResponse) {
	if (pathResponse) {
		cout << "Path finished." << endl;
		if (!startPushing) {
			if (pathIdx < 1) {
				for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
				cout << "Pushing finished!" << endl;
				return;
			}
			motorCmd[0] = -40000;
			motorCmd[1] = 0;
			motorCmd[2] = 2000000;
			boost::shared_ptr< std::vector<int> > motorCmdData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(motorCmd.begin(),motorCmd.end()));
			for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
			motorCmdInformer->publish(motorCmdData);
			usleep(3000000);

			currObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x()/cellSize,pushingPath->pose(pathIdx).y()/cellSize);
			pathIdx--;
			cout << pathIdx << endl;
			nextDestObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x()/cellSize,pushingPath->pose(pathIdx).y()/cellSize);
			cv::Point2f objectShiftVector = nextDestObjectPos - currObjectPos;
			cv::Point2f objectRobotVector = objectShiftVector * ((objectRadius*0.9+robotRadius)/cv::norm(objectShiftVector));
			startOwnPos = currObjectPos - objectRobotVector;
			destOwnPos = nextDestObjectPos - objectRobotVector*0.5f;

			// Convert that path to a pose2DList
			startOwnPos *= cellSize;
			destOwnPos *= cellSize;
		}
		rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
		twbTracking::proto::Pose2D *destPose = pose2DList->add_pose();
		destPose->set_x(destOwnPos.x);
		destPose->set_y(destOwnPos.y);
		destPose->set_orientation(0);
		destPose->set_id(0);
		if (startPushing) {
			startPushing = false;
		} else {
			twbTracking::proto::Pose2D *startPose = pose2DList->add_pose();
			startPose->set_x(startOwnPos.x);
			startPose->set_y(startOwnPos.y);
			startPose->set_orientation(0);
			startPose->set_id(0);
		}
		pathInformer->publish(pose2DList);
		for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::RED));
		cout << "Next path segment sent." << endl;
	} else cout << "Path error!" << endl;
}

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string proxSensorInscope = "/rir_prox";
	std::string trackingInscope = "/murox/roboterlocation";
	std::string floorInscope = "/prox/floor";
//	std::string pathResponseInscope = "/pathResponse";
	std::string pathOutScope = "/path";
	std::string edgeOutscope = "/edge";

	std::string mapServerScope = "/mapGenerator";
	std::string pathResponseInScope = "/pathResponse";
	std::string commandInscope = "/";
	std::string stateOutscope = "";


	// id of the tracking marker
	int trackingMarkerID = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("id", po::value<int>(&trackingMarkerID),
			"ID of the tracking marker")("edgeout", po::value<std::string>(&edgeOutscope),
			"Outscope for edge found signals")("pathRe", po::value<std::string>(&pathOutScope),
			"Inscope for path responses.")("pathOut", po::value<std::string>(&pathOutScope),
			"Outscope for the robots path.")("mapServer", po::value<std::string>(&mapServerScope),
			"Scope for the mapGenerator server")("meterPerPixel,mpp",po::value<float>(&meterPerPixel), "Camera parameter: Meter per Pixel");

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

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();



	// ------------ Converters ----------------------

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// Register converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// register converter for cv::Mat
	boost::shared_ptr<MatConverter> matConverter(new MatConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(matConverter);

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ---------- Listener ---------------

	// prepare RSB for commands from the state machine
	rsb::ListenerPtr commandListener = factory.createListener(commandInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>(1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));

	// prepare RSB listener for the floor prox data
	rsb::ListenerPtr floorListener = factory.createListener(floorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >floorQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	floorListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(floorQueue)));

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

//	// prepare RSB listener for path responses
//	rsb::ListenerPtr pathResponseListener = factory.createListener(pathResponseInscope);
//	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>pathResponseQueue(
//			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
//	pathResponseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(pathResponseQueue)));

	// prepare RSB listener for localPlanner responses
	rsb::ListenerPtr pathPlannerListener = factory.createListener(pathResponseInScope);
	pathPlannerListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<bool>(&calcAndPublishNextPathSegment)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish states
	pathInformer = factory.createInformer<twbTracking::proto::Pose2DList>(pathOutScope);

	// create rsb informer to publish the robots path
	rsb::Informer<std::string>::Ptr pathInformer = factory.createInformer<std::string>(stateOutscope);

	// mapGenertor server
	RemoteServerPtr mapServer = factory.createRemoteServer(mapServerScope);

	cv::Point2f currOwnPos;

	cv::Mat map;

	try {
		map = *mapServer->call<cv::Mat>("getObstacleMap");
	} catch (const rsc::threading::FutureTimeoutException & e) {
		cerr << "MapGenerator not responding when trying to retrieve obstacle map! CleanUpTask shutting down." << endl;
		return EXIT_FAILURE;
	}

	while(trackingQueue->empty()) {
		cout << "Waiting for tracking for dumping ground..." << endl;
		usleep(250000);
	}
	cv::Point3f destObjectPose = readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()),9);
	destObjectPos = cv::Point2f(destObjectPose.x/cellSize, destObjectPose.y/cellSize);

	while(trackingQueue->empty()) {
		cout << "Waiting for tracking for robot..." << endl;
		usleep(250000);
	}
	cv::Point3f robotPose = readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()),trackingMarkerID);
	currOwnPos = cv::Point2f(robotPose.x/cellSize, robotPose.y/cellSize);
//	cout << currOwnPos.x << ", " << currOwnPos.y << endl;

	map.convertTo(help0, CV_16S);
	help0 += 128;
	help0.convertTo(help1, CV_8UC1);

	// Blob detection
	vector<vector<cv::Point2i> > contours;
	cv::Mat mask, thresholded;

	// Get area inside of walls
	cv::threshold(map,thresholded,250,255,cv::THRESH_BINARY);
	thresholded.copyTo(mask);
	cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	mask = Mat::zeros(map.size(),CV_8UC1);
	cv::drawContours(mask, contours, -1, cv::Scalar(255),-1);

	cv::Mat help = Mat::ones(map.size(),CV_8UC1)*255;
	thresholded.copyTo(help,mask);

	cout << "Area inside of walls computed." << endl;

	// Switch black & white
	cv::Mat objects = Mat::ones(map.size(), CV_8UC1)*255;
	cv::subtract(objects, help, objects);

	// Find objects
	cv::findContours(objects, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//	cv::cvtColor(map, help1, CV_GRAY2BGR);
//	cv::drawContours(help1, contours, -1, cv::Scalar(255,105,65),-1);

	cout << "Objects extracted." << endl;

	int numObjects = contours.size();
	cv::Point2f centers[numObjects];
	cv::Moments moments;
	int minDist = std::numeric_limits<int>::max(), idx = minDist;

	for(int i = 0; i < numObjects; ++i) {
		if (cv::contourArea(contours[i]) < 5) continue;

		// Calculate objects center of gravity
		moments = cv::moments(contours[i], true);
		centers[i] = Point2f(moments.m10/moments.m00 , moments.m01/moments.m00);
//		cv::circle(help1, centers[i], 3, cv::Scalar(139,0,0),-1);

		// Get closest object's center
		if (cv::norm(centers[i]-currOwnPos) < minDist) {
			minDist = cv::norm(centers[i]-currOwnPos);
			currObjectPos = centers[i];
			idx = i;
		}
	}



	if (numObjects > idx) {
		cout << numObjects << " Objects found. Closest one calculated. Retrieving pushing path." << endl;
//		cv::drawContours(help1, contours, idx, cv::Scalar(255,191,0),-1);
//		cv::circle(help1, centers[idx], 3, cv::Scalar(139,0,0),-1);
		cv::Point2f center;
		cv::minEnclosingCircle(contours[idx],center,objectRadius);

		boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList(new twbTracking::proto::Pose2DList);
		twbTracking::proto::Pose2D *pose2D1 = pose2DList->add_pose();
		pose2D1->set_x(currObjectPos.x * cellSize);
		pose2D1->set_y(currObjectPos.y * cellSize);
		pose2D1->set_orientation(0);
		pose2D1->set_id(0);
		twbTracking::proto::Pose2D *pose2D2 = pose2DList->add_pose();
		pose2D2->set_x(destObjectPos.x * cellSize);
		pose2D2->set_y(destObjectPos.y * cellSize);
		pose2D2->set_orientation(0);
		pose2D2->set_id(0);
		for (int i = 0; i < contours[idx].size(); ++i) {
			cv::Point2f p = contours[idx][i];
			twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
			pose2D->set_x(p.x);
			pose2D->set_y(p.y);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}
		try {
			pushingPath = mapServer->call<twbTracking::proto::Pose2DList>("getPushingPath", pose2DList);
		} catch (const rsc::threading::FutureTimeoutException & e) {
			cerr << "MapGenerator not responding when trying to retrieve pushing path! CleanUpTask shutting down." << endl;
			return EXIT_FAILURE;
		}

		cout << "Pushing path retrieved (length: " << pushingPath->pose_size() << "). Calculating starting position." << endl;

		// Calculate clean up path
		if (pushingPath->pose_size() > 0) {
			pathIdx = pushingPath->pose_size()-1;
			nextDestObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x()/cellSize,pushingPath->pose(pathIdx).y()/cellSize);
			cv::Point2f objectShiftVector = nextDestObjectPos - currObjectPos;
			cv::Point2f objectRobotVector = objectShiftVector * ((objectRadius+robotRadius)/cv::norm(objectShiftVector));
			startOwnPos = currObjectPos - objectRobotVector;
			destOwnPos = nextDestObjectPos - objectRobotVector*0.5f;
			currObjectPos *= cellSize;
			startOwnPos *= cellSize;
			destOwnPos *= cellSize;

			cout << "Starting position calculated. Retrieving path thereto." << endl;

			boost::shared_ptr<twbTracking::proto::Pose2D> startOwnPose2D(new twbTracking::proto::Pose2D());
			startOwnPose2D->set_x(startOwnPos.x);
			startOwnPose2D->set_y(startOwnPos.y);
			startOwnPose2D->set_orientation(0);
			startOwnPose2D->set_id(0);
			try {
				pathToStartPos = mapServer->call<twbTracking::proto::Pose2DList>("getPath", startOwnPose2D);
			} catch (const rsc::threading::FutureTimeoutException & e) {
				cerr << "MapGenerator not responding when trying to retrieve path to start position! CleanUpTask shutting down." << endl;
				return EXIT_FAILURE;
			}

			pathInformer->publish(pathToStartPos);
			pathToStartPosSent = true;
			cout << "Path to starting position found and published." << endl;

			for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));

		} else cout << "No pushing path found!" << endl;
	} else cout << "No objects found!" << endl;

	cout << "Waiting for path event..." << endl;
	while(pathIdx > 0) {
		usleep(250000);
	}


	while(true) {
		boost<shared_ptr<string>> command = commandQueue->pop();
		switch (*command) {
			case "init":
				// call init method
				break;
			default:
				break;
		}
	}

	return EXIT_SUCCESS;
}
