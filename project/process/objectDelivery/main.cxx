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
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

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
#include <rsb/Handler.h>
#include <rsc/threading/SynchronizedQueue.h>

using namespace rsb;
using namespace rsb::patterns;

// converters
#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>
#include <extspread.hpp>
#include <ControllerAreaNetwork.h>

#include <kbhit.hpp>

// camera parameter
float meterPerPixel = 1.0 / 400.0;
const float cellSize = 0.01;

/*// Constants
enum SENSOR_POS {
	MIDDLE_RIGHT = 0, RIGHT = 1, LEFT = 2, MIDDLE_LEFT = 3
};
enum EDGE_SIDE {
	UNKNOWN, LEFTSIDE, RIGHTSIDE
};

// Datastructure for the CAN messages

int floorProxMinValues[4] = { 3200, 3100, 3300, 4900 };
int floorProxMaxValues[4] = { 31000, 22200, 28200, 34200 };
bool lineBelow[4] = { false, false, false, false };
const boost::shared_ptr<std::vector<int>> vecSteering(new std::vector<int>(3));

rsb::Informer<std::vector<int>>::Ptr steeringInformer;
float floorProxValuesNormalized[4];*/

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

boost::shared_ptr<twbTracking::proto::Pose2DList> pathToStartPosTmp(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> pathToStartPos(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> pushingPath(new twbTracking::proto::Pose2DList);
int pathIdx = -1;
cv::Point2f destObjectPos(164, 52), nextDestObjectPos = destObjectPos, currObjectPos, startOwnPos, destOwnPos;
bool pathToStartPosSent = false, startPushing = true;
rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathInformer;
rsb::Informer< std::vector<int> >::Ptr motorCmdInformer;
float objectRadius = 0;
int robotRadius = 5; // check!
std::vector<int> motorCmd(3,0);
ControllerAreaNetwork myCAN;

bool debug = false;

//debug
boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList4PathRequest(new twbTracking::proto::Pose2DList);
//	boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList4Drawing(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> pushingPath4Drawing(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> path4Drawing(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> pushingArrow4Drawing(new twbTracking::proto::Pose2DList);

rsb::Informer<twbTracking::proto::Pose2DList>::Ptr drawObjectsInformer, drawPathInformer, drawArrowsInformer;

boost::shared_ptr<twbTracking::proto::Pose2D> objectPtr(new twbTracking::proto::Pose2D());

RemoteServerPtr mapServer;

cv::Point3i color_blue(255,0,0), color_red(0,0,255), color_green(0,255,0), color_orange(36,127,255);

twbTracking::proto::Pose2D object;

rsb::Informer<twbTracking::proto::Pose2D>::Ptr insertObjectInformer, deleteObjectInformer;

cv::Point2f currOwnPos;

void addPos2PoseList(boost::shared_ptr<twbTracking::proto::Pose2DList> &pose2DList, cv::Point2f pos2D, float orientation=0) {
	twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
	pose2D->set_x(pos2D.x);
	pose2D->set_y(pos2D.y);
	pose2D->set_orientation(orientation);
	pose2D->set_id(0);
}

void calcAndPublishNextPathSegment(boost::shared_ptr<bool> pathResponse) {
	if (!pathResponse) {
		cout << "Path error!" << endl;
		return;
	}
	if (pathIdx<0) {
		for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
		cout << "Pushing finished!" << endl;
		return;
	}
	currObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x(),pushingPath->pose(pathIdx).y());
	pathIdx--;
	nextDestObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x(),pushingPath->pose(pathIdx).y());
	cv::Point2f objectShiftVector = nextDestObjectPos - currObjectPos;
	cv::Point2f objectRobotVector = objectShiftVector * (objectRadius/cv::norm(objectShiftVector));
	startOwnPos = currObjectPos - objectRobotVector;
	destOwnPos = nextDestObjectPos - objectRobotVector*0.5f;

	if(debug){
		pushingArrow4Drawing->Clear();
		addPos2PoseList(pushingArrow4Drawing, startOwnPos, 3);
		addPos2PoseList(pushingArrow4Drawing, destOwnPos, 3);
		addPos2PoseList(pushingArrow4Drawing, cv::Point2f(color_red.x, color_red.y), color_red.z);
	}

	cout << "Starting position calculated. Retrieving path thereto." << endl;

	boost::shared_ptr<twbTracking::proto::Pose2D> startOwnPose2D(new twbTracking::proto::Pose2D());
	startOwnPose2D->set_x(startOwnPos.x);
	startOwnPose2D->set_y(startOwnPos.y);
	startOwnPose2D->set_orientation(0);
	startOwnPose2D->set_id(0);

	insertObjectInformer->publish(objectPtr);

	try {
		pathToStartPosTmp = mapServer->call<twbTracking::proto::Pose2DList>("getPath", startOwnPose2D);
	} catch (const rsc::threading::FutureTimeoutException & e) {
		cerr << "MapGenerator not responding when trying to retrieve path to start position! CleanUpTask shutting down." << endl;
		return;
	}

	pathToStartPos->Clear();
	addPos2PoseList(pathToStartPos, destOwnPos, 0);
	for(int i = 0; i < pathToStartPosTmp->pose_size(); ++i) {
		addPos2PoseList(pathToStartPos, cv::Point2f(pathToStartPosTmp->pose(i).x(),pathToStartPosTmp->pose(i).y()), 0);
	}

	deleteObjectInformer->publish(objectPtr);

	cout << "Path to starting position retrieved (length: " << pathToStartPos->pose_size() << ")" << endl;

	if(debug){
		path4Drawing->Clear();
		path4Drawing->CopyFrom(*pathToStartPosTmp);
		addPos2PoseList(path4Drawing, currOwnPos, 3);
		addPos2PoseList(path4Drawing, cv::Point2f(color_orange.x, color_orange.y), color_orange.z);
		currOwnPos = destOwnPos;
	}

	pathInformer->publish(pathToStartPos);
	cout << "Path to starting position published." << endl;

	for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));

//	if (!startPushing) {
//		if (pathIdx==0) {
//			for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
//			cout << "Pushing finished!" << endl;
//			return;
//		}
//		motorCmd[0] = -40000;
//		motorCmd[1] = 0;
//		motorCmd[2] = 2000000;
//		boost::shared_ptr< std::vector<int> > motorCmdData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(motorCmd.begin(),motorCmd.end()));
//		for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
//		motorCmdInformer->publish(motorCmdData);
//		usleep(3000000);
//
//		currObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x(),pushingPath->pose(pathIdx).y());
//		pathIdx--;
//		cout << pathIdx << endl;
//		nextDestObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x()/cellSize,pushingPath->pose(pathIdx).y()/cellSize);
//		cv::Point2f objectShiftVector = nextDestObjectPos - currObjectPos;
//		cv::Point2f objectRobotVector = objectShiftVector * ((objectRadius*0.9+robotRadius)/cv::norm(objectShiftVector));
//		startOwnPos = currObjectPos - objectRobotVector;
//		destOwnPos = nextDestObjectPos - objectRobotVector*0.5f;
//	}
//	rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
//	twbTracking::proto::Pose2D *destPose = pose2DList->add_pose();
//	destPose->set_x(destOwnPos.x);
//	destPose->set_y(destOwnPos.y);
//	destPose->set_orientation(0);
//	destPose->set_id(0);
//	if (startPushing) {
//		startPushing = false;
//	} else {
//		twbTracking::proto::Pose2D *startPose = pose2DList->add_pose();
//		startPose->set_x(startOwnPos.x);
//		startPose->set_y(startOwnPos.y);
//		startPose->set_orientation(0);
//		startPose->set_id(0);
//	}
//	pathInformer->publish(pose2DList);
//	for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::RED));
//	cout << "Next path segment sent." << endl;
}

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string proxSensorInscope = "/rir_prox";
	std::string trackingInscope = "/murox/roboterlocation";
	std::string floorInscope = "/prox/floor";
//	std::string pathResponseInscope = "/pathResponse";
	std::string pathOutScope = "/path";
	std::string edgeOutscope = "/edge";
	std::string steeringOutScope = "/motor/04";
	std::string mapServerScope = "/mapGenerator";
	std::string pathResponseInScope = "/pathResponse";
	std::string rsbMotoOutScope = "/motor/03";
	std::string stateMachineCmdScope = "/objectDelivery/command";
	std::string stateMachineAnswerScope = "/objectDelivery/answer";
	std::string drawObjectsOutscope = "/draw/objects";
	std::string drawPointsOutscope = "/draw/points";
	std::string drawPathOutscope = "/draw/path";
	std::string drawArrowsOutscope = "/draw/arrows";
	std::string insertObjectOutscope = "/mapGenerator/insertObject";
	std::string deleteObjectOutscope = "/mapGenerator/deleteObject";

	std::string spreadhost = "127.0.0.1";
	std::string spreadport = "4803";

	// id of the tracking marker
	int trackingMarkerID = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()
			("help,h", "Display a help message.")
			("id", po::value<int>(&trackingMarkerID), "ID of the tracking marker")
			("edgeout", po::value<std::string>(&edgeOutscope), "Outscope for edge found signals")
			("pathRe", po::value<std::string>(&pathOutScope), "Inscope for path responses.")
			("pathOut", po::value<std::string>(&pathOutScope), "Outscope for the robots path.")
			("mapServer", po::value<std::string>(&mapServerScope), "Scope for the mapGenerator server")
			("host", po::value<std::string>(&spreadhost), "Host for Programatik Spread.")
			("port", po::value<std::string>(&spreadport), "Port for Programatik Spread.")
			("meterPerPixel,mpp", po::value<float>(&meterPerPixel), "Camera parameter: Meter per Pixel")
			("debugVis", "Debug mode: object selection via keyboard & visualization");

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

	debug = vm.count("debugVis") > 0;
	if(debug) cout << "Debug mode: object selection via keyboard (number as Id). Visualization data will be published for showMap." << endl;

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

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

	/*// prepare RSB listener for the floor prox data
	rsb::ListenerPtr floorListener = factory.createListener(floorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >floorQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	floorListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(floorQueue)));

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));
*/
	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

//	// prepare RSB listener for path responses
//	rsb::ListenerPtr pathResponseListener = factory.createListener(pathResponseInscope);
//	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>pathResponseQueue(
//			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
//	pathResponseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(pathResponseQueue)));

	// prepare RSB listener for localPlanner responses
	rsb::ListenerPtr pathPlannerListener = factory.createListener(pathResponseInScope);
	pathPlannerListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<bool>(&calcAndPublishNextPathSegment)));

	rsb::ListenerPtr stateMachineListener = factory.createListener(stateMachineCmdScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D> > > stateMachineCmdQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D> >(1));
	stateMachineListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(stateMachineCmdQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish the robots path
	pathInformer = factory.createInformer<twbTracking::proto::Pose2DList>(pathOutScope);

	// Prepare RSB informer
//	motorCmdInformer = factory.createInformer< std::vector<int> > (rsbMotoOutScope);

	if(debug){
		drawObjectsInformer = factory.createInformer<twbTracking::proto::Pose2DList>(drawObjectsOutscope);
		//	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr  drawPointsInformer = factory.createInformer<twbTracking::proto::Pose2DList>(drawPointsOutscope);
		drawPathInformer = factory.createInformer<twbTracking::proto::Pose2DList>(drawPathOutscope);
		drawArrowsInformer = factory.createInformer<twbTracking::proto::Pose2DList>(drawArrowsOutscope);
	}

	boost::shared_ptr<twbTracking::proto::Pose2DList> objectsList;
	rsb::Informer<std::string>::Ptr stateMachineInformer = factory.createInformer< std::string > (stateMachineAnswerScope);
	insertObjectInformer = factory.createInformer<twbTracking::proto::Pose2D>(insertObjectOutscope);
	deleteObjectInformer = factory.createInformer<twbTracking::proto::Pose2D>(deleteObjectOutscope);

	mapServer = factory.createRemoteServer(mapServerScope);

	boost::shared_ptr<bool> truePtr(new bool(true));
	boost::shared_ptr<std::string> finishStr(new string("finish"));

	if(debug){
		destObjectPos = cv::Point2f(135,139)*cellSize;
		try {
			objectsList = mapServer->call<twbTracking::proto::Pose2DList>("getObjectsList");
		} catch (const rsc::threading::FutureTimeoutException & e) {
			cerr << "Error: MapGenerator not responding when trying to retrieve objects list! Exiting deliverObject." << endl;
			return EXIT_FAILURE;
		}
		addPos2PoseList(objectsList, cv::Point2f(color_green.x, color_green.y), color_green.z);
		cout << "Objects list retrieved: " << objectsList->pose_size() << " objects found." << endl;
	} else {
		cout << "Waiting for tracking for dumping ground..." << endl;
//		while(trackingQueue->empty()) usleep(250000);
//		cv::Point3f destObjectPose = readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()),9);
//		destObjectPos = cv::Point2f(destObjectPose.x, destObjectPose.y);
	}

	while(true) {

		cout << "Waiting for next object request..." << endl;

		while((!debug && stateMachineCmdQueue->empty()) || (debug && !kbhit())){
			if(debug){
				drawObjectsInformer->publish(objectsList);
				drawPathInformer->publish(pushingPath4Drawing);
				drawPathInformer->publish(path4Drawing);
				drawArrowsInformer->publish(pushingArrow4Drawing);
			}
			usleep(125000);
			if (pathIdx == 0) {
				stateMachineInformer->publish(finishStr);
				pathIdx = -1;
			}
		}
		if(debug){
			// select object via keyboard (number as Id)
			char c;
			cin >> c;
			object = objectsList->pose(c-'0');
		} else object = *stateMachineCmdQueue->pop();

		objectRadius = object.orientation();
		currObjectPos = cv::Point2f(object.x(), object.y());

		objectPtr->set_x(object.x());
		objectPtr->set_y(object.y());
		objectPtr->set_orientation(object.orientation());
		objectPtr->set_id(0);

		cout << "Deliver command received. Start delivering object... " << endl;

		if(debug){
			currOwnPos = cv::Point2f(129,57)*cellSize;
		} else {
//			cout << "Waiting for tracking for robot..." << endl;
//			while(trackingQueue->empty()) usleep(250000);
//			cv::Point3f robotPose = readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()),trackingMarkerID);
//			currOwnPos = cv::Point2f(robotPose.x, robotPose.y);
		}

		pose2DList4PathRequest->Clear();
		addPos2PoseList(pose2DList4PathRequest, currObjectPos, 4);
		addPos2PoseList(pose2DList4PathRequest, destObjectPos, 4);
		*pose2DList4PathRequest->add_pose() = object;

		try {
			pushingPath = mapServer->call<twbTracking::proto::Pose2DList>("getPushingPath", pose2DList4PathRequest);
		} catch (const rsc::threading::FutureTimeoutException & e) {
			cerr << "Error: MapGenerator not responding when trying to retrieve pushing path! Exiting deliverObject." << endl;
			return EXIT_FAILURE;
		}
		addPos2PoseList(pushingPath, currObjectPos, 0);

		if(debug){
			cout << "Pushing path retrieved (length: " << pushingPath->pose_size() << ")." << endl;
			pushingPath4Drawing->CopyFrom(*pushingPath);
			addPos2PoseList(pushingPath4Drawing, cv::Point2f(color_blue.x, color_blue.y), color_blue.z);
		}

		if (pushingPath->pose_size() > 1) {
			pathIdx = pushingPath->pose_size()-1;
			calcAndPublishNextPathSegment(truePtr);
		} else cout << "No pushing path found!" << endl;
	}

	return EXIT_SUCCESS;
}
