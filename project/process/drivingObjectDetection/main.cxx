//============================================================================
// Name        : main.cxx
// Author      : mbarther
// Description : -
//============================================================================

//#define TRACKING
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>


#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>

#include <Eigen/Geometry>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>


using namespace rsb;

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

#include <converter/vecIntConverter/main.hpp>
//#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>

#include <ControllerAreaNetwork.h>

using namespace rsb;
using namespace rsb::patterns;

float diameter = 0.1;
int trackingMarkerID = 0;
float meterPerPixel = 1.0/400.0;

#define VEL_TURNING 40
#define VEL_FORWARD 8

float robotRadius = 50000;
float robotObjectDist = 10000;
float turnThreshold = 5 * 180/M_PI;

int objCount = 0;
int objOffset = 2;

// scopenames for rsb
std::string progressInscope = "/objectDetectionMain/command";
std::string progressOutscope = "/objectDetectionMain/answer";
std::string odometryInscope = "/odom";
std::string pathResponseInscope = "/pathResponse";
std::string pathOutScope = "/path";
std::string mapServerScope = "/mapGenerator";
std::string objectOutscope = "/objectDetection/command";
std::string objectInscope = "/objectDetection/detected";

// string publisher
boost::shared_ptr<std::string> stringPublisher(new std::string);


// terminal flags
bool skipPP = false;
bool skipLP = false;
bool skipFR = false;
bool skipOD = false;
bool skipVC = false;


// method prototypes
void getOwnPosition(types::position& ts_pose, rst::geometry::Pose odomInput);
void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN);
float normAngle(float angle);
int mymcm(int mym);


int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			     ("skipPathPlanner", "Skipping Path Planner.")
			     ("skipLocalPlanner", "Skipping Local Planner.")
			     ("skipFinalRotation", "Skipping Final Rotation towards the object.")
			     ("skipDetection", "Skipping Object Detection.")
			     ("skipCorrection", "Skipping View Correction after detection failure.");

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

	skipPP = vm.count("skipPathPlanner");
	skipLP = vm.count("skipLocalPlanner");
	skipFR = vm.count("skipFinalRotation");
	skipOD = vm.count("skipDetection");
	skipVC = vm.count("skipCorrection");

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// ------------ Converters ----------------------

	// Register converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// Register new converter for Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterlocalization);

        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
        rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);

	// ------------ Listener ----------------------

	// prepare RSB listener for path responses
	rsb::ListenerPtr pathResponseListener = factory.createListener(pathResponseInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>pathResponseQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
	pathResponseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(pathResponseQueue)));

	// prepare rsb listener for progress data
        rsb::ListenerPtr progressListener = factory.createListener(progressInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>progressQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	progressListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(progressQueue)));

	// Prepare RSB async listener for odometry messages
	rsb::ListenerPtr odomListener = factory.createListener(odometryInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>>odomQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>(1));
	odomListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::Pose>(odomQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish the robots path
	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathInformer = factory.createInformer<twbTracking::proto::Pose2DList>(pathOutScope);

	// mapGenertor server
	RemoteServerPtr mapServer = factory.createRemoteServer(mapServerScope);

	// create rsb informer to publish progress data
	rsb::Informer<std::string>::Ptr progressInformer = factory.createInformer<std::string>(progressOutscope);



	// Init the CAN interface
	ControllerAreaNetwork myCAN;

        // do algorithm
	boost::shared_ptr<twbTracking::proto::Pose2D> objectPositionPtr(new twbTracking::proto::Pose2D());
	boost::shared_ptr<twbTracking::proto::Pose2D> detectionPositionPtr(new twbTracking::proto::Pose2D());
	types::position ownPos;

	/*
         * 1) Wait for object position + radius
         * 2) Calculate final detection position (fdp)
         * 3) Get path to fdp
         * 4) Drive to fdp
         * 5) Rotate towards object
         * 6) Do object detection
         * 7) If not detected:
         * 7.1) Drive away from object
         * 7.2) Continue with 2)
         * 8) Publish result
         */
        INFO_MSG("Start progress");

	while (true) {
		if (!progressQueue->empty()) {
			// load object position
			objectPositionPtr = progressQueue->pop();
                        bool objectDetected = false;
			float detectionDist = robotRadius + robotObjectDist + objectPositionPtr->orientation();
			INFO_MSG("Focusing on object at position " << objectPositionPtr->x() << "/" << objectPositionPtr->y() << " with a radius of " << objectPositionPtr->orientation() << " um");

			// if the object hasn't been detected yet
                        while(!objectDetected) {
				// calculate final detection position
				getOwnPosition(ownPos, *odomQueue->pop());
				INFO_MSG("Find oneself at position " << ownPos.x << "/" << ownPos.y);
				float angleToObject = atan2(objectPositionPtr->y()-ownPos.y, objectPositionPtr->x()-ownPos.x);
				detectionPositionPtr->set_x(objectPositionPtr->x()-detectionDist*cos(angleToObject));
				detectionPositionPtr->set_y(objectPositionPtr->y()-detectionDist*sin(angleToObject));
				INFO_MSG("Final detection position is " << detectionPositionPtr->x() << "/" << detectionPositionPtr->y());
/*
				// calculate path to final detection position
				boost::shared_ptr<twbTracking::proto::Pose2DList> path = mapServer->call<twbTracking::proto::Pose2DList>("getPath", detectionPositionPtr);
				INFO_MSG("Calculated path (#items: " << path->pose_size() << "):");
				for (int i=0; i<path->pose_size(); i++) {
					INFO_MSG(" -> " << path->pose(i).x() << "/" << path->pose(i).y());
				}

				// drive to final detection position
				pathInformer->publish(path);
				if (!pathResponseQueue->empty()) {
					pathResponseQueue->pop();
				}
				usleep(500000);
				while (pathResponseQueue->empty());
				pathResponseQueue->pop();
				INFO_MSG("Finished driving.");
*/

				// test movement
/*				float ownAngleD = normAngle(((float)ownPos.f_z)/1000000.0);
				float driveAngle = atan2(detectionPositionPtr->y()-ownPos.y, detectionPositionPtr->x()-ownPos.x);
				float driveDist = sqrt(pow(detectionPositionPtr->y()-ownPos.y, 2) + pow(detectionPositionPtr->x()-ownPos.x, 2));

				float turnAngleD = 0;
				float angleDiffD = abs(driveAngle-ownAngleD);
				if (angleDiffD < M_PI && angleDiffD > turnThreshold) {
					turnAngleD = driveAngle-ownAngleD;
				} else {
					turnAngleD = driveAngle-ownAngleD+2*M_PI;
				}
				float facD = 1;
				if (turnAngleD < 0) facD = -1;
				int waitingTimeD_us = (int)(((turnAngleD*1000.0) / ((float)VEL_TURNING*10.0*facD)) * 1000000);
				sendMotorCmd(0, mymcm(facD*VEL_TURNING), myCAN);
				usleep(waitingTimeD_us);
				sendMotorCmd(0, 0, myCAN);

				waitingTimeD_us = (int)(driveDist / ((float)VEL_FORWARD*10000.0) * 1000000);
				sendMotorCmd(mymcm(VEL_FORWARD), 0, myCAN);
				usleep(waitingTimeD_us);
				sendMotorCmd(0, 0, myCAN);
*/

				// rotate towards object
				if (!skipFR) {
					getOwnPosition(ownPos, *odomQueue->pop());
					float ownAngle = normAngle(((float)ownPos.f_z)/1000000.0);
        	                        DEBUG_MSG("Own Position is " << ownPos.x << "/" << ownPos.y << " (" << ownAngle << " rad)");
					float angle = atan2(objectPositionPtr->y()-ownPos.y, objectPositionPtr->x()-ownPos.x);
					float turnAngle = 0;
					float angleDiff = abs(angle-ownAngle);
					if (angleDiff < M_PI && angleDiff > turnThreshold) {
						turnAngle = angle-ownAngle;
					} else {
						turnAngle = angle-ownAngle+2*M_PI;
					}
        	                        DEBUG_MSG("Angle = " << angle << ", angle diff = " << angleDiff << ", turn angle = " << turnAngle);
					float fac = 1;
					if (turnAngle < 0) fac = -1;
					int waitingTime_us = (int)(((turnAngle*1000.0) / ((float)VEL_TURNING*10.0*fac)) * 1000000);
					INFO_MSG("Turning for " << turnAngle << " rad for " << (waitingTime_us/1000) << " ms with a speed of " << fac*VEL_TURNING/100 << " rad/s");
					sendMotorCmd(0, mymcm(fac*VEL_TURNING), myCAN);
					usleep(waitingTime_us);
					sendMotorCmd(0, 0, myCAN);
				}

				// do object detection
				if (!skipOD) {
				} else {
					objectDetected = true;
				}

				// correct object view if not detected
				if (!skipVC) {
				} else {
					break;
				}
			}

			// publish result
                        objCount++;
			std::string sOutput = std::to_string(objCount+objOffset);
			if (!objectDetected) {
				sOutput = "null";
			} else {
	                        INFO_MSG("Object " << (objCount+objOffset) << " detected");
			}
			*stringPublisher = sOutput;
			progressInformer->publish(stringPublisher);
			
		} else {
			usleep(500000);
		}
        }

	return EXIT_SUCCESS;
}


void getOwnPosition(types::position& pose, rst::geometry::Pose odomInput) {
	rst::geometry::Translation translation = odomInput.translation();
	rst::geometry::Rotation rotation = odomInput.rotation();

	// Convert from quaternion to euler
	Eigen::Quaterniond lidar_quat(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
	Eigen::AngleAxisd lidar_angle(lidar_quat);
	Eigen::Matrix<double,3,1> rpy = lidar_angle.toRotationMatrix().eulerAngles(0,1,2);
	const double yaw = rpy(2);

	// Save data
	pose.x = translation.x()*1000000;
	pose.y = translation.y()*1000000;
	pose.f_z = yaw*1000000;
}

float normAngle(float angle) {
	while (angle >= 2*M_PI) {
		angle -= 2*M_PI;
	}
	while (angle < 0) {
		angle += 2*M_PI;
	}
	return angle;
}


void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed(speed, angle);
	DEBUG_MSG( "v: " << speed << "w: " << angle);
}

int mymcm(int mym) {
	return mym*10000;
}


