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

// converters
#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>

// project includes
#include "localPlanner.hpp"

// RST Proto types
#include <rst/kinematics/Twist.pb.h>

// search the pose-list received from tracking for the robots id and get the corresponding pose.
cv::Point3f convertPose(boost::shared_ptr<twbTracking::proto::Pose2D> data) {
	return cv::Point3f(data->x(), data->y(), data->orientation() );
}

int main(int argc, char **argv) {

	std::string steeringOutScope = "/motor";

	// id of the tracking marker
	int id = 0;

	// frequency of sending the map
	int mapFreq = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("simulate,s", "Simulate steering.")("id", po::value<int>(&id),
			"ID")("steeringOut",po::value<std::string>(&steeringOutScope), "Outscope for steering command in the simulation.");
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

	// scopenames for rsb
	std::string pathResponseOutscope = "/amiro"+lexical_cast<std::string>(id)+"/pathResponse";
	std::string pathInScope = "/amiro"+lexical_cast<std::string>(id)+"/path";
	std::string poseInscope = "/amiro"+lexical_cast<std::string>(id)+"/pose";


	// ------------ Converters ----------------------

  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::kinematics::Twist> > converter(new rsb::converter::ProtocolBufferConverter<rst::kinematics::Twist>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// Register new converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ---------- Listener ---------------

	// prepare rsb listener for poses
	rsb::ListenerPtr poseListener = factory.createListener(poseInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>poseQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	poseListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(poseQueue)));

	// prepare RSB listener for paths
	rsb::ListenerPtr pathListener = factory.createListener(pathInScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>pathQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	pathListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(pathQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish steering commands
	rsb::Informer<rst::kinematics::Twist>::Ptr steeringInformer = factory.createInformer<rst::kinematics::Twist>(steeringOutScope);

	// create rsb informer to publish path responses
	rsb::Informer<bool>::Ptr pathResponseInformer = factory.createInformer<bool>(pathResponseOutscope);

	// initialize the robots pose

	cv::Point3f robotPose(0, 0, 0);
	cv::Point3f lastRobotPose = robotPose;

	// object to drive exploration paths
	LocalPlanner localPlanner(steeringInformer,vm.count("simulate")>0);

	bool firstPose = true;
	bool pathInProgress = true;

	while (true) {
       // std::cout << "Waiting for pose ----------------------------------------" << std::endl;
		try {
			// get the position from tracking
			robotPose = convertPose(boost::static_pointer_cast<twbTracking::proto::Pose2D>(poseQueue->pop(1000)));
		} catch (const rsc::threading::QueueEmptyException & e) {
            robotPose = cv::Point3f(0,0,0);
		}

		// if tracking fails stop the robot
		if (robotPose == cv::Point3f(0,0,0)) {
			localPlanner.stopRobot();
			continue;
		}
		//cout <<robotPose<< endl;

		// to start the exploration drive 5 cm forwards
		if (firstPose) {
            //std::cout << "SET PATH -----------------------------------------------" << std::endl;
			/*localPlanner.setPath(
					{ cv::Point2f(robotPose.x +  0.05, robotPose.y +  0.05),
					cv::Point2f(robotPose.x + 0.05, robotPose.y -  0.05),
					cv::Point2f(robotPose.x -  0.05, robotPose.y -  0.05),
					cv::Point2f(robotPose.x -  0.05, robotPose.y +  0.05)});*/
            localPlanner.setPath({cv::Point2f(robotPose.x , robotPose.y )});
			//cout <<"GOAL" << cv::Point2f(robotPose.x , robotPose.y ) << endl;
			firstPose = false;
		}

		// update the path planner
		PATH_STATUS pathStatus = localPlanner.updatePose(robotPose);

		// if the path was finished or if an error occured publish this information
		if (pathInProgress) {
			switch (pathStatus) {
			case PATH_FINISHED: {
				boost::shared_ptr<bool> pathResponse(new bool(true));
				pathResponseInformer->publish(pathResponse);
				pathInProgress = false;
            //    std::cout << "PATH FINISHED ............................................." << std::endl;
			}
				break;
			case PATH_ERROR: {
				boost::shared_ptr<bool> pathResponse(new bool(false));
				pathResponseInformer->publish(pathResponse);
				pathInProgress = false;
			}
				break;
			default: {
			}
				break;
			}
		}

		// read new path
		if (!pathQueue->empty()) {
			localPlanner.setPath(*pathQueue->pop());
			pathInProgress = true;
		}

	}

	return EXIT_SUCCESS;
}
