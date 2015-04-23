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
#include <boost/thread.hpp>
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
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>

// search the pose-list received from tracking for the robots id and get the corresponding pose.
cv::Point3f convertPose(boost::shared_ptr<twbTracking::proto::Pose2D> data) {
	return cv::Point3f(data->x(), data->y(), data->orientation() * M_PI / 180.0);
}

int main(int argc, char **argv) {

	// id of the tracking marker
	int id = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("id", po::value<int>(&id),
			"ID of the tracking marker");

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

	// scopenames for rsb
	std::string pathResponseInscope = "/amiro"+lexical_cast<std::string>(id)+"/pathResponse";
	std::string pathOutScope = "/amiro"+lexical_cast<std::string>(id)+"/path";
	std::string mapServerScope = "/amiro"+lexical_cast<std::string>(id)+"/mapGenerator";
	std::string poseInscope = "/amiro"+lexical_cast<std::string>(id)+"/pose";


	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// ------------ Converters ----------------------

	// Register converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// ---------- Listener ---------------

	// prepare rsb listener for pose data
	rsb::ListenerPtr poseListener = factory.createListener(poseInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>poseQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	poseListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(poseQueue)));

	// prepare RSB listener for path responses
	rsb::ListenerPtr pathResponseListener = factory.createListener(pathResponseInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>pathResponseQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
	pathResponseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(pathResponseQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish the robots path
	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathInformer = factory.createInformer<
			twbTracking::proto::Pose2DList>(pathOutScope);

	// mapGenertor server
	RemoteServerPtr mapServer = factory.createRemoteServer(mapServerScope);

	bool running = true;
	cv::Point3f edgeApproachPose, robotPose;
	cv::Point2f pathEnd(-100, -100);

//	ControllerAreaNetwork myCAN;

//	for (int led = 0; led < 8; led++)
//		myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));

	while (running) {

		float goaldistance = cv::norm(cv::Point2f(pathEnd.x - robotPose.x, pathEnd.y - robotPose.y));
		if ((!pathResponseQueue->empty() || goaldistance < 0.15)) {
			if (!pathResponseQueue->empty()) {
				cout << "Frontierexplo: Path finished, new path ---------------" << endl;
				pathResponseQueue->pop();
			}
			try {
				// get a new path
				boost::shared_ptr<twbTracking::proto::Pose2DList> path =
						mapServer->call<twbTracking::proto::Pose2DList>("getFrontierPath");

				if (path->pose_size() > 0) {
					twbTracking::proto::Pose2D pEnd = path->pose(0);
					pathEnd = cv::Point2f(pEnd.x(), pEnd.y());
					// send the path to the local planner
					pathInformer->publish(path);
				} else {
					cout << "Exploration finished" << endl;
					running = false;
				}
			} catch (const rsc::threading::FutureTimeoutException & e) {
				cerr << "MapGenerator not responding! FrontierExploration shutting down." << endl;
				running = false;
			}
		}

		if (!poseQueue->empty()) {
			cv::Point3f newRobotPose = convertPose(boost::static_pointer_cast<twbTracking::proto::Pose2D>(poseQueue->pop()));
			if (newRobotPose != cv::Point3f(0, 0, 0)) {
				robotPose = newRobotPose;
			}
		}

	}

//	for (int led = 0; led < 8; led++)
//		myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	return EXIT_SUCCESS;
}
