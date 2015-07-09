#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <converter/mapUpdateConverter/mapUpdateConverter.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/util/EventQueuePushHandler.h>
#include <types/twbTracking.pb.h>
#include "mapGenerator.hpp"
#include "mapUpdate.hpp"

using namespace boost;
using namespace std;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// size of the map
cv::Size mapSize = cv::Size(400, 400);

const float cellSize = 0.01;

const cv::Scalar colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255,
		0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255), cv::Scalar(0, 125, 125), cv::Scalar(125, 125, 0),
		cv::Scalar(125, 0, 125), cv::Scalar(255, 125, 125), cv::Scalar(125, 125, 255), cv::Scalar(125, 255, 125) };

// camera parameter
float meterPerPixel = 1.0 / 400.0;

// Show an image in a window with the given name.
// The image will be flipped along its x-axis.
// The window will appear at (x,y).
void imshowf(const string & winname, cv::InputArray mat, int x = 0, int y = 0) {
	cv::Mat fmat;
	cv::flip(mat, fmat, 0);
	cv::imshow(winname, fmat);
	cv::moveWindow(winname, x, y);
}

// read tracking data into array
void readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, cv::Point2i poses[], int ids[]) {
	twbTracking::proto::Pose2D pose;
	for (int i = 0; i < 12; i++) {
		if (i < data->pose_size()) {
			pose = data->pose(i);
			poses[i] = cv::Point2i((int) (pose.x() * meterPerPixel / cellSize),
					(int) (pose.y() * meterPerPixel / cellSize));
			ids[i] = pose.id();
		} else {
			poses[i] = cv::Point2i(0, 0);
			ids[i] = -1;
		}
	}
}

int main(int argc, char **argv) {

	std::string mapUpdateScope = "/mapUpdate";
	std::string trackingInscope = "/murox/roboterlocation";
	std::string pathUpdateScope = "/pathUpdate";

	int offset = 0;
	int size = 400;
	float scale = 1.0;

	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("updateinout", po::value<std::string>(&mapUpdateScope),
			"Scope for map updates")("trackingin", po::value<std::string>(&trackingInscope),
			"Inscope for tracking pose")("meterPerPixel,mpp", po::value<float>(&meterPerPixel),
			"Camera parameter: Meter per Pixel")("size,", po::value<int>(&size), "width/height of the map")("offset,o",
			po::value<int>(&offset), "offset from the left side of the screen.")("scale,", po::value<float>(&scale),
			"scale factor");

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

	po::notify(vm);

	mapSize = cv::Size(size, size);

	rsb::Factory &factory = rsb::Factory::getInstance();

	// ------------ Converters ----------------------

	// register converter for MapUpdate
	boost::shared_ptr<MapUpdateConverter> mapUpdateConverter(new MapUpdateConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(mapUpdateConverter);

	// Register new converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// ---------- Listener ---------------

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

	// prepare RSB listener for mapUpdate
	rsb::ListenerPtr mapUpdateListener = factory.createListener(mapUpdateScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<MapUpdate>>>mapUpdateQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<MapUpdate>>(10));
	mapUpdateListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<MapUpdate>(mapUpdateQueue)));

	// prepare RSB listener for paths
	rsb::ListenerPtr pathListener = factory.createListener(pathUpdateScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> pathQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	pathListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(pathQueue)));

	// initialize array of poses
	cv::Point2i poses[12];
	int ids[12];
	for (int i = 0; i < 12; ++i) {
		poses[i] = cv::Point2i(0, 0);
		ids[i] = -1;
	}

	// obect used to update the map from sensorvalues
	MapGenerator mapGenerator(cellSize);

	// initialize maps
	cv::Mat gridmap = cv::Mat::zeros(mapSize, CV_8SC1);
	cv::Mat edgeMap(mapSize, CV_8SC1, Scalar(0));
	cv::Mat combinedMap(mapSize, CV_8SC1, Scalar(0));
	cv::Mat obstacleMap, obstacleMap0, obstacleMap1, combinedMap0, combinedMap1;

	std::vector<cv::Point2i> paths[12];

	while (true) {

		bool replot = false;

		// update the robots positions
		if (!trackingQueue->empty()) {
			readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()), poses, ids);
			replot = true;
		}

		if (!pathQueue->empty()) {

			// wait for a rsb message
			EventPtr event = pathQueue->pop();
			int pathid = atoi(event->getScope().getComponents()[1].c_str());


			boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList = boost::static_pointer_cast<
					twbTracking::proto::Pose2DList>(event->getData());

			paths[pathid].clear();
			for (int i = pose2DList->pose_size() - 1; i >= 0; --i) {
				cv::Point2i p(pose2DList->pose(i).x() / cellSize, pose2DList->pose(i).y() / cellSize);
				paths[pathid].push_back(p);
			}
			replot = true;
		}

		// receive and apply mapUpdates
		while (!mapUpdateQueue->empty()) {
			MapUpdate mapUpdate = *mapUpdateQueue->pop().get();
			if (mapUpdate.theta == 0) {
				mapGenerator.updateMap(mapUpdate, edgeMap);
			} else {
				mapGenerator.updateMap(mapUpdate, gridmap);
			}
			replot = true;
		}

		if (replot) {
			// convert the map to rgb
			combinedMap = gridmap + edgeMap + edgeMap;
			combinedMap.convertTo(combinedMap0, CV_16S);
			combinedMap0 += 128;
			combinedMap0.convertTo(combinedMap1, CV_8UC1);
			cv::cvtColor(combinedMap1, combinedMap1, CV_GRAY2BGR);

			// generate obstacle map
			mapGenerator.generateObstacleMap(combinedMap, obstacleMap);
			cv::cvtColor(obstacleMap, obstacleMap1, CV_GRAY2BGR);

			// plot robotlocations
			for (int i = 0; i < 12; i++) {
				if (poses[i] != cv::Point2i(0, 0) && ids[i] >= 0) {
					cv::circle(combinedMap1, poses[i], 5, colors[ids[i]]);
					cv::circle(obstacleMap1, poses[i], 5, colors[ids[i]]);

					cv::Point2i lastPoint = poses[i];
					for (cv::Point2i pathPoint : paths[ids[i]]) {
						cv::line(combinedMap1, lastPoint, pathPoint, colors[ids[i]]);
						cv::line(obstacleMap1, lastPoint, pathPoint, colors[ids[i]]);
						lastPoint = pathPoint;
					}
				}
			}
			cv::resize(combinedMap1, combinedMap1, Size(0, 0), scale, scale);
			cv::resize(obstacleMap1, obstacleMap1, Size(0, 0), scale, scale);
			imshowf("input", combinedMap1, offset);
			imshowf("obstacle", obstacleMap1, combinedMap1.rows + offset);
		}

		cv::waitKey(1);
	}

	return 0;

}
