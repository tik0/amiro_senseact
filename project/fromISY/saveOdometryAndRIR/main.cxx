//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Gets the robot position and sensor values over RSB,
//               updates the occupancy grid map and send it over RSB
//============================================================================

//#define TRACKING
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>

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

#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>

// micrometer to meter
const float YM_2_M = 1000000.0;

// scopenames for rsb
std::string proxSensorInscope = "/rir_prox";
std::string odometryInscope = "/odometrydata";

int main(int argc, char **argv) {

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	converterRepository<std::string>()->registerConverter(converterVecInt);

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// prepare rsb listener for odometry data
	rsb::ListenerPtr odometryListener = factory.createListener(odometryInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>odoQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	odometryListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(odoQueue)));

	// Register new converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	FILE *file;
	file = fopen("RingProximityAndOdometry.txt", "a");

	// read sensor values

	while (proxQueue->empty() || odoQueue->empty()) {
		// Sleep for a while
		boost::this_thread::sleep( boost::posix_time::milliseconds(200) ); 
	}       

	boost::shared_ptr<std::vector<int>> sensorValues = boost::static_pointer_cast<std::vector<int> >(
			proxQueue->pop());

	// get odometry data
	boost::shared_ptr<twbTracking::proto::Pose2D> robotPosition = boost::static_pointer_cast<
			twbTracking::proto::Pose2D>(odoQueue->pop());

	// calculate the robots pose from odomety data (starting in the middle of the global map)
	int posX = robotPosition->x();
	int posY = robotPosition->y();
	int theta = robotPosition->orientation();

	for (int sensorIdx = 0; sensorIdx < sensorValues->size(); sensorIdx++) {
		fprintf(file, "%i\t", sensorValues->at(sensorIdx));
	}
	fprintf(file, "%i\t%i\t%i\n", posX, posY, theta);

	fclose(file);

	INFO_MSG("Data read");

	return EXIT_SUCCESS;
}
