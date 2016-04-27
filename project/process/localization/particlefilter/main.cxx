
/*
 * Includes
 */
#include <MSG.h>                            // error/debug/... messages
#include <boost/program_options.hpp>        // handles program options

// RSB related
#include <boost/shared_ptr.hpp>
// RSB
#include <rsb/Factory.h>
#include <rsb/Informer.h>
#include <rsb/Handler.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
//#include <types/LocatedLaserScan.pb.h>        // already included in particlefilter.h
//#include <rst/geometry/Translation.pb.h>      // s.a.

// RSC
//#include <rsc/misc/SignalWaiter.h>

// OpenCV for map loading
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>      // imread
#include <opencv2/imgproc/imgproc.hpp>      // cvtColor

#include "particlefilter.h"
#include "raycastingmodel.h"
#include "likelihoodfieldmodel.h"

int main(int argc, const char **argv) {
    /*
     * Handle program options
     */
    std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
    std::string odomInScope = "/AMiRo_Hokuyo/gps";

    size_t sampleCount = 10;
    float meterPerPixel = 0.01;
    std::string pathToMap = "map.png";

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("lidarInScope", po::value < std::string > (&lidarInScope)->default_value(lidarInScope), "Scope for receiving lidar data")
            ("odomInScope", po::value < std::string > (&odomInScope)->default_value(odomInScope), "Scope for receiving odometry data")
            ("sampleCount", po::value < std::size_t > (&sampleCount)->default_value(sampleCount), "Number of particles")
            ("meterPerPixel", po::value < float > (&meterPerPixel)->default_value(meterPerPixel), "resolution of the map in meter per pixel")
            ("pathToMap", po::value < std::string > (&pathToMap)->default_value(pathToMap), "Filesystem path to image that contains the map");

    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

    // first, process the help option
    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }

    // afterwards, let program options handle argument errors
    po::notify(vm);

    /*
     * RSB Informers and listeners
     */
    rsb::Factory& factory = rsb::getFactory();

    // Setup converters
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
    rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
    rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);

    // Prepare RSB listener for incomming lidar scans
    rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
    lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
    // Prepare RSB async listener for odometry messages
    rsb::ListenerPtr odomListener = factory.createListener(odomInScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>>odomQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>(1));
    odomListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::Pose>(odomQueue)));

    /*
     * Setup the particle filter
     */
    // Load map
    Map map(cv::imread(pathToMap, CV_LOAD_IMAGE_GRAYSCALE));
    if (!map.data) {
        ERROR_MSG("Could not load map file: " << pathToMap);
        return 1;
    }
    map.meterPerCell = meterPerPixel;

    // Blocks until first scan is received. It will be used as configuration for the particle filter
    boost::shared_ptr<rst::vision::LocatedLaserScan> scanPtr = lidarQueue->pop();
    // Get inital odometry
    boost::shared_ptr<rst::geometry::Pose> odomPtr = odomQueue->pop();

    // Finally set up the particle filter
    //RayCastingModel sensorModel(&map);
    LikelihoodFieldModel sensorModel(&map);
    ParticleFilter particlefilter(sampleCount, *scanPtr, *odomPtr, &map, &sensorModel);

    /*
     * Main loop
     */
    //rsc::misc::initSignalWaiter();
    while (true) {
        // Update scan
        if (!lidarQueue->empty()) {
            scanPtr = lidarQueue->pop();
        }
        // Update odometry
        if (!odomQueue->empty()) {
            odomPtr = odomQueue->pop();
        }

        boost::uint64_t start = rsc::misc::currentTimeMillis();
        particlefilter.update(*scanPtr, *odomPtr);
        boost::uint64_t stop = rsc::misc::currentTimeMillis();
        INFO_MSG("Updating particle filter took " << (stop - start) << " ms");

#ifndef __arm__
        // Visualization
        // convert to RGB
        cv::Mat3b vis(map.size(), CV_8UC3);
        cv::cvtColor(map, vis, CV_GRAY2RGB, 3);
        // resize
        int height = min(map.size().height, 700);
        float scale = (float)height / map.size().height;
        cv::resize(vis, vis, cv::Size(map.size().width * scale, map.size().height * scale));

        sample_t *samples = particlefilter.getSamples();
        float maxImportance = 0;
        for (size_t i = 0; i < sampleCount; ++i)
            maxImportance = max(maxImportance, samples[i].importance);

        for (size_t i = 0; i < sampleCount; ++i) {
            sample_t sample = samples[i];

            cv::Point p(sample.pose.x / meterPerPixel * scale, sample.pose.y / meterPerPixel * scale);

            int radius = 5;
            int intensity = -255 / maxImportance * sample.importance + 255;
            cv::circle(vis, p, radius, cv::Scalar(255,intensity,intensity));

            cv::Point q( p.x + cos(sample.pose.theta) * radius * 1.5f, p.y + sin(sample.pose.theta) * 1.5f * radius );
            cv::line(vis, p, q, cv::Scalar(255,intensity,intensity));
        }

        cv::flip(vis, vis, 0); // flip horizontally
        cv::imshow("visualization", vis);
        if (cv::waitKey(100) == (char)27) { // 27 = escape key
            break;
        }
#endif
        INFO_MSG("==== END OF MAIN LOOP ====");
    }

    return 0;
}
