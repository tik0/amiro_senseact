
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
#include <rsc/misc/SignalWaiter.h>

// OpenCV for map loading
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>      // imread
#include <opencv2/imgproc/imgproc.hpp>      // cvtColor

#include "particlefilter.h"
#include "raycastingmodel.h"
#include "likelihoodfieldmodel.h"

#include "importancetype/inversedistance.h"

int main(int argc, const char **argv) {
    /*
     * Handle program options
     */
    std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
    std::string odomInScope = "/AMiRo_Hokuyo/gps";
    std::string debugImageOutScope = "/particlefilter/debugImage";

    size_t sampleCount = 10;
    float meterPerPixel = 0.01f;
    std::string pathToMap = "map.png";
    int beamskip = 1;
    float newSampleProb = 0.1f;
    float maxFrequency = 10.0f;

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("lidarInScope", po::value < std::string > (&lidarInScope)->default_value(lidarInScope), "Scope for receiving lidar data")
            ("odomInScope", po::value < std::string > (&odomInScope)->default_value(odomInScope), "Scope for receiving odometry data")
            ("sampleCount", po::value < std::size_t > (&sampleCount)->default_value(sampleCount), "Number of particles")
            ("meterPerPixel", po::value < float > (&meterPerPixel)->default_value(meterPerPixel), "resolution of the map in meter per pixel")
            ("pathToMap", po::value < std::string > (&pathToMap)->default_value(pathToMap), "Filesystem path to image that contains the map")
            ("kldsampling", "Switches on KLD sampling.")
            ("newSampleProb", po::value < float > (&newSampleProb)->default_value(newSampleProb), "Probability for generating a new sample (instead of roulette wheel selection)")
            ("beamskip", po::value < int > (&beamskip)->default_value(beamskip), "Take every n-th beam into account when calculating importance factor")
            ("maxFrequency", po::value < float > (&maxFrequency)->default_value(maxFrequency), "Maximum frequency at which new positon is published (1/s)")
            ("debugImageOutScope", po::value < std::string > (&debugImageOutScope), "Scope for sending the debug image.");

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

    // do KLD sampling?
    bool doKLDSampling = false;
    if (vm.count("kldsampling")) {
        INFO_MSG("KLD sampling enabled");
        doKLDSampling = true;
    }

    // send debug image?
    bool sendDebugImage = false;
    if (vm.count("debugImageOutScope")) {
        INFO_MSG("Sending debug image on scope: " << debugImageOutScope);
        sendDebugImage = true;
    }

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

    // Informer for map
    rsb::Informer<std::string>::Ptr debugImageInformer = factory.createInformer<std::string>(debugImageOutScope);

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
    INFO_MSG("Waiting for first laser scan...");
    boost::shared_ptr<rst::vision::LocatedLaserScan> scanPtr = lidarQueue->pop();
    // Get inital odometry
    INFO_MSG("Waiting for first odometry data...");
    boost::shared_ptr<rst::geometry::Pose> odomPtr = odomQueue->pop();

    // Finally set up the particle filter
    //RayCastingModel sensorModel(&map);
    LikelihoodFieldModel<InverseDistance> sensorModel(&map);
    ParticleFilter particlefilter(sampleCount, *odomPtr, *scanPtr, &map, &sensorModel, newSampleProb, doKLDSampling, beamskip);

    /*
     * Main loop
     */
    boost::uint64_t minPeriod = 1000 / maxFrequency; // in ms
    rsc::misc::initSignalWaiter();
    bool running = true;
    bool visualizeImportance = true;
    while (rsc::misc::lastArrivedSignal() == rsc::misc::NO_SIGNAL && running) {
        boost::uint64_t loopStart = rsc::misc::currentTimeMillis();

        if (sendDebugImage) {
            INFO_MSG("Preparing visualization");
            // Visualization
            // convert to RGB
            cv::Mat3b vis(map.size(), CV_8UC3);
            cv::cvtColor(map, vis, CV_GRAY2RGB, 3);
            // resize
            int height = min(map.size().height, 700);
            float scale = (float)height / map.size().height;
            cv::resize(vis, vis, cv::Size(map.size().width * scale, map.size().height * scale));

            // find maximum importance
            sample_set_t *sampleSet = particlefilter.getSamplesSet();
            float maxImportance = 0;
            for (size_t i = 0; i < sampleSet->size; ++i) {
                maxImportance = max(maxImportance, sampleSet->samples[i].importance);
            }

            float meanX = 0.0f, meanY = 0.0f;
            float sumThetaX = 0.0f, sumThetaY = 0.0f;

            // draw all samples
            for (size_t i = 0; i < sampleSet->size; ++i) {
                sample_t sample = sampleSet->samples[i];
                // accumulate data for mean
                meanX += sample.importance * sample.pose.x;
                meanY += sample.importance * sample.pose.y;
                sumThetaX += sample.importance * cos(sample.pose.theta);
                sumThetaY += sample.importance * sin(sample.pose.theta);

                cv::Point p(sample.pose.x / meterPerPixel * scale, sample.pose.y / meterPerPixel * scale);

                int radius = 5;
                int intensity = 0;
                if (visualizeImportance) {
                    intensity = -255 / maxImportance * sample.importance + 255;
                }
                cv::circle(vis, p, radius, cv::Scalar(255,intensity,intensity));

                // also indicate theta
                cv::Point q( p.x + cos(sample.pose.theta) * radius * 1.5f, p.y + sin(sample.pose.theta) * 1.5f * radius );
                cv::line(vis, p, q, cv::Scalar(255,intensity,intensity));
            }

            // overlay mean
            float meanTheta = atan2(sumThetaY, sumThetaX);

            cv::Point p(meanX / meterPerPixel * scale, meanY / meterPerPixel * scale);

            int radius = 5;
            cv::circle(vis, p, 5, cv::Scalar(0,0,255));

            cv::Point q( p.x + cos(meanTheta) * radius * 1.5f, p.y + sin(meanTheta) * 1.5f * radius );
            cv::line(vis, p, q, cv::Scalar(0,0,255));

#ifndef __arm__
            // finally show image
            cv::flip(vis, vis, 0); // flip horizontally
            cv::imshow("visualization", vis);
            int key = cv::waitKey(100);
            switch (key) {
                case 27: // 27 == escape key
                    running = false;
                    break;

                case ' ': // space bar
                    visualizeImportance = !visualizeImportance;
                    break;
            }
#else
            std::vector<uchar> buf;
            std::vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            compression_params.push_back(100);
            cv::imencode(".jpg", vis, buf, compression_params);

            rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
            debugImageInformer->publish(frameJpg);
#endif
        }

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

        // TODO: publish position here

        boost::uint64_t loopPeriod = rsc::misc::currentTimeMillis() - loopStart;
        if (loopPeriod < minPeriod) {
            usleep((minPeriod - loopPeriod) * 1000);
        }

        INFO_MSG("==== END OF MAIN LOOP ====");
    }

    INFO_MSG("Terminating normally");
    return 0;
}
