
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <converter/matConverter/matConverter.hpp>
#include <types/twbTracking.pb.h>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

static std::string g_sServerScope = "/server";
static std::string g_sServerRequestObstacleMap = "mapObstacle";
static std::string g_sServerRequestObstacles = "getObjectsList";
static float meterPerPixel = 0.01;
static float posX = 0;
static float posY = 0;

int main(int argc, char **argv) {  

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("serverScope,s", po::value<std::string>(&g_sServerScope),"Scope sending the server request")
            ("serverRequestObstacleMap,m", po::value<std::string>(&g_sServerRequestObstacleMap),"Server request name for obstacle map")
            ("serverRequestNameObstacles,o", po::value<std::string>(&g_sServerRequestObstacles),"Server request name for obstacles")
            ("meterPerPixel,p", po::value<float>(&meterPerPixel),"Meter per pixel of the map.")
            ("posX", po::value<float>(&posX),"Position x.")
            ("posY", po::value<float>(&posY),"Position y.");

    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(options).positional(p).run(),
            vm);

    // first, process the help option
    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }
    
    // afterwards, let program options handle argument errors
    po::notify(vm);
    
    INFO_MSG( "Scope:                 " << g_sServerScope)
    INFO_MSG( "Request Obstacle Map:  " << g_sServerRequestObstacleMap)
    INFO_MSG( "Request all Obstacles: " << g_sServerRequestObstacles)


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Register our converter within the collection of converters for
  // the string wire-type (which is used for arrays of octets in
  // C++).
  shared_ptr<MatConverter> converter(new MatConverter());
  converterRepository<std::string>()->registerConverter(converter);
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
  rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);


  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create a client
  rsb::patterns::RemoteServerPtr remoteServer = factory.createRemoteServer(g_sServerScope);

  // Pop the images and show them
//  INFO_MSG( "Press ESC for exit, or any key for an request")
  INFO_MSG( "Send map request.")
  boost::shared_ptr<cv::Mat> obstacleMap = remoteServer->call<cv::Mat>(g_sServerRequestObstacleMap, 25);
  INFO_MSG( "Send obstacle request.")
  boost::shared_ptr<bool> request(new bool(false));
  boost::shared_ptr<twbTracking::proto::Pose2DList> obstacles = remoteServer->call<twbTracking::proto::Pose2DList>(g_sServerRequestObstacles, request, 25);
//  Mat grayMap;
//  cv::cvtColor(obstacleMap, grayMap, CV_GRAY2BGR);
  for(int i = 0; i < obstacles->pose_size(); ++i) {
    cv::Point2f center((obstacles->pose(i).x()+5.12)/meterPerPixel, (obstacles->pose(i).y()+5.12)/meterPerPixel);
    if ((posX+5.12-0.02)/meterPerPixel < center.x && center.x < (posX+5.12+0.02)/meterPerPixel
        && (posY+5.12-0.02)/meterPerPixel < center.y && center.y < (posY+5.12+0.02)/meterPerPixel) {
      cv::circle(*obstacleMap, center, obstacles->pose(i).orientation()/meterPerPixel, cv::Scalar(0,0,255),1);
    } else {
      cv::circle(*obstacleMap, center, obstacles->pose(i).orientation()/meterPerPixel, cv::Scalar(0,0,64),1);
    }
  }
  cv::imshow("input", *obstacleMap);
  // Exit the program if any key was pressed
  cv::waitKey(0);


  return 0;

}
