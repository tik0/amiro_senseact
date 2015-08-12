
#define INFO_MSG_
//#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
 #define ERROR_MSG_
#include <MSG.h>

#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// mrpt
#include <mrpt/maps/CMultiMetricMap.h>
//#include <mrpt/maps.h>
#include <mrpt/opengl.h>
#include <mrpt/gui.h>

// Converting helpers
#include <Eigen/Geometry>

static double transX, transY, transZ;
static double rotX, rotY, rotZ;

inline Eigen::Quaterniond
euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// Marging for non present information: 0.45 .. 0.55 -> 0.1
static const float uncertaintyMargin = 0.1f;

#define NUM_MAPS 9
enum maps {
  stockDensity1,
  stockDensity2,
  stockDensity3,
  obstacleNonProcNonAcc,
  obstacleProcNonAcc,
  obstacleNonProcAcc,
  stockFlattened,
  stockEdge,
  processed  // Overwrites all other properties for visualization and other reasoning
};

std::string mapStrings[NUM_MAPS] {
  "stockDensity1",
  "stockDensity2",
  "stockDensity3",
  "obstacleNonProcNonAcc",
  "obstacleProcNonAcc",
  "obstacleNonProcAcc",
  "stockFlattened",
  "stockEdge",
  "processed"
};

unsigned char mapColorBGR[NUM_MAPS][3] {
  {255,   0,   0}, // Light blue
  {200,   0,   0}, // Mid blue
  {127,   0,   0}, // Dark blue
  {  0,   0, 255}, // Dark red
  { 42,  42, 165}, // Brown
  { 30, 105, 210}, // Chocolat
  {200,   0, 200}, // Purple
  {  0, 255, 255}, // Yellow
  {  0, 255,   0} // Green
};


// The mapt stack
static std::vector<mrpt::maps::COccupancyGridMap2D> mapStack;
static mrpt::maps::COccupancyGridMap2D::TMapDefinition def;

// Convinience
static bool sendMapAsCompressedImage = false;

// RSB
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
//#include <rst0.11/stable/rst/vision/LaserScan.pb.h>

// RSC
#include <rsc/misc/SignalWaiter.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // resize


// For sending the localization
// rsb::Informer<rst::geometry::PoseEuler>::Ptr informerOdometry;

// Actual position
// rsb::Informer<rst::geometry::PoseEuler>::DataPtr odomData(new rst::geometry::PoseEuler);

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;

#include <mutex>          // std::mutex
std::mutex map_to_odom_mutex_;
std::mutex mtxOdom;       // mutex for odometry messages
static double pathObstacleErosion = 0.1; // m
static double detectionObstacleErosion = 0.1; // m


// RSB Server function for the mapping server which replies with a 8bit map of the environment
class mapServer: public rsb::patterns::LocalServer::Callback<void, cv::Mat> {
public:
  boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
    const int sizeX = (def.max_x - def.min_x) / def.resolution;
    const int sizeY = (def.max_y - def.min_y) / def.resolution;

    boost::shared_ptr<cv::Mat> dst(new cv::Mat(sizeX, sizeY, CV_8UC3));
    dst->setTo(cv::Scalar(255,255,255)); // to set all values to 255

    const float validCellValue = 0.5 + uncertaintyMargin;
    for (int mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
      for (int idy = 0; idy < sizeY; ++idy) {
        for (int idx = 0; idx < sizeX; ++idx) {
          if (mapStack[mapIdx].getCell(idx, idy) > validCellValue) {
            dst->at<cv::Vec3b>(idy,idx)[0] = mapColorBGR[mapIdx][0];
            dst->at<cv::Vec3b>(idy,idx)[1] = mapColorBGR[mapIdx][1];
            dst->at<cv::Vec3b>(idy,idx)[2] = mapColorBGR[mapIdx][2];
          }
        }
      }
    }

    INFO_MSG("Server returns map")
    cv::flip(*dst, *dst, 0);  // horizontal flip
    return dst;
  }
};

boost::shared_ptr<cv::Mat> testFunctionForMapServer() {
  const int sizeX = (def.max_x - def.min_x) / def.resolution;
  const int sizeY = (def.max_y - def.min_y) / def.resolution;

  boost::shared_ptr<cv::Mat> dst(new cv::Mat(sizeX, sizeY, CV_8UC3));
  dst->setTo(cv::Scalar(255,255,255)); // to set all values to 255

  const float validCellValue = 0.5 + uncertaintyMargin;
  INFO_MSG("1")
  for (int mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    INFO_MSG("2")
    for (int idy = 0; idy < sizeY; ++idy) {
      for (int idx = 0; idx < sizeX; ++idx) {
        if (mapStack[mapIdx].getCell(idx, idy) > validCellValue) {
          dst->at<cv::Vec3b>(idy,idx)[0] = mapColorBGR[mapIdx][0];
          dst->at<cv::Vec3b>(idy,idx)[1] = mapColorBGR[mapIdx][1];
          dst->at<cv::Vec3b>(idy,idx)[2] = mapColorBGR[mapIdx][2];
        }
      }
    }
  }

  INFO_MSG("Server returns map")
  cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
};

int main(int argc, const char **argv){
  rsc::misc::initSignalWaiter();
  // Handle program options
  namespace po = boost::program_options;
  
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
//  std::string odomInScope = "/AMiRo_Hokuyo/gps";
  std::string localizationOutScope = "/localization";
  std::string serverScope = "/AMiRo_Hokuyo/server/slam";
  std::string mapAsImageOutScope = "/AMiRo_Hokuyo/image";
  std::string sExplorationScope = "/exploration";
  std::string sExplorationCmdScope = "/command";
  std::string sExplorationAnswerScope = "/answer";
  std::string sPathInputScope = "/pathReq/request";
  std::string sPathOutputScope = "/pathReq/answer";
  std::string sObjectsInputScope = "/objectsReq/request";
  std::string sObjectsOutputScope = "/objectsReq/answer";
  std::string sPushPathInputScope = "/pushPathServer/request";
  std::string sPushPathOutputScope = "/pushPathServer/answer";
	std::string insertObjectInscope = "/mapGenerator/insertObject";
	std::string deleteObjectInscope = "/mapGenerator/deleteObject";
  std::string pathServerReq = "path";
  std::string pushingPathServerReq = "getPushingPath";
  std::string mapServerReq = "map";
  std::string mapServerObstacleReq = "mapObstacle";
  std::string obstacleServerReq = "getObjectsList";
  std::string remoteHost = "localhost";
  std::string remotePort = "4803";
  int robotID = 0;

//  mrpt::maps::TSetOfMetricMapInitializers map_inits;
  // Define an occupancy map
  def.resolution = 0.1f;
  def.insertionOpts.maxOccupancyUpdateCertainty = 0.8f;
  def.insertionOpts.maxDistanceInsertion = 30.0f;
  def.max_x = 30.0f;
  def.min_x = 0.0f;
  def.max_y = 30.0f;
  def.min_y = 0.0f;

  // Create the maps
  for (unsigned char mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx)
    mapStack.push_back(*mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def));
{
  // Update a sensor scan:
  // The transformation already points to the lower left edge of the sensor scan
  transX = 2.0f;
  transY = 5.0f;

  static mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
  def.max_x = 4.0f;
  def.min_x = 0.0f;
  def.max_y = 3.0f;
  def.min_y = 0.0f;
  mrpt::maps::COccupancyGridMap2D sensorUpdate = *mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def);
  mapStack[0].fill (0.3);
  sensorUpdate.fill (0.8);

  int mapOffsetXStart = transX / def.resolution;
    int mapOffsetXEnd   = (transX + def.max_x - def.min_x) / def.resolution;
  int mapOffsetYStart = transY / def.resolution;
  int mapOffsetYEnd   = (transY + def.max_y - def.min_y) / def.resolution;

  for (int mapIdx = 0; mapIdx < 9 ; ++mapIdx) {
    int mapOffsetXStart = transX / def.resolution * (1 + mapIdx);
    int mapOffsetXEnd   = (transX + def.max_x - def.min_x) / def.resolution * (1 + mapIdx);
    std::cout << "mapOffsetYStart " << mapOffsetYStart << std::endl;
      std::cout << "mapOffsetYEnd " << mapOffsetYEnd << std::endl;
  for (int idy = mapOffsetYStart; idy < mapOffsetYEnd ; ++idy) {
    for (int idx = mapOffsetXStart; idx < mapOffsetXEnd ; ++idx) {
      mapStack[mapIdx].updateCell(idx,idy,sensorUpdate.getCell(idx-mapOffsetXStart, idy-mapOffsetYStart));
    }
  }
}
}

//  for (float idx = 0; idx < 5.0f; idx = idx + 0.05f)
//    for (float idy = 0; idy < 5.0f; idy = idy + 0.05f)
//      mapStack[0].setPos (idx, idy, 1.0f);

  boost::shared_ptr<cv::Mat> image(testFunctionForMapServer());

//    mrpt::utils::CImage mapImg;
//    mapStack[0].getAsImage( mapImg );
//    cv::Mat image = cv::Mat(mapImg.getAs<IplImage>());
  imshow( "Display window", *image );                   // Show our image inside it.
  cv::waitKey(0);                                      // Wait for a keystroke in the window




//  mrpt::opengl::CSetOfObjectsPtr outObj;
//  theMap.getAs3DObject(outObj);
//  mrpt::gui::CDisplayWindow3D win("My window");
//  // Adquire the scene:
//  mrpt::opengl::COpenGLScenePtr &ptrScene = win.get3DSceneAndLock();
//  // Modify the scene:
//  mrpt::opengl::CRenderizablePtr renderizablePtr;
//  outObj.CRenderizablePtr(&renderizablePtr);
//  ptrScene->insert(renderizablePtr, std::string("main"));
//  // Unlock it, so the window can use it for redraw:
//  win.unlockAccess3DScene();
//  // Update window, if required
//  win.forceRepaint();
/*
  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("robotID", po::value <int> (&robotID), "ID of robot for communication (default=0)")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data")
//    ("odominscope", po::value < std::string > (&odomInScope), "Scope for receiving odometry data")
    ("localizationOutScope", po::value < std::string > (&localizationOutScope), "Scope sending the odometry data")
    ("serverScope", po::value < std::string > (&serverScope), "Scope for handling server requests")
    ("mapServerReq", po::value < std::string > (&mapServerReq), "Map server request string (Std.: map)")
    ("mapServerObstacleReq", po::value < std::string > (&mapServerObstacleReq), "Map server obstacle request string (Std.: mapObstacle)")
    ("pathServerReq", po::value < std::string > (&pathServerReq), "Path server request string (Std.: path)")
    ("obstacleServerReq", po::value < std::string > (&obstacleServerReq), "Obstacle server request string (Std.: getObjectsList)")
    ("remoteHost", po::value < std::string > (&remoteHost), "Remote spread daemon host name")
    ("remotePort", po::value < std::string > (&remotePort), "Remote spread daemon port")
    ("senImage", po::value < bool > (&sendMapAsCompressedImage), "Send map as compressed image")
    ("pathObstacleErosion", po::value < double > (&pathObstacleErosion), "Radius in meter of the erosion of objects for the path planer")
    ("detectionObstacleErosion", po::value < double > (&detectionObstacleErosion), "Radius in meter of the erosion of objects for the tabletop obstacle detection")
    ("mapAsImageOutScope", po::value < std::string > (&mapAsImageOutScope), "Scope for sending the map as compressed image to a remote spread daemon")
    ("sigma_xy", po::value < double > (&sigma_xy_), "XY uncertainty for marcov localization [m]")
    ("sigma_theta", po::value < double > (&sigma_theta_), "Theta uncertainty for marcov localization [m]")
    ("throttle_scans", po::value < int > (&throttle_scans_), "Only take every n'th scan")
    ("samples", po::value < int > (&samples), "Sampling steps of the marcov localization sampler")
    ("hole_width", po::value < double > (&hole_width_), "Width of impacting rays [m]")
    ("delta", po::value < double > (&delta_), "Resolution [m/pixel]")
    ("rayPruningAngleDegree", po::value < float > (&rayPruningAngleDegree), "Pruning of adjiacent rays if they differ to much on the impacting surface [0° .. 90°]")
    ("transX", po::value < double > (&transX),"Translation of the lidar in x [m]")
    ("transY", po::value < double > (&transY),"Translation of the lidar in y [m]")
    ("transZ", po::value < double > (&transZ),"Translation of the lidar in z [m]")
    ("rotX", po::value < double > (&rotX),"Rotation of the lidar around x (roll) [rad]")
    ("rotY", po::value < double > (&rotY),"Rotation of the lidar around y (pitch) [rad]")
    ("rotZ", po::value < double > (&rotZ),"Rotation of the lidar around z (yaw) [rad]");

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
  if (vm.count("robotID")) {
    serverScope.append(std::to_string(robotID));
  }


  // Get the RSB factory

#if RSB_VERSION_NUMERIC<1200
  rsb::Factory& factory = rsb::Factory::getInstance();
#else
  rsb::Factory& factory = rsb::getFactory();
#endif
  
  //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
  ///////////////////////////////////////////////////////////////////////////////
  // Get the global participant config as a template
  rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
        {
          // disable socket transport
          rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();
          tmpPropSocket["enabled"] = boost::any(std::string("0"));

          // Get the options for spread transport, because we want to change them
          rsc::runtime::Properties tmpPropSpread  = tmpPartConf.mutableTransport("spread").getOptions();

          // enable socket transport
          tmpPropSpread["enabled"] = boost::any(std::string("1"));

          // Change the config
          tmpPropSpread["host"] = boost::any(std::string(remoteHost));

          // Change the Port
          tmpPropSpread["port"] = boost::any(std::string(remotePort));

          // Write the tranport properties back to the participant config
          tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
          tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
        }
  ///////////////////////////////////////////////////////////////////////////////
  
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler > > odomEulerConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomEulerConverter);
  boost::shared_ptr<muroxConverter::MatConverter> matConverter(new muroxConverter::MatConverter());
  rsb::converter::converterRepository<std::string>()->registerConverter(matConverter);
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
  rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
  rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
  // Prepare RSB listener for incomming path request
  rsb::ListenerPtr pathListener = factory.createListener(sPathInputScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>pathQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
  pathListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(pathQueue)));
  // Prepare RSB listener for incomming objects request
  rsb::ListenerPtr objectsListener = factory.createListener(sObjectsInputScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>objectsQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  objectsListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(objectsQueue)));
  // Prepare RSB listener for incomming pushing path request
  rsb::ListenerPtr pushPathListener = factory.createListener(sPushPathInputScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>pushPathQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
  pushPathListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(pushPathQueue)));
  // Prepare RSB informer for sending the path answer
  rsb::Informer<std::string>::Ptr pathInformer = factory.createInformer<std::string> (sPathOutputScope);
  // Prepare RSB informer for sending the objects answer
  rsb::Informer<std::string>::Ptr objectsInformer = factory.createInformer<std::string> (sObjectsOutputScope);
  // Prepare RSB informer for sending the pushing path answer
  rsb::Informer<std::string>::Ptr pushPathInformer = factory.createInformer<std::string> (sPushPathOutputScope);
  // Prepare RSB listener for controling the programs behaviour
  rsb::ListenerPtr expListener = factory.createListener(sExplorationScope);
  expListener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&controlCoreSLAMBehaviour)));
  // Prepare RSB informer for sending the map as an compressed image
  rsb::Informer<std::string>::Ptr informer = factory.createInformer<std::string> (mapAsImageOutScope, tmpPartConf);
  // Prepare RSB informer for sending the map as an compressed image
  informerOdometry = factory.createInformer<rst::geometry::PoseEuler> (localizationOutScope);

	// prepare RSB listener for commands to insert an object in the map
	rsb::ListenerPtr insertObjectListener = factory.createListener(insertObjectInscope);
	insertObjectListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<twbTracking::proto::Pose2D>(&insertObject)));

	// prepare RSB listener for commands to delete an object from the map
	rsb::ListenerPtr deleteObjectListener = factory.createListener(deleteObjectInscope);
	deleteObjectListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<twbTracking::proto::Pose2D>(&deleteObject)));
*/

  // As events are received asynchronously we have to wait here for
  // them.
  return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());
//  return 0;
}

