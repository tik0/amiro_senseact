
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include "MSG.h"

#include <math.h>
#include <utils.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// for reading and writing maps
#include <fstream>

// tinySLAM
#ifdef __cplusplus
extern "C"{
#endif
#include "CoreSLAM.h"
#ifdef __cplusplus
}
#endif
// Parameters needed for Marcov sampling
static double sigma_xy_ = 0.01;  // m
static double sigma_theta_ = 0.35;  // rad
static int samples = 100; // Number of resampling steps
// parameters for coreslam
static double hole_width_ = 0.1;  // m
static double delta_ = 0.02;  // Meter per pixel
static ts_map_t ts_map_;
static ts_state_t state_;
static ts_position_t position_;
static ts_position_t prev_odom_;
static ts_laser_parameters_t lparams_;
#define METERS_TO_MM    1000
#define MM_TO_METERS    0.001
static bool got_first_scan_ = false;
//static bool got_map_ = false;
static int laser_count_ = 0;
static int throttle_scans_ = 1;
// Check "http://de.wikipedia.org/wiki/Sinussatz"!
// c ist the first ray, b the second. If beta is 90°, it means that c is hitting a surface very perpendiular.
// Every deviation of the 90° is an incident, which means that the surface is not perpendicular to the ray.
// The value rayPruningAngleDegree gives the maximal allowed deviation from 90 degrees.
// Every deviation above that angle results in a pruning of the ray c
static float rayPruningAngleDegree = 60; /* [0 .. 90] */
float rayPruningAngle(){return asin((90 - rayPruningAngleDegree) / 180 * M_PI);}
static double mapOffset = 0;

// Switch for localization
static bool doMapUpdate = false;

// Converting helpers
#include <Eigen/Geometry>

static double transX, transY, transZ;
static double rotX, rotY, rotZ;

// Convinience
static bool sendMapAsCompressedImage = false;

// Paths to read map from
static std::string mapImagePath = "";
static std::string mapPGMPath = "";

// RSB
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
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
#include <types/LocatedLaserScan.pb.h>
//#include <rst/geometry/Pose.pb.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // resize

#include "pathplanner.h"
#include <iterator>
// init path planner
PathPlanner pathplanner = PathPlanner();

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;

#include <mutex>          // std::mutex
std::mutex map_to_odom_mutex_;
std::mutex mtxOdom;       // mutex for odometry messages
static rst::geometry::Translation odomTrans;
static rst::geometry::Rotation odomRot;

ts_position_t pose;
ts_position_t odom_pose;

cv::Mat getOccupancyMap() {
  // Convert the map to a cv::Mat image
  cv::Mat map(cv::Size(ts_map_.size,ts_map_.size), CV_8UC1);
  cv::Mat tmp = cv::Mat(ts_map_.size, ts_map_.size, CV_16UC1, static_cast<void*>(&ts_map_.map[0]));
  tmp.convertTo(map, CV_8UC1, 255.0f / TS_NO_OBSTACLE);  // Convert to 8bit depth image

  // Threshold image
  for (ssize_t idx = 0; idx < map.rows * map.cols; ++idx)
      map.at<uint8_t>(idx) = (map.at<uint8_t>(idx) > 126) ? 255 : 0; // note: 126 due to rounding errors

  // Erode w.r.t. robot size
  float robotRadiusInMeter = 0.05;
  cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                            cv::Size(2*robotRadiusInMeter/delta_ + 1, 2*robotRadiusInMeter/delta_ + 1),
                            cv::Point(robotRadiusInMeter/delta_, robotRadiusInMeter/delta_));
  cv::erode(map, map, structuringElement);

  return map;
}

std::list<cv::Point2i> getPath(ts_position_t &targetPose) {
    // Convert from tinySLAM coordinates (mm) to pixel coordinates
    cv::Point2i start(pose.x * MM_TO_METERS / delta_, pose.y * MM_TO_METERS / delta_);
    cv::Point2i goal(targetPose.x * MM_TO_METERS / delta_, targetPose.y * MM_TO_METERS / delta_);
    // Generate occupacy map
    cv::Mat1b om = getOccupancyMap();
    // Get path
    std::list<cv::Point2i> path = pathplanner.getPath(start, goal, om);

    // DEBUG START
//#ifndef NDEBUG
    cv::Mat clr(ts_map_.size, ts_map_.size, CV_8UC3);
    cv::cvtColor(om, clr, cv::COLOR_GRAY2RGB, 3);

    cv::circle(clr, start, 5, cv::Scalar(255,0,0), 2);
    cv::circle(clr, goal, 5, cv::Scalar(0,255,0), 2);

    for (auto r : path) {
        clr.at<cv::Vec3b>(r) = cv::Vec3b(0,0,255);
    }

    imwrite("clr.png", clr);
//#endif
    // DEBUG END

//#ifndef NDEBUG
    // Optimize path
    pathplanner.removeRedundantNodes(path);
    pathplanner.optimizePath(path, om);

    // DEBUG START
    for (auto r : path) {
        clr.at<cv::Vec3b>(r) = cv::Vec3b(0,255,0);
    }
    imwrite("opt.png", clr);
    // DEBUG END
//#endif

    // TODO: convert to coordinates to outer space?
    DEBUG_MSG("calculated path")
    return path;
}

int convertDataToScan(boost::shared_ptr< rst::vision::LocatedLaserScan > data , rst::vision::LocatedLaserScan &rsbScan) {
  
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return 1;

//  DEBUG_MSG( "Scan rec.")
//  Eigen::Quaterniond lidar_quat(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
//  Eigen::AngleAxisd lidar_angle(lidar_quat);
//  Eigen::Matrix<double,3,1> rpy = lidar_angle.toRotationMatrix().eulerAngles(0,1,2);
//  const double yaw = rpy(2);

  // Copy the whole scan
  rsbScan = *data;

  // Shifting the rays
//  DEBUG_MSG("Startscan: " << rsbScan.scan_angle_start() * 180.0 / M_PI)
//  DEBUG_MSG("Endscan: " << rsbScan.scan_angle_end() * 180.0 / M_PI)
//  DEBUG_MSG("Angle: " << rsbScan.scan_angle() * 180.0 / M_PI)
//  DEBUG_MSG("Size: " << rsbScan.scan_values_size())
//  DEBUG_MSG("Inc: " << rsbScan.scan_angle_increment() * 180.0 / M_PI)
//  rsbScan.set_scan_angle_increment(-rsbScan.scan_angle_increment);

  return 0;
}

void storeOdomData(boost::shared_ptr<rst::geometry::Pose> event) {
  mtxOdom.lock();
    odomTrans = event->translation();
    odomRot = event->rotation();
  mtxOdom.unlock();
}

bool
getOdomPose(ts_position_t& ts_pose)
{
  rst::geometry::Translation translation;
  rst::geometry::Rotation rotation;
  mtxOdom.lock();
    translation = odomTrans;
    rotation = odomRot;
  mtxOdom.unlock();
  
  // Convert from quaternion to euler
  Eigen::Quaterniond lidar_quat(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
  Eigen::Matrix<double,3,1> rpy;
  conversion::quaternion2euler(&lidar_quat, &rpy);
  const double yaw = rpy(2);

  DEBUG_MSG( "CoreSLAM(RPY): " <<  rpy(0) << ", "<< rpy(1) << ", "<< rpy(2))
  DEBUG_MSG( "CoreSLAM(WXYZ): " <<  rotation.qw() << ", "<< rotation.qx() << ", "<< rotation.qy() << ", " << rotation.qz())

  ts_pose.x = translation.x()*METERS_TO_MM + ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM); // convert to mm
  ts_pose.y = translation.y()*METERS_TO_MM + ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM); // convert to mm
  ts_pose.theta = (yaw * 180/M_PI);

  DEBUG_MSG( "-------------------------------------------------------------------------------------------" )
  DEBUG_MSG( "Odometry: x(m): " <<  translation.x() << " y(m): " << translation.y() << " theta(rad): " << yaw)
  DEBUG_MSG( "Odometry map-centered: x(mm):" << ts_pose.x << " y(mm): " << ts_pose.y << " theta(deg): " << ts_pose.theta)

  return true;
}

bool
initMapper(const rst::vision::LocatedLaserScan& scan)
{

  // configure previous_odom
  if(!getOdomPose(prev_odom_))
     return false;
  position_ = prev_odom_;

  // configure laser parameters
  lparams_.offset = 0.0;  // No offset of the lidar base
  lparams_.scan_size = scan.scan_values_size();
  lparams_.angle_min = scan.scan_angle_start()  * 180/M_PI;
  lparams_.angle_max = scan.scan_angle_end()  * 180/M_PI;
  lparams_.detection_margin = 0;
  lparams_.distance_no_detection = scan.scan_values_max() * METERS_TO_MM;

  // new coreslam instance
  ts_state_init(&state_, &ts_map_, &lparams_, &position_, sigma_xy_, sigma_theta_*180/M_PI , (int)(hole_width_*1000), 0, samples);

  INFO_MSG("Initialized with sigma_xy=" << sigma_xy_<< ", sigma_theta=" << sigma_theta_ << ", hole_width=" << hole_width_ << ", delta=" << delta_);
  INFO_MSG("Initialization complete");
  return true;
}

void convertToScan(const rst::vision::LocatedLaserScan &scan , ts_scan_t &ranges) {
  ranges.nb_points = 0;
  const float delta_angle = scan.scan_angle_increment();

    for(int i=0; i < lparams_.scan_size; i++) {
      // Must filter out short readings, because the mapper won't
      if(scan.scan_values(i) > scan.scan_values_min() && scan.scan_values(i) < scan.scan_values_max()){
        // HACK "+ 120" is a workaround, and works only for startStep 44 and enStep 725!!!!
        ranges.x[ranges.nb_points] = cos((lparams_.angle_min + 120 )* M_PI/180.0f + i*delta_angle ) * (scan.scan_values(i)*METERS_TO_MM);
        ranges.y[ranges.nb_points] = sin((lparams_.angle_min + 120) * M_PI/180.0f + i*delta_angle) * (scan.scan_values(i)*METERS_TO_MM);
        ranges.value[ranges.nb_points] = TS_OBSTACLE;
        ranges.nb_points++;
      }
    }

}

bool addScan(const rst::vision::LocatedLaserScan &scan, ts_position_t &pose)
{
  // update odometry
  ts_position_t odom_pose;
  if(!getOdomPose(odom_pose))
     return false;
  state_.position.x += odom_pose.x - prev_odom_.x;
  state_.position.y += odom_pose.y - prev_odom_.y;
  state_.position.theta += odom_pose.theta - prev_odom_.theta;
  prev_odom_ = odom_pose;

  ts_position_t prev = state_.position;

  // Do marcov localization on the map (this is done already in ts_iterative_map_building, but we can already do here for debugging)
//  ts_scan_t ranges;
//  convertToScan(scan , ranges);
//  INFO_MSG( "Pose1st " << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
//  ts_monte_carlo_search(&state_.randomizer, &ranges, &ts_map_, &state_.position, state_.sigma_xy, state_.sigma_theta, -10000, NULL);
//  INFO_MSG( "Pose2st " << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
//  INFO_MSG( "PoseOdom " << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta)

  ts_sensor_data_t data;
  data.position[0] = state_.position;
  if(lparams_.angle_max < lparams_.angle_min){
    // flip readings
    for(int i=0; i < scan.scan_values_size(); i++)
      data.d[i] = (int) (scan.scan_values(scan.scan_values_size()-1-i)*METERS_TO_MM);
  }else{
    for(int i=0; i < scan.scan_values_size(); i++)
      data.d[i] = (int) (scan.scan_values(i)*METERS_TO_MM);
  }

  // Mapping
  if(laser_count_ < 10){
    WARNING_MSG("BOOTSTRAP")
  // not much of a map, let's bootstrap for now
    ts_scan_t ranges;
    ts_build_scan(&data, &ranges, &state_, 3 /*widening of the ray*/);
    if (doMapUpdate) {
        ts_map_update(&ranges, &ts_map_, &state_.position, 50, (int)(hole_width_*1000));
    }
    DEBUG_MSG("Update step, " << laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
  }else{

    // Monte carlo localization is done inside
    ts_iterative_map_building(&data, &state_, doMapUpdate);

    DEBUG_MSG("Iterative step, "<< laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
    DEBUG_MSG("Correction: "<< state_.position.x - prev.x << ", " << state_.position.y - prev.y << ", " << state_.position.theta - prev.theta)
  }
  // Set the new pose
  pose = state_.position;

  return true;
}

/**
 * @brief saveMapAsPGM Saves a given map as a 16bit PGM.
 * @param map pointer to tinySLAM map struct.
 * @param path path in the file system where the image should be saved.
 */
void saveMapAsPGM(ts_map_t *map, std::string path) {
    std::ofstream file;
    file.open(path);

    // Magic Number for Portable Graymap in ASCII
    file << "P2" << std::endl;

    // Dimensions of the PGM
    file << map->size << " " << map->size << std::endl;

    // Image depth
    file << TS_NO_OBSTACLE << std::endl;

    // Actual image data
    int x, y;
    ts_map_pixel_t *ptr;
    for (ptr = map->map, y = 0; y < map->size; y++) {
        for (x = 0; x < map->size; x++, ptr++) {
            file << *ptr << " ";
        }
        file << std::endl;
    }

    file.close();
}

/**
 * @brief loadMapFromPGM loads a 16bit grayscale PGM and converts it into a tinySLAM map struct. This function can be used to load PGMs created with saveMapAsPGM(). In contrast to loadMapFromImage() PGM are loaded with 16bit depth.
 * @param map pointer to a tinySLAM map struct.
 * @param path path in the file system to the PGM file.
 * @return true on success, otherwise false
 */
bool loadMapFromPGM(ts_map_t *map, std::string path) {
    // read image from file
    cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_UNCHANGED); // load unchanged to keep 16bit

    // check if loading image failed
    if (!image.data) {
        ERROR_MSG("Could not open file: " << path);
        return false;
    }

    // check if image is square
    if (image.cols != image.rows) {
        ERROR_MSG("Height of image must be equal to width!");
        return false;
    }

    // initalize struct
    map->size = image.cols;
    map->map = (ts_map_pixel_t *) malloc(sizeof(ts_map_pixel_t) * map->size * map->size);

    // copy image into map structure
    int x, y;
    ts_map_pixel_t *ptr = map->map;
    for (y = 0; y < image.rows; y++) {
        for (x = 0; x < image.cols; x++) {
            *ptr = image.at<uint16_t>(y,x);
            ptr++;
        }
    }

    return true;
}

/**
 * @brief loadMapFromImage loads a grayscale image using OpenCV and converts it into a tinySLAM map struct.
 * @param map pointer to a tinySLAM map struct.
 * @param path path in the file system to the image file.
 * @return true on success, otherwise false
 */
bool loadMapFromImage(ts_map_t *map, std::string path) {
    // read image from file
    cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);

    // check if loading image failed
    if (!image.data) {
        ERROR_MSG("Could not open file: " << path);
        return false;
    }

    // check if image is square
    if (image.cols != image.rows) {
        ERROR_MSG("Height of image must be equal to width!");
        return false;
    }

    // initalize struct
    map->size = image.cols;
    map->map = (ts_map_pixel_t *) malloc(sizeof(ts_map_pixel_t) * map->size * map->size);

    // convert image to map
    int x, y;
    ts_map_pixel_t *ptr = map->map;
    for (y = 0; y < image.rows; y++) {
        for (x = 0; x < image.cols; x++) {
            *ptr = image.at<uchar>(y,x) * (TS_NO_OBSTACLE / 255);
            ptr++;
        }
    }

    return true;
}

int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
  std::string odomInScope = "/AMiRo_Hokuyo/gps";
  std::string localizationOutScope = "/localization";
  std::string serverScope = "/AMiRo_Hokuyo/server/slam";
  std::string mapAsImageOutScope = "/AMiRo_Hokuyo/image";
  std::string sExplorationScope = "/exploration";
  std::string sExplorationCmdScope = "/command";
  std::string sExplorationAnswerScope = "/answer";
  std::string sPathInputScope = "/path/request";
  std::string sPathOutputScope = "/path/answer";
  std::string pathServerReq = "path";
  std::string mapServerReq = "map";
  std::string mapServerObstacleReq = "mapObstacle";
  std::string obstacleServerReq = "getObjectsList";
  std::string remoteHost = "localhost";
  std::string remotePort = "4803";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data")
    ("odominscope", po::value < std::string > (&odomInScope), "Scope for receiving odometry data")
    ("localizationOutScope", po::value < std::string > (&localizationOutScope), "Scope sending the odometry data")
    ("serverScope", po::value < std::string > (&serverScope), "Scope for handling server requests")
    ("mapServerReq", po::value < std::string > (&mapServerReq), "Map server request string (Std.: map)")
    ("mapServerObstacleReq", po::value < std::string > (&mapServerObstacleReq), "Map server obstacle request string (Std.: mapObstacle)")
    ("pathServerReq", po::value < std::string > (&pathServerReq), "Path server request string (Std.: path)")
    ("obstacleServerReq", po::value < std::string > (&obstacleServerReq), "Obstacle server request string (Std.: getObjectsList)")
    ("remoteHost", po::value < std::string > (&remoteHost), "Remote spread daemon host name")
    ("remotePort", po::value < std::string > (&remotePort), "Remote spread daemon port")
    ("senImage", po::value < bool > (&sendMapAsCompressedImage), "Send map as compressed image")
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
    ("rotZ", po::value < double > (&rotZ),"Rotation of the lidar around z (yaw) [rad]")
    ("loadMapFromImage", po::value < std::string > (&mapImagePath),"Load map from image file")
    ("loadMapFromPGM", po::value < std::string > (&mapPGMPath),"Load map from *16bit* PGM file")
    ("doMapUpdate", po::value < bool > (&doMapUpdate),"Update the map (false = only localization, default = true)");

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

  // tinySLAM init
  ts_map_set_scale(MM_TO_METERS/delta_);  // Set TS_MAP_SCALE at runtime
  mapOffset = ((ts_map_.size/2)*delta_*METERS_TO_MM);

  // initalize map
  if (!mapImagePath.empty()) {
      INFO_MSG("Reading map from file.");
      loadMapFromImage(&ts_map_, mapImagePath);
  } else if (!mapPGMPath.empty()) {
      INFO_MSG("Reading map from PGM.");
      loadMapFromPGM(&ts_map_, mapPGMPath);
  } else {
      ts_map_init(&ts_map_);
  }

  rsb::Factory& factory = rsb::getFactory();
  
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


  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
  // Prepare RSB async listener for odometry messages
  rsb::ListenerPtr listener = factory.createListener(odomInScope);
  listener->addHandler(HandlerPtr(new DataFunctionHandler<rst::geometry::Pose> (&storeOdomData)));
  // Prepare RSB informer for sending the map as an compressed image
  rsb::Informer<std::string>::Ptr informer = factory.createInformer<std::string> ("/image", tmpPartConf);

  // Show the map as a cv Image
  cv::Size size(ts_map_.size / 2,ts_map_.size / 2);
  cv::Mat dst(size, CV_16S); // destination image for scaling
  cv::Mat dstColor(size, CV_8UC3); // Color image

  rst::vision::LocatedLaserScan scan;

  // path to goal
  ts_position_t targetPose = { 8237.05, 13279.9, 0.236681 };
  std::list<cv::Point2i> path;

  while( true ){
    // Fetch a new scan and store it to scan
    if (convertDataToScan(lidarQueue->pop(), scan))
      continue;
    getOdomPose(odom_pose);
    // We can't initialize CoreSLAM until we've got the first scan
    if(!got_first_scan_)
    {
      if(!initMapper(scan))
        continue;
      got_first_scan_ = true;
    } else {

      ///////////////////////////////////////////////
      if(addScan(scan, pose))
      {
        DEBUG_MSG("scan processed.");

        if (path.empty()) {
            DEBUG_MSG("Searching path to goal...")
            path = getPath(targetPose);
        }
      }

    }

    if (sendMapAsCompressedImage) {

      // Convert to RGB image
      cv::Mat image = cv::Mat(ts_map_.size, ts_map_.size, CV_16U, static_cast<void*>(&ts_map_.map[0]));
      cv::resize(image,dst,size);//resize image
      dst.convertTo(dst, CV_8U, 0.00390625);  // Convert to 8bit depth image
      cv::cvtColor(dst, dstColor, cv::COLOR_GRAY2RGB, 3);  // Convert to color image

      // Draw tinySLAM position
      cv::Point robotPosition(pose.x * MM_TO_METERS / delta_ * size.width / ts_map_.size,
                              pose.y * MM_TO_METERS / delta_ * size.height / ts_map_.size);  // Draw MCMC position
      cv::circle( dstColor, robotPosition, 0, cv::Scalar( 0, 0, pow(2,8)-1), 10, 8 );

      // Draw odom position
      cv::Point robotOdomPosition(odom_pose.x * MM_TO_METERS / delta_ * size.width / ts_map_.size,
                                  odom_pose.y * MM_TO_METERS / delta_ * size.height / ts_map_.size);  // Draw odometry
      cv::circle( dstColor, robotOdomPosition, 0, cv::Scalar( 0, pow(2,8)-1), 0, 10, 8 );

      // Draw tinySLAM position
      cv::Point targetPosition(targetPose.x * MM_TO_METERS / delta_ * size.width / ts_map_.size,
                               targetPose.y * MM_TO_METERS / delta_ * size.height / ts_map_.size);
      cv::circle( dstColor, targetPosition, 0, cv::Scalar(pow(2,8)-1), 10, 8 );

      // Draw waypoints
      if (!path.empty()) {
          for (auto p = path.begin(); p != std::prev(path.end()); p++) {
              cv::Point q(p->x * size.width / ts_map_.size, p->y * size.height / ts_map_.size);
              auto r = std::next(p);
              cv::Point s(r->x * size.width / ts_map_.size, r->y * size.height / ts_map_.size);
              cv::line(dstColor, q, s, cv::Scalar(255,0,0));
          }
      }

      cv::flip(dstColor, dstColor, 0);  // horizontal flip
      DEBUG_MSG( "---------------------------------")
      DEBUG_MSG( "Pose TinySLAM: " << pose.x << ", " << pose.y << ", " << pose.theta)
      DEBUG_MSG( "Pose Odometry: " << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta)
      DEBUG_MSG( "Target pose: " << targetPose.x << ", " << targetPose.y << ", " << targetPose.theta)
      #ifndef __arm__
      cv::imshow("input", dstColor);
      cv::waitKey(1);
      #endif
      // Send the map as image
//      std::vector<uchar> buf;
//      std::vector<int> compression_params;
//      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
//      compression_params.push_back(85/*g_uiQuality [ 0 .. 100]*/);
//      imencode(".jpg", dstColor, buf, compression_params);

      // Send the data.
//      rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
//      informer->publish(frameJpg);
    }
  }

  return 0;
}
