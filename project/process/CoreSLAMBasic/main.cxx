
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
 #define ERROR_MSG_
#include <MSG.h>

#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

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
static double sigma_theta_ = 0.05;  // rad
static int samples = 100; // Number of resampling steps
// parameters for coreslam
static double hole_width_ = 0.1;  // m
static double delta_ = 0.02;  // Meter per pixel
static ts_map_t ts_map_;
static ts_state_t state_;
static ts_position_t position_;
static ts_position_t first_odom_;
static ts_position_t prev_odom_;
static bool gotFirstOdometry = false;
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
#include <converter/matConverter/matConverter.hpp>

// RST Proto types
//#include <rst0.11/stable/rst/vision/LaserScan.pb.h>
#include <types/LocatedLaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>
#include <types/twbTracking.pb.h>
#include <types/PoseEuler.pb.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // resize

// Include CAN funtionality for odometry
#include <ControllerAreaNetwork.h>
// Declare the CAN interface
ControllerAreaNetwork *CAN;

// For sending the localization
rsb::Informer<rst::geometry::PoseEuler>::Ptr informerOdometry;

// Actual position
rsb::Informer<rst::geometry::PoseEuler>::DataPtr odomData(new rst::geometry::PoseEuler);

#include "pathPlanner.hpp"

// object that calculates paths
PathPlanner *pathPlanner;


using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace muroxConverter; // The namespace for the own converters

#include <mutex>          // std::mutex
std::mutex map_to_odom_mutex_;
std::mutex mtxOdom;       // mutex for odometry messages
static rst::geometry::Translation odomTrans;
static rst::geometry::Rotation odomRot;
static double pathObstacleErosion = 0.1; // m
static double detectionObstacleErosion = 0.1; // m

rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DListPublish(new twbTracking::proto::Pose2DList);

// Erode the map by with an eliptic pattern of pixel radius erosion_size = <size in meter> / delta_;
void mapErosion(int erosion_size, cv::Mat &map) {
  cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                             cv::Point( erosion_size, erosion_size ) );
  cv::erode( map, map, element );
}

cv::Mat getObstacleMap() {
  // Convert the map to a cv::Mat image
  cv::Mat map(cv::Size(TS_MAP_SIZE,TS_MAP_SIZE), CV_8UC1); // destination image for sending
  cv::Mat tmp = cv::Mat(TS_MAP_SIZE, TS_MAP_SIZE, CV_16UC1, static_cast<void*>(&ts_map_.map[0]));
  tmp.convertTo(map, CV_8UC1, 0.00390625);  // Convert to 8bit depth image
  for (ssize_t idx = 0; idx < map.rows * map.cols; ++idx)
    if (map.at<uint8_t>(idx) > 127) // Delete all unsure guesses
      map.at<uint8_t>(idx) = 0xFF;
  return map;
}

// Show an image in a window with the given name.
// The image will be flipped along its x-axis.
// The window will appear at (x,y).
void imshowf(const string & winname, cv::InputArray mat, int x = 0, int y = 0) {
  cv::Mat fmat;
  cv::flip(mat, fmat, 0);
  cv::imshow(winname, fmat);
  cv::moveWindow(winname, x, y);
}

// callBack for path to point
class pathCallback: public rsb::patterns::LocalServer::Callback<twbTracking::proto::Pose2D, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/,
			boost::shared_ptr<twbTracking::proto::Pose2D> pose) {
/*
		INFO_MSG("Path to " << pose.get()->x() << "/" << pose.get()->y() << " requested.");
		// generate the obstacle map
		cv::Mat obstacleMap(getObstacleMap());
		const int erosion_size = pathObstacleErosion / delta_;
		mapErosion(erosion_size, obstacleMap);
		int xSize = obstacleMap.size().width*delta_;
		int ySize = obstacleMap.size().height*delta_;

		// convert pose to cv::point2f
		// note: 3. coordinate is ignored
		Point2f target(pose.get()->x()+xSize/2, pose.get()->y()+ySize/2);

		// convert robot position to cv::point3f
		mtxOdom.lock();
		Point3f robotPose(odomData->mutable_translation()->x(), odomData->mutable_translation()->y(), odomData->mutable_rotation()->yaw());
		mtxOdom.unlock();

		// calculate a path
		// Erode the map by with an eliptic pattern of pixel radius erosion_size = <size in meter> / delta_;
		INFO_MSG("Starting path calculation.");
		std::vector<cv::Point2f> path = pathPlanner->getPathToTarget(obstacleMap, robotPose, target);

		// convert that path to a pose2DList
		INFO_MSG("Send path.");
		rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
		for (Point2f p : path) {
			twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
			pose2D->set_x(p.x-xSize/2);
			pose2D->set_y(p.y-ySize/2);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}
		return pose2DList;*/

		return pose2DListPublish;
	}
};


void pathRequestFunction(twbTracking::proto::Pose2D pose) {
		INFO_MSG("Path to " << pose.x() << "/" << pose.y() << " requested.");
		// generate the obstacle map
		cv::Mat obstacleMap(getObstacleMap());
		const int erosion_size = pathObstacleErosion / delta_;
		mapErosion(erosion_size, obstacleMap);
		float xSize = ((float)obstacleMap.size().width) * delta_;
		float ySize = ((float)obstacleMap.size().height) * delta_;

		// convert pose to cv::point2f
		// note: 3. coordinate is ignored
		Point2f target(pose.x()+xSize/2.0, pose.y()+ySize/2.0);

		// convert robot position to cv::point3f
		mtxOdom.lock();
		Point3f robotPose(odomData->mutable_translation()->x(), odomData->mutable_translation()->y(), odomData->mutable_rotation()->yaw());
		mtxOdom.unlock();
		robotPose.x = robotPose.x + xSize/2.0;
		robotPose.y = robotPose.y + ySize/2.0;

		// calculate a path
		// Erode the map by with an eliptic pattern of pixel radius erosion_size = <size in meter> / delta_;
		INFO_MSG("Starting path calculation.");
		std::vector<cv::Point2f> path = pathPlanner->getPathToTarget(obstacleMap, robotPose, target);

		// convert that path to a pose2DList
		INFO_MSG("Send path.");
		//rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DListPublish(new twbTracking::proto::Pose2DList);
		pose2DListPublish->clear_pose();
		for (Point2f p : path) {
			twbTracking::proto::Pose2D *pose2D = pose2DListPublish->add_pose();
			pose2D->set_x(p.x - xSize/2.0);
			pose2D->set_y(p.y - ySize/2.0);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}
}

// callBack for path to point
class objectsCallback: public rsb::patterns::LocalServer::Callback<bool, twbTracking::proto::Pose2DList> {
  boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/, boost::shared_ptr<bool> draw_debug) {

    // Get the obstacles
    cv::Mat map = getObstacleMap();

    // Expand them
    const int erosion_size = detectionObstacleErosion / delta_;
    mapErosion(erosion_size, map);
    float xSize = ((float)map.size().width) * delta_;
    float ySize = ((float)map.size().height) * delta_;

    // Obstacle list
    vector<vector<cv::Point2i> > contours;

    // Temporary stuff
    cv::Mat mask, thresholded, debug;

    if (draw_debug) cv::cvtColor(map, debug, CV_GRAY2BGR);

    // Get area inside of walls
    cv::threshold(map,thresholded,127,255,cv::THRESH_BINARY);

    thresholded.copyTo(mask);

    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    mask = Mat::zeros(map.size(),CV_8UC1);

    cv::drawContours(mask, contours, -1, cv::Scalar(255),-1);

    cv::Mat help = Mat::ones(map.size(),CV_8UC1)*255;

    thresholded.copyTo(help,mask);

    // Switch black & white
    cv::Mat objects = Mat::ones(map.size(), CV_8UC1)*255;
    cv::subtract(objects, help, objects);

    // Find objects
    cv::findContours(objects, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int numObjects = contours.size();
    cv::Point2f centers[numObjects];
    float objectRadius[numObjects];
    cv::Moments moments;
    boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList(new twbTracking::proto::Pose2DList);

    for(int i = 0; i < numObjects; ++i) {
      if (cv::contourArea(contours[i]) < 5) continue;

      // Calculate objects center of gravity
      moments = cv::moments(contours[i], true);
      centers[i] = Point2f(moments.m10/moments.m00 , moments.m01/moments.m00);

      // Get objects radius
      cv::Point2f center;
      cv::minEnclosingCircle(contours[i],center,objectRadius[i]);

      // Add object as pose
      twbTracking::proto::Pose2D *pose2D1 = pose2DList->add_pose();
      pose2D1->set_x(centers[i].x * delta_ - xSize/2.0);
      pose2D1->set_y(centers[i].y * delta_ - ySize/2.0);
      pose2D1->set_orientation(objectRadius[i] * delta_);
      pose2D1->set_id(0);

      if (draw_debug) {
        cv::drawContours(debug, contours, i, cv::Scalar(255,191,0),-1);
        cv::circle(debug, centers[i], 3, cv::Scalar(139,0,0),-1);
        cv::circle(debug, center, objectRadius[i], cv::Scalar(0,0,139),1);
      }
    }
#ifndef __arm__
    if (draw_debug) {
      imshowf("objects", debug);
      cv::waitKey(1);
    }
#endif
    if (draw_debug) {
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(3);
      imwrite("detection.png", debug, compression_params);
      INFO_MSG("Server returns obstacle list")
    }
    return pose2DList;
  }
};

// RSB Server function for the mapping server which replies with a 8bit map of the environment
class mapServerObstacles: public rsb::patterns::LocalServer::Callback<void, cv::Mat> {
public:
  boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
    boost::shared_ptr<cv::Mat> dst(new cv::Mat(getObstacleMap()));
    // Expand them
    const int erosion_size = detectionObstacleErosion / delta_;
    mapErosion(erosion_size, *dst);
    INFO_MSG("Server returns obstacle map")
    return dst;
  }
};

// RSB Server function for the mapping server which replies with a 8bit map of the environment
class mapServer: public rsb::patterns::LocalServer::Callback<void, cv::Mat> {
public:
  boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
    boost::shared_ptr<cv::Mat> dst(new cv::Mat(getObstacleMap()));
    INFO_MSG("Server returns map")
    return dst;
  }
};

#define NUM_STATES 3
enum states {
  idle,
  slam,
  localization
};

static states slamState = idle;

std::string statesString[NUM_STATES] {
  "idle",
  "slam",
  "localization"
};

// Control the SLAM behaviour
void controlCoreSLAMBehaviour(boost::shared_ptr<std::string> e) {

  // Control the behaviour
  if ((e->compare("start") == 0) || (e->compare("slam") == 0))
    slamState = slam;
  else if ((e->compare("finish") == 0) || (e->compare("localization") == 0))
    slamState = localization;
  else if (e->compare("idle") == 0)
    slamState = idle;
  else
    WARNING_MSG("STATEMACHINE: Received string " << *e << ". Remain in state " << statesString[slamState])

    INFO_MSG("STATEMACHINE: Received " << *e << " State is: " << statesString[slamState])
}

static types::position robotInitPosition;
static double mapOffset = 0;

bool
getOdomPose(ts_position_t& ts_pose)
{

  // Read the odometry data (This function halts until some data was received)
  types::position robotPosition = ::CAN->getOdometry();

  // Set the Robot to the center of the map, independent from the first odometry message
  if(!gotFirstOdometry) {
    robotInitPosition.x = robotPosition.x;
    robotInitPosition.y = robotPosition.y;
    robotInitPosition.f_z = robotPosition.f_z;
    gotFirstOdometry = true;
  }

  const double translation_x = static_cast<double>(robotPosition.x - robotInitPosition.x) * 1e-3;
  const double translation_y = static_cast<double>(robotPosition.y - robotInitPosition.y) * 1e-3;
  const double yaw           = static_cast<double>(robotPosition.f_z) * 1e-6; // static_cast<double>(robotPosition.f_z - robotInitPosition.f_z) * 1e-6;
  ts_pose.x = translation_x + mapOffset; // convert to mm
  ts_pose.y = translation_y + mapOffset; // convert to mm
  ts_pose.theta = (yaw * 180/M_PI);

  DEBUG_MSG("ODOM POSE: " << ts_pose.x << " mm " << ts_pose.y << " mm " << ts_pose.theta << " deg")

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
  ts_map_init(&ts_map_);
  ts_state_init(&state_, &ts_map_, &lparams_, &position_, sigma_xy_, sigma_theta_*180/M_PI, (int)(hole_width_*1000), 0, samples);

  INFO_MSG("Initialized with sigma_xy=" << sigma_xy_<< ", sigma_theta=" << ", hole_width=" << hole_width_ << ", delta=" << delta_);
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
  state_.position.theta += odom_pose.theta - prev_odom_.theta; //- first_odom_.theta;
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

//  state_.position.theta = 90;
  DEBUG_MSG("Initial position step: "<< laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta << ")")

  // Mapping
  if(laser_count_ < 10){
    INFO_MSG("BOOTSTRAP -- I assume that I stand still")
  // not much of a map, let's bootstrap for now
    ts_scan_t ranges;
    ts_build_scan(&data, &ranges, &state_, 3 /*widening of the ray*/);
    ts_map_update(&ranges, &ts_map_, &state_.position, 50, (int)(hole_width_*1000));
    DEBUG_MSG("Update step, " << laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
  }else{

    // Monte carlo localization is done inside
    if (slamState == slam)
      ts_iterative_map_building(&data, &state_, true /*do Map Update*/);
    else /*if (slamState == localization)*/
      ts_iterative_map_building(&data, &state_, false /*do only localization*/);

    DEBUG_MSG("End postition step: "<< laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta << ")")
    DEBUG_MSG("Correction: "<< state_.position.x - prev.x << ", " << state_.position.y - prev.y << ", " << state_.position.theta - prev.theta)
  }
  // Set the new pose
  pose = state_.position;


  // Publish the new odometry data
  mtxOdom.lock();
  odomData->mutable_translation()->set_x((pose.x - mapOffset) * 1e-3); // From mm to m
  odomData->mutable_translation()->set_y((pose.y - mapOffset) * 1e-3); // From mm to m
  odomData->mutable_translation()->set_z(0.0f);
  odomData->mutable_rotation()->set_roll(0.0f);
  odomData->mutable_rotation()->set_pitch(0.0f);
  odomData->mutable_rotation()->set_yaw(pose.theta * M_PI / 180);
  mtxOdom.unlock();
  DEBUG_MSG("SLAM POSE: " << (pose.x - mapOffset) * 1e-3 << " m " << (pose.y - mapOffset) * 1e-3 << " m " << pose.theta * M_PI / 180 << " rad")
  informerOdometry->publish(odomData);

  return true;
}

int main(int argc, const char **argv){

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

  // tinySLAM init
  ts_map_set_scale(MM_TO_METERS/delta_);  // Set TS_MAP_SCALE at runtime
  mapOffset = ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM);

  // Create path planner
  PathPlanner pathPlannerObj(delta_);
  pathPlanner = &pathPlannerObj;

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
  // Prepare RSB informer for sending the path answer
  rsb::Informer<std::string>::Ptr pathInformer = factory.createInformer<std::string> (sPathOutputScope);
  // Prepare RSB listener for controling the programs behaviour
  rsb::ListenerPtr expListener = factory.createListener(sExplorationScope);
  expListener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&controlCoreSLAMBehaviour)));
  // Prepare RSB informer for sending the map as an compressed image
  rsb::Informer<std::string>::Ptr informer = factory.createInformer<std::string> (mapAsImageOutScope, tmpPartConf);
  // Prepare RSB informer for sending the map as an compressed image
  informerOdometry = factory.createInformer<rst::geometry::PoseEuler> (localizationOutScope);
  // Prepare RSB server for the map server
  rsb::patterns::LocalServerPtr server = factory.createLocalServer(serverScope, tmpPartConf, tmpPartConf);
  server->registerMethod(pathServerReq, rsb::patterns::LocalServer::CallbackPtr(new pathCallback()));
  server->registerMethod(mapServerReq, rsb::patterns::LocalServer::CallbackPtr(new mapServer()));
  server->registerMethod(mapServerObstacleReq, rsb::patterns::LocalServer::CallbackPtr(new mapServerObstacles()));
  server->registerMethod(obstacleServerReq, rsb::patterns::LocalServer::CallbackPtr(new objectsCallback()));

  // Init the CAN controller
  ::CAN = new(ControllerAreaNetwork);

  rst::vision::LocatedLaserScan scan;
  // Do SLAM
  while( true ){
    // Idle arround if the state says so
    if (slamState == idle) {
      usleep(50000); // Sleep for 50 ms
      continue;
    }

    if (!pathQueue->empty()) {
      pathRequestFunction(*pathQueue->pop());
      boost::shared_ptr<std::string> stringPublisher(new std::string("Fin"));
      pathInformer->publish(stringPublisher);
    }

    ++laser_count_;
    if ((laser_count_ % throttle_scans_) != 0)
      continue;

    // Fetch a new scan and store it to scan
    scan = *(lidarQueue->pop());
    ts_position_t pose;
    ts_position_t odom_pose;
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
        DEBUG_MSG("scan processed");
        DEBUG_MSG("Updated the map");
      }
    }

    if (sendMapAsCompressedImage) {
      // Show the map as a cv Image
      cv::Size size(TS_MAP_SIZE / 2,TS_MAP_SIZE / 2);
      cv::Mat dst(size, CV_16S); // destination image for scaling
      cv::Mat dstColor(size, CV_8UC3); // Color image

      cv::Mat image = cv::Mat(TS_MAP_SIZE, TS_MAP_SIZE, CV_16U, static_cast<void*>(&ts_map_.map[0]));
      cv::resize(image,dst,size);//resize image
      cv::flip(dst, dst, 0);  // horizontal flip
      dst.convertTo(dst, CV_8U, 0.00390625);  // Convert to 8bit depth image
      cv::cvtColor(dst, dstColor, cv::COLOR_GRAY2RGB, 3);  // Convert to color image
      cv::Point robotPosition(pose.x * MM_TO_METERS / delta_ * size.width / TS_MAP_SIZE,(TS_MAP_SIZE - (pose.y * MM_TO_METERS / delta_)) * size.height / TS_MAP_SIZE);  // Draw MCMC position
      cv::circle( dstColor, robotPosition, 0, cv::Scalar( 0, 0, pow(2,8)-1), 10, 8 );
      cv::Point robotOdomPosition(odom_pose.x * MM_TO_METERS / delta_ * size.width / TS_MAP_SIZE,(TS_MAP_SIZE - (odom_pose.y * MM_TO_METERS / delta_)) * size.height / TS_MAP_SIZE);  // Draw odometry
      cv::circle( dstColor, robotOdomPosition, 0, cv::Scalar( 0, pow(2,8)-1), 0, 10, 8 );
      DEBUG_MSG( "Pose " << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta)
      #ifndef __arm__
      cv::imshow("input", dstColor);
      cv::waitKey(1);
      #endif
      // Send the map as image
      std::vector<uchar> buf;
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
      compression_params.push_back(85/*g_uiQuality [ 0 .. 100]*/);
      imencode(".jpg", dstColor, buf, compression_params);

      // Send the data.
      rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
      informer->publish(frameJpg);
    }
  }

  return 0;
}

