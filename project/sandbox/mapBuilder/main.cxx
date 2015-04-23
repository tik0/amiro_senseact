
// std includes
#include <iostream>
using namespace std;



// types
#include <types/twbTracking.pb.h>


// project includes
#include "mapGenerator.hpp"
#include "pathPlanner.hpp"

//#define INFO_MSG_
//#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "../../includes/MSG.h"

#include <math.h>

#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <types/twbTracking.pb.h>
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
static double height_amiro = 94;
static double height_lidar_measurement = 57.25;
static double radius_lidar = 25;
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
static double rotX, rotY = 0, rotZ;

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
#include <rst/geometry/Pose.pb.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // resize
using namespace cv;

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace rsb::patterns;

#include <mutex>          // std::mutex
std::mutex map_to_odom_mutex_;
std::mutex mtxOdom;       // mutex for odometry messages
static rst::geometry::Translation odomTrans;
static rst::geometry::Rotation odomRot;



////////////////////////////////////////









// cellSize
const float cellSize = 0.01;


// obect used to update the map from sensorvalues
MapGenerator mapGenerator(cellSize);

// object that calculates paths
PathPlanner pathPlanner(cellSize);

// initialize the maps with 0
cv::Mat gridmap;


// the robots pose
cv::Point3f robotPose(0, 0, 0);

// callBack for the map
class MapCallback: public LocalServer::Callback<void, cv::Mat> {
	boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {

		boost::shared_ptr<cv::Mat> frame(new cv::Mat(gridmap));
		return frame;
	}
};

// callBack for the obstacleMap
class ObstacleMapCallback: public LocalServer::Callback<void, cv::Mat> {
	boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {

		Mat obstacleMap;
		mapGenerator.generateObstacleMap(gridmap, obstacleMap);
		boost::shared_ptr<cv::Mat> frame(new cv::Mat(obstacleMap));
		return frame;

	}
};

// callBack for frontier path
class frontierPathCallback: public LocalServer::Callback<void, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/) {

		// generate the obstacle map
		Mat obstacleMap;
		mapGenerator.generateObstacleMap(gridmap, obstacleMap);

		// calculate a path
		std::vector<cv::Point2f> path = pathPlanner.getPathToFrontier(obstacleMap, robotPose);

		// convert that path to a pose2DList
		rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
		for (Point2f p : path) {
			twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
			pose2D->set_x(p.x);
			pose2D->set_y(p.y);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}

		return pose2DList;
	}
};

// callBack for path to point
class pathCallback: public LocalServer::Callback<twbTracking::proto::Pose2D, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/,
			boost::shared_ptr<twbTracking::proto::Pose2D> pose) {

		// generate the obstacle map
		Mat obstacleMap;
		mapGenerator.generateObstacleMap(gridmap, obstacleMap);

		// convert pose to cv::point2f
		// note: 3. coordinate is ignored
		Point2f target(pose.get()->x(), pose.get()->y());

		// calculate a path
		std::vector<cv::Point2f> path = pathPlanner.getPathToTarget(obstacleMap, robotPose, target);

		// convert that path to a pose2DList
		rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
		for (Point2f p : path) {
			twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
			pose2D->set_x(p.x);
			pose2D->set_y(p.y);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}
		return pose2DList;
	}
};

// callBack for path to point
class pushingPathCallback: public LocalServer::Callback<twbTracking::proto::Pose2DList, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/,
			boost::shared_ptr<twbTracking::proto::Pose2DList> inputPointList) {

		// generate the obstacle map
		Mat obstacleMap;
		mapGenerator.generateObstacleMap(gridmap, obstacleMap);

		// convert pose to cv::point2f
		// note: 3. coordinate is ignored
		Point3f startPos(inputPointList->pose(0).x(), inputPointList->pose(0).y(), 0);
		Point2f target(inputPointList->pose(1).x(), inputPointList->pose(1).y());
		vector<vector<cv::Point2i> > contours;
		vector<cv::Point2i> contour;
		for(int i = 2; i < inputPointList->pose_size()-2; ++i) contour.push_back(Point2i(inputPointList->pose(i).x(), inputPointList->pose(i).y()));
		contours.push_back(contour);
		cv::drawContours(obstacleMap, contours, 0, cv::Scalar(255),-1);
//		cv::drawContours(gridmap, contours, 0, cv::Scalar(255),-1);

		// calculate a path
		std::vector<cv::Point2f> path = pathPlanner.getPathToTarget(obstacleMap, startPos, target);

		// convert that path to a pose2DList
		rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
		for (Point2f p : path) {
			twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
			pose2D->set_x(p.x);
			pose2D->set_y(p.y);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}
		return pose2DList;
	}
};



void convertDataToScan(boost::shared_ptr< rst::vision::LocatedLaserScan > data , rst::vision::LocatedLaserScan &rsbScan) {
  
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

//  DEBUG_MSG( "Scan rec.")
//  Eigen::Quaterniond lidar_quat(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
//  Eigen::AngleAxisd lidar_angle(lidar_quat);
//  Eigen::Matrix<double,3,1> rpy = lidar_angle.toRotationMatrix().eulerAngles(0,1,2);
//  const double yaw = rpy(2);

  // Copy the whole scan
  rsbScan = *data;

  // Shifting the rays
  DEBUG_MSG("Startscan: " << rsbScan.scan_angle_start() * 180.0 / M_PI)
  DEBUG_MSG("Endscan: " << rsbScan.scan_angle_end() * 180.0 / M_PI)
  DEBUG_MSG("Angle: " << rsbScan.scan_angle() * 180.0 / M_PI)
  DEBUG_MSG("Size: " << rsbScan.scan_values_size())
  DEBUG_MSG("Inc: " << rsbScan.scan_angle_increment() * 180.0 / M_PI)
  DEBUG_MSG("Minrange: " << rsbScan.scan_values_min());
//  rsbScan.set_scan_angle_increment(-rsbScan.scan_angle_increment);

//  rsbScan.set_scan_angle_start(rsbScan.scan_angle_start() + M_PI / 2.0f);
//  rsbScan.set_scan_angle_end(rsbScan.scan_angle_end() + M_PI / 2.0f);
//
//  rsbScan.set_scan_angle_start(0);
//  rsbScan.set_scan_angle_end(240.0f / 180.0f * M_PI);
//  rsbScan.set_scan_angle_start(120.0f / 180.0f * M_PI);
//    rsbScan.set_scan_angle_end(-120.0f / 180.0f * M_PI);
  
  // Check if two adjiacent rays stand almost perpendiculat on the surface
  // and set the first one to an invalid measurement if the angle is to big
//  for (int idx = 0; idx < rsbScan.scan_values_size()-1; idx++) {
//    float a = rsbScan.scan_values(idx);
//    float b = rsbScan.scan_values(idx+1);
//    float h = a * sin(rsbScan.scan_angle_increment());
//    float b_t = sqrt(pow(a,2) - pow(h,2));
//    float b_tt = b - b_t;
//    float beta_t = atan2(h, b_t);
//    float beta_tt = atan2(h, b_tt);
//    float beta = beta_t + beta_tt;
//    if (sin(beta) < rayPruningAngle())
//      rsbScan.mutable_scan_values()->Set(idx, rsbScan.scan_values_max() + 42.0f); // Increment the value, so that it becomes invalid
//  }
//  // Delete the last value, if the former value is invalid
//  if (rsbScan.scan_values(rsbScan.scan_values_size() -2) > rsbScan.scan_values_max())
//    rsbScan.mutable_scan_values()->Set(rsbScan.scan_values_size() - 1, rsbScan.scan_values_max() + 42.0f);


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
  Eigen::AngleAxisd lidar_angle(lidar_quat);
  Eigen::Matrix<double,3,1> rpy = lidar_angle.toRotationMatrix().eulerAngles(0,1,2);
  const double yaw = rpy(2);

  ts_pose.x = translation.x()*METERS_TO_MM + ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM); // convert to mm
  ts_pose.y = translation.y()*METERS_TO_MM + ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM); // convert to mm
  ts_pose.theta = (yaw * 180/M_PI);

  DEBUG_MSG( "Odom: x: " <<  translation.x() << "m   y: " << translation.y() << "m     theta: " << rotation.qz() << "°" )
  DEBUG_MSG("ODOM POSE: " << ts_pose.x << " " << ts_pose.y << " " << ts_pose.theta)

  return true;
}

bool
initMapper(const rst::vision::LocatedLaserScan& scan, const double rotY)
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
  lparams_.distance_no_detection = 500 ;// scan.scan_values_max() * METERS_TO_MM;
  lparams_.tilt_angle =  rotY * M_PI / 180;
  lparams_.depth = sin(lparams_.tilt_angle+atan(height_lidar_measurement/radius_lidar)) * sqrt(pow(height_lidar_measurement,2)+pow(radius_lidar,2)) + height_amiro;
  lparams_.depth_max = lparams_.depth + 50;
  lparams_.depth_min = lparams_.depth - 15;
  lparams_.angle_cap_min = -90;
  lparams_.angle_cap_max = 90;

  // new coreslam instance
  ts_map_init(&ts_map_);
  ts_state_init(&state_, &ts_map_, &lparams_, &position_, (int)(sigma_xy_*1000), (int)(sigma_theta_*180/M_PI), (int)(hole_width_*1000), 0);

  INFO_MSG("Initialized with sigma_xy=" << sigma_xy_<< ", sigma_theta=" << ", hole_width=" << hole_width_ << ", delta=" << delta_);
  INFO_MSG("Initialization complete");
  return true;
}

void convertToScan(const rst::vision::LocatedLaserScan &scan , ts_scan_t &ranges) {
  ranges.nb_points = 0;
  const float delta_angle = scan.scan_angle_increment();

    for(int i=0; i < lparams_.scan_size; i++) {
      // Must filter out short readings, because the mapper won't
      if(scan.scan_values(i) > scan.scan_values_min()&& scan.scan_values(i) < scan.scan_values_max()){
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

  // Mapping
  if(laser_count_ < 10){
    // not much of a map, let's bootstrap for now
    // TODO need to be fixed for arbitrary angels
//    ts_scan_t ranges;
//    convertToScan(scan , ranges);
//    ts_map_update(&ranges, &ts_map_, &state_.position, 50, (int)(hole_width_*1000));
//    DEBUG_MSG("Update step, " << laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
  }else{

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
    // Monte carlo localization is done inside
    ts_iterative_map_building(&data, &state_);
    DEBUG_MSG("Iterative step, "<< laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
    DEBUG_MSG("Correction: "<< state_.position.x - prev.x << ", " << state_.position.y - prev.y << ", " << state_.position.theta - prev.theta)
  }
  // Set the new pose
  pose = state_.position;

  return true;
}

int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
  std::string odomInScope = "/AMiRo_Hokuyo/gps";
  int id = 0;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data")
    ("odominscope", po::value < std::string > (&odomInScope), "Scope for receiving odometry data")
    ("sigma_xy", po::value < double > (&sigma_xy_), "XY uncertainty for marcov localization [m]")
    ("sigma_theta", po::value < double > (&sigma_theta_), "Theta uncertainty for marcov localization [m]")
    ("hole_width", po::value < double > (&hole_width_), "Width of impacting rays [m]")
    ("delta", po::value < double > (&delta_), "Resolution [m/pixel]")
    ("rayPruningAngleDegree", po::value < float > (&rayPruningAngleDegree), "Pruning of adjiacent rays if they differ to much on the impacting surface [0° .. 90°]")
    ("senImage", po::value < bool > (&sendMapAsCompressedImage), "Send map as compressed image")
    ("transX", po::value < double > (&transX),"Translation of the lidar in x [m]")
    ("transY", po::value < double > (&transY),"Translation of the lidar in y [m]")
    ("transZ", po::value < double > (&transZ),"Translation of the lidar in z [m]")
    ("rotX", po::value < double > (&rotX),"Rotation of the lidar around x (roll) [rad]")
    ("rotY", po::value < double > (&rotY),"Rotation of the lidar around y (pitch) [rad]")
    ("rotZ", po::value < double > (&rotZ),"Rotation of the lidar around z (yaw) [rad]")
	("id",po::value<int>(&id),"Id");

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

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();
  
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);


	// ------------ Converters ----------------------

// register converter for twbTracking::proto::Pose2D
boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
		new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

// Register new converter for the pose list
boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
		new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);






  // scopenames for rsb
  	std::string poseOutscope = "/amiro"+lexical_cast<std::string>(id)+"/pose";
  	std::string serverScope = "/amiro"+lexical_cast<std::string>(id)+ "/mapGenerator";




  	  		// ---------------- Informer ---------------------

  	// create rsb informer to publish poses
  	rsb::Informer<twbTracking::proto::Pose2D>::Ptr poseInformer = factory.createInformer<twbTracking::proto::Pose2D>(poseOutscope);

  		// ---------------- Remote Procedure calls ---------------------

  	LocalServerPtr server = factory.createLocalServer(serverScope);

  	// Register method with name and implementing callback object.
  	server->registerMethod("getMap", LocalServer::CallbackPtr(new MapCallback()));
  	server->registerMethod("getObstacleMap", LocalServer::CallbackPtr(new ObstacleMapCallback()));
  	server->registerMethod("getFrontierPath", LocalServer::CallbackPtr(new frontierPathCallback()));
  	server->registerMethod("getPath", LocalServer::CallbackPtr(new pathCallback()));
  	server->registerMethod("getPushingPath", LocalServer::CallbackPtr(new pushingPathCallback()));



  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
  // Prepare RSB async listener for odometry messages
  rsb::ListenerPtr listener = factory.createListener(odomInScope);
  listener->addHandler(HandlerPtr(new DataFunctionHandler<rst::geometry::Pose> (&storeOdomData)));
  // Prepare RSB informer for sending the map as an compressed image
  rsb::Informer<std::string>::Ptr informer = factory.createInformer<std::string> ("/image");

  // Show the map as a cv Image
  cv::Size size(TS_MAP_SIZE / 2,TS_MAP_SIZE / 2);
  cv::Mat dst(size, CV_16S); // destination image for scaling
  cv::Mat dstColor(size, CV_8UC3); // Color image

  rst::vision::LocatedLaserScan scan;
  while( true ){
    // Fetch a new scan and store it to scan
    convertDataToScan(lidarQueue->pop(), scan);
    ts_position_t pose;
    ts_position_t odom_pose;
    getOdomPose(odom_pose);
    // We can't initialize CoreSLAM until we've got the first scan
    if(!got_first_scan_)
    {
      if(!initMapper(scan,rotY))
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


    boost::shared_ptr<twbTracking::proto::Pose2D> pos(new twbTracking::proto::Pose2D());
    robotPose.x = pose.x    *MM_TO_METERS ;
    robotPose.y = pose.y    *MM_TO_METERS ;
    robotPose.z = pose.theta;
    pos->set_x(robotPose.x);
    pos->set_y(robotPose.y);
    pos->set_orientation(robotPose.z);
    poseInformer->publish(pos);


    cv::Mat gridmap0 = cv::Mat(TS_MAP_SIZE, TS_MAP_SIZE, CV_16U, static_cast<void*>(&ts_map_.map[0]));
    gridmap0.convertTo(gridmap,CV_8U, 0.00390625);
    if (sendMapAsCompressedImage) {
    	cv::Mat omap;
      //cv::Mat image = cv::Mat(TS_MAP_SIZE, TS_MAP_SIZE, CV_16U, static_cast<void*>(&ts_map_.map[0]));
    	mapGenerator.generateObstacleMap(gridmap,omap);
      cv::resize(omap,dst,size);//resize image
      cv::flip(dst, dst, 0);  // horizontal flip
      //dst.convertTo(dst, CV_8U, 0.00390625);  // Convert to 8bit depth image
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
//      rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
//      informer->publish(frameJpg);
    }
  }

  return 0;
}

