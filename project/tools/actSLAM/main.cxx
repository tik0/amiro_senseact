
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "../../includes/MSG.h"

#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// tinySLAM
#ifdef __cplusplus
extern "C"{
#endif
#include <CoreSLAM.h>
#ifdef __cplusplus
}
#endif
// parameters for coreslam
static double sigma_xy_;
static double sigma_theta_;
static double hole_width_;
static int span_;
static double delta_;
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
const float rayPruningAngleDegree = 60; /* [0 .. 90] */
constexpr float rayPruningAngle(){return asin(90 - rayPruningAngleDegree / 180 * M_PI);}


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
#include <rst0.11/stable/rst/vision/LaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // resize


using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;

#include <mutex>          // std::mutex
std::mutex map_to_odom_mutex_;
std::mutex mtxOdom;       // mutex for odometry messages
static rst::geometry::Translation odomTrans;
static rst::geometry::Rotation odomRot;

void convertDataToScan(rsb::EventPtr event, ts_scan_t *scan, rst::vision::LaserScan &rsbScan) {
  
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  INFO_MSG( "Scan rec.")
  // Get the message
  boost::shared_ptr< rst::vision::LaserScan > data = boost::static_pointer_cast< rst::vision::LaserScan >(event->getData());

  // Copy the whole scan
  rsbScan = *data;

  // Check if two adjiacent rays stand almost perpendiculat on the surface
  // and set the first one to an invalid measurement if the angle is to big
  for (int idx = 0; idx < rsbScan.scan_values_size()-1; idx++) {
    float a = rsbScan.scan_values(idx);
    float b = rsbScan.scan_values(idx+1);
    float h = a * sin(rsbScan.scan_angle_increment());
    float b_t = sqrt(pow(a,2) - pow(h,2));
    float b_tt = b - b_t;
    float beta_t = atan2(h, b_t);
    float beta_tt = atan2(h, b_tt);
    float beta = beta_t + beta_tt;
    if (sin(beta) < rayPruningAngle())
      rsbScan.mutable_scan_values()->Set(idx, rsbScan.scan_values_max() + 42.0f); // Increment the value, so that it becomes invalid
  }
  // Delete the last value, if the former value is invalid
  if (rsbScan.scan_values(rsbScan.scan_values_size() -2) > rsbScan.scan_values_max())
    rsbScan.mutable_scan_values()->Set(rsbScan.scan_values_size() - 1, rsbScan.scan_values_max() + 42.0f);

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
  const double a = rotation.qw();
  const double b = rotation.qx();
  const double c = rotation.qy();
  const double d = rotation.qz();

  // Calculate the rotation in [-pi .. pi]
//  const double f_x = atan2(2*((a*b) + (c*d)) , (pow(a,2) - pow(b,2) - pow(c,2) + pow(d,2)));
//  const double f_y = - asin(2*((b*d) + (a*c)));
  const double f_z = atan2(2*((a*d) + (b*c)) , (pow(a,2) + pow(b,2) - pow(c,2) - pow(d,2)));

  ts_pose.x = translation.x()*METERS_TO_MM + ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM); // convert to mm
  ts_pose.y = translation.y()*METERS_TO_MM + ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM);
  ts_pose.theta = (f_z * 180/M_PI);

  DEBUG_MSG("ODOM POSE: " << ts_pose.x << " " << ts_pose.y << " " << ts_pose.theta)

  return true;
}

bool
initMapper(const rst::vision::LaserScan& scan)
{

  // configure previous_odom
  if(!getOdomPose(prev_odom_))
     return false;
  position_ = prev_odom_;

  // configure laser parameters
  lparams_.offset = 0.0;  // No offset Todo what is the effect of changing this value?
  lparams_.scan_size = scan.scan_values_size();
  lparams_.angle_min = scan.scan_angle_start()  * 180/M_PI;
  lparams_.angle_max = scan.scan_angle_end()  * 180/M_PI;
  lparams_.detection_margin = 0; // Todo what is this
  lparams_.distance_no_detection = scan.scan_values_max() * METERS_TO_MM;

  // new coreslam instance
  ts_map_init(&ts_map_);
  ts_state_init(&state_, &ts_map_, &lparams_, &position_, (int)(sigma_xy_*1000), (int)(sigma_theta_*180/M_PI), (int)(hole_width_*1000), 0);

  INFO_MSG("Initialized with sigma_xy=" << sigma_xy_<< ", sigma_theta=" << ", hole_width=" << hole_width_ << ", delta=" << delta_);
  INFO_MSG("Initialization complete");
  return true;
}

bool addScan(const rst::vision::LaserScan& scan, ts_position_t& odom_pose)
{
  // update odometry
  if(!getOdomPose(odom_pose))
     return false;
  state_.position.x += odom_pose.x - prev_odom_.x;
  state_.position.y += odom_pose.y - prev_odom_.y;
  state_.position.theta += odom_pose.theta - prev_odom_.theta;
  prev_odom_ = odom_pose;

  ts_position_t prev = state_.position;

  // update params -- mainly for PML
  lparams_.scan_size = scan.scan_values_size();
  lparams_.angle_min = scan.scan_angle_start()  * 180/M_PI;
  lparams_.angle_max = scan.scan_angle_end()  * 180/M_PI;

  if(laser_count_ < 10){
    // not much of a map, let's bootstrap for now
    ts_scan_t ranges;
    ranges.nb_points = 0;
    const float delta_angle = scan.scan_angle() / scan.scan_values_size();
    for(int i=0; i < lparams_.scan_size; i++) {
      // Must filter out short readings, because the mapper won't
      if(scan.scan_values(i) > 0.9 /*hokuyo.sdf*/ && scan.scan_values(i) < 4.9/*hokuyo.sdf*/){
        ranges.x[ranges.nb_points] = cos(lparams_.angle_min + i*delta_angle) * (scan.scan_values(i)*METERS_TO_MM);
        ranges.y[ranges.nb_points] = sin(lparams_.angle_min + i*delta_angle) * (scan.scan_values(i)*METERS_TO_MM);
        ranges.value[ranges.nb_points] = TS_OBSTACLE;
        ranges.nb_points++;
      }
    }
    ts_map_update(&ranges, &ts_map_, &state_.position, 50, (int)(hole_width_*1000));
    DEBUG_MSG("Update step, " << laser_count_ << ", now at (" << state_.position.x ", " << state_.position.y << ", " << state_.position.theta)
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
    ts_iterative_map_building(&data, &state_);
    DEBUG_MSG("Iterative step, "<< laser_count_ << ", now at (" << state_.position.x ", " << state_.position.y << ", " << state_.position.theta)
    DEBUG_MSG("Correction: "<< state_.position.x - prev.x << ", " << state_.position.y - prev.y << ", " << state_.position.theta - prev.theta)
  }

  odom_pose = state_.position;

  return true;
}

int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
  std::string odomInScope = "/AMiRo_Hokuyo/gps";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data")
    ("odominscope", po::value < std::string > (&odomInScope), "Scope for receiving odometry data");

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
  static ts_scan_t scan;
  // Parameters needed for CoreSLAM
  sigma_xy_ = 0.1;
  sigma_theta_ = 0.35;
  hole_width_ = 0.1;
  span_ = 3;
  delta_ = 0.03;
  ts_map_set_scale(MM_TO_METERS/delta_);  // Set TS_MAP_SCALE at runtime

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();
  
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);


  // Prepare RSB reader for incomming lidar scans
  rsb::ReaderPtr reader = factory.createReader(lidarInScope);
  // Prepare RSB async listener for odometry messages
  rsb::ListenerPtr listener = factory.createListener(odomInScope);
  listener->addHandler(HandlerPtr(new DataFunctionHandler<rst::geometry::Pose> (&storeOdomData)));

  // Show the map as a cv Image
  cv::Size size(TS_MAP_SIZE / 2,TS_MAP_SIZE / 2);
  cv::Mat dst(size, CV_16S); // destination image for scaling
  cv::Mat dstColor(size, CV_16UC3); // Color image


  rst::vision::LaserScan rsbScan;
  while( true ){
    convertDataToScan(reader->read(), &scan, rsbScan);

    // We can't initialize CoreSLAM until we've got the first scan
    if(!got_first_scan_)
    {
      if(!initMapper(rsbScan))
        continue;
      got_first_scan_ = true;
    }

    ///////////////////////////////////////////////
    ts_position_t odom_pose;
    if(addScan(rsbScan, odom_pose))
    {
      DEBUG_MSG("scan processed");
      DEBUG_MSG("odom pose:" << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta);

      //TODO pulish the map
      DEBUG_MSG("Updated the map");
    }

    cv::waitKey(1);
    cv::Mat image = cv::Mat(TS_MAP_SIZE, TS_MAP_SIZE, CV_16U, static_cast<void*>(&ts_map_.map[0]));
    cv::resize(image,dst,size);//resize image
    cv::flip(dst, dst, 0);  // horizontal flip
    cv::cvtColor(dst, dstColor, cv::COLOR_GRAY2RGB, 3);
    cv::Point robotPosition(odom_pose.x * MM_TO_METERS / delta_ * size.width / TS_MAP_SIZE,(TS_MAP_SIZE - (odom_pose.y * MM_TO_METERS / delta_)) * size.height / TS_MAP_SIZE);
    cv::circle( dstColor, robotPosition, 0, cv::Scalar( 0, 0, pow(2,16)-1), 10, 8 );
    DEBUG_MSG( "Pose " << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta)
    cv::imshow("input", dstColor);
  }

  return 0;
}

