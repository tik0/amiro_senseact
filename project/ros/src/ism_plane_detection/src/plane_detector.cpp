#include <opencv2/opencv.hpp>
#include <exception>
#include <algorithm>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <fstream>
#include <functional>
#include <iostream>


// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

ros::Publisher publisher, publisherDebug;

// Stdandard libraries
#include <mutex>          // std::mutex
#include <future>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>    // std::min
#include <exception>
#include <map>
#include <memory>

static std::string inputTopic, outputTopic, baseFrameId, outputTopicDebug;
static signed char lower_boundary_schar, upper_boundary_schar;
static double lower_boundary, upper_boundary, fuzzyIntersection_m, fuzzyUpperLimit_m, fuzzyScale;
const double fuzzyIntersectionProbability = 0.5;
static int debug;

static double resolution, width, depth;

static nav_msgs::OccupancyGrid msgOgm;

tf::TransformListener *listenerTf;

void callback(const sensor_msgs::LaserScan::ConstPtr &msg) {


  // Handle the scan data in the ROI
  sensor_msgs::PointCloud cloud, cloudBase; // Point clouds for transformation
  laser_geometry::LaserProjection projector;
  projector.projectLaser(*msg, cloud, -1.0);

  try {
    listenerTf->waitForTransform(baseFrameId, cloud.header.frame_id, cloud.header.stamp, ros::Duration(0.3));
    listenerTf->transformPointCloud(baseFrameId, cloud, cloudBase);
  } catch(const std::exception &exc) {
    const std::string excStr(exc.what());
    ROS_WARN("%s, frameTarget:%s, cloud.header.frame_id:%s\n", excStr.c_str(), baseFrameId.c_str(), cloud.header.frame_id.c_str());
    return;
  }

  // Create OGM
  msgOgm.header = msg->header;
  msgOgm.header.frame_id = baseFrameId;
  msgOgm.info.resolution = resolution;
  msgOgm.info.width = uint32_t(width / resolution);
  msgOgm.info.height = uint32_t(depth / resolution);
  tf::Pose ogmPose(tf::Quaternion(0,0,0,1), tf::Vector3(tfScalar(-width / 2.0), tfScalar(-depth / 2.0), tfScalar(0)));
  tf::poseTFToMsg(ogmPose, msgOgm.info.origin);
  msgOgm.data.assign(msgOgm.info.width * msgOgm.info.height, 50);  // Reset

  // Classification (without intensities, only height in Z is respected)
  for (auto it = cloudBase.points.begin(); it != cloudBase.points.end(); ++it) {
      if (fabs(it->x) < (width / 2.0) && fabs(it->y) < (depth / 2.0)) {
          const float zAbs = fabs(it->z);
          // Do symmetric fuzzy classification with respect to the intersection
          char ogmVal;
          if (zAbs > fuzzyUpperLimit_m) {
              ogmVal = upper_boundary_schar;
          } else {
              ogmVal = char(fuzzyScale * zAbs) + lower_boundary_schar;
          }

          const double x_m = (it->x + width / 2.0);
          const double y_m = (it->y + depth / 2.0);
          const std::size_t idx = std::size_t(x_m / resolution) +  std::size_t(y_m / resolution) * msgOgm.info.width;
          ROS_DEBUG("(x_m, y_m) -> (idx, idy, index): (%f, %f) -> (%d, %d, %d)", x_m, y_m, int(x_m / resolution), int(y_m / resolution), int(idx));
          if (idx < msgOgm.data.size()) {
              msgOgm.data.at(idx) = ogmVal;
          } else {
              ROS_WARN("msgOgm: Data out of bounds");
          }
      }
  }


  // Publish
  if(debug) {
      publisherDebug.publish(cloudBase);
  }
  publisher.publish(msgOgm);
}

int main(int argc, char **argv){

  // ROS
  ros::init(argc, argv, "ism_plane_detection");
  ros::NodeHandle n("~");

  n.param<std::string>("laser_input_topic", inputTopic, "");
  n.param<std::string>("ogm_output_topic", outputTopic, "");
  n.param<std::string>("pointcloud_output_topic", outputTopicDebug, "fooo");
  n.param<std::string>("base_frame_id", baseFrameId, "");
  n.param<double>("resolution", resolution, 0.2);
  n.param<double>("width", width, 1.0);
  n.param<double>("depth", depth, 1.0);
  n.param<double>("fuzzy_intersection_m", fuzzyIntersection_m, 0.01);
  n.param<double>("upper_boundary", upper_boundary, 0.8);
  n.param<double>("lower_boundary", lower_boundary, 0.2);
  n.param<int>("debug", debug, 0); // Enable debug outputs

  upper_boundary_schar = schar(upper_boundary * 100.0);
  lower_boundary_schar = schar(lower_boundary * 100.0);
  fuzzyUpperLimit_m = fuzzyIntersection_m * 2.0;
  fuzzyScale = (upper_boundary_schar - lower_boundary_schar) / fuzzyUpperLimit_m;

  ROS_INFO("ISM will be generated around the %s frame with (width (m), depth (m), resolution (m/cell)): (%f, %f, %f)", baseFrameId.c_str(), width, depth, resolution);


  listenerTf = new tf::TransformListener;

  ros::Subscriber subscriber = n.subscribe<sensor_msgs::LaserScan>(inputTopic, 2, callback);
  publisher = n.advertise<nav_msgs::OccupancyGrid>(outputTopic, 2);
  if (debug) {
      publisherDebug = n.advertise<sensor_msgs::PointCloud>(outputTopicDebug, 2);
  }

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate rate(5);
  while (ros::ok()) {
      rate.sleep();
  }
  delete listenerTf;
  return 0;
}
