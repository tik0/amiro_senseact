// ============================================================================
// Name        : ros_laser_scan_to_ros_pointcloud2.cpp
// Author      : Jonas Dominik Homburg <jhomburg@techfak.uni-bielefeld.de>
// Description : This tool directly converts data provided as laser_scan into pointcloud2.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

string laserscan_topic;
string pointcloud2_topic;
string target_frame;
// tf::TransformListener tfListener;
ros::Publisher pointcloud2;
laser_geometry::LaserProjection projector;


const string programName = "ros_laser_scan_to_ros_pointcloud2";

void convert(const sensor_msgs::LaserScan::ConstPtr& scan){
  std::cout << "Test1\n";
  sensor_msgs::PointCloud2 cloud;
  projector.projectLaser(*scan, cloud);
  pointcloud2.publish(cloud);
}

int main(int argc, char * argv[]){
  std::cout << "Test2\n";
  ROS_INFO("Start: %s", programName.c_str());

  std::cout << "Test3\n";
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  std::cout << "Test4\n";
  node.param<string>("laserscan_topic",laserscan_topic,"/amiro/laserscan");
  node.param<string>("pointcloud2_topic",pointcloud2_topic,"/amiro/pointcloud2");
  // node.param<string>("target_frame",target_frame,"amiro/base_pointcloud");
  ROS_INFO("laserscan_topic: %s",laserscan_topic.c_str());
  ROS_INFO("pointcloud2_topic: %s",pointcloud2_topic.c_str());
  // ROS_INFO("target_frame: %s",target_frame.c_str());

  std::cout << "Test5\n";
  pointcloud2 = node.advertise<sensor_msgs::PointCloud2>(pointcloud2_topic, 1);
  ros::Subscriber laser = node.subscribe(laserscan_topic, 1, convert);
  // tfListener.setExtrapolationLimit(ros::Duration(0.1));

  std::cout << "Test6\n";
  ros::spin();
  return 0;
}
