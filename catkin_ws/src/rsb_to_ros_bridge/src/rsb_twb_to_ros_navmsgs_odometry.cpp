// ============================================================================
// Name        : rsb_twb_to_ros_navmsgs_odometry.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve  twbTracking::proto::ObjectLists and publish each Object via ros::nav_msgs::Odometry.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Proto types
#include <types/loc.pb.h>

#include "rsb_to_ros_time_converter.hpp"

using namespace std;

// RSB Listener Scope
string rsbListenerScope;

// ROS Publish Topic
string rosPublishPoseStamped;

ros::Publisher rosPosePub;

// program name
const string programName = "rsb_twb_to_ros_navmsgs_odometry";

void euler2Quaternion(double (&euler)[3], double (&quat)[4]) {
  double c1 = cos(euler[2] / 2);
  double s1 = sin(euler[2] / 2);
  double c2 = cos(euler[1] / 2);
  double s2 = sin(euler[1] / 2);
  double c3 = cos(euler[0] / 2);
  double s3 = sin(euler[0] / 2);

  quat[0] = (double) (c1 * c2 * s3 - s1 * s2 * c3);
  quat[1] = (double) (c1 * s2 * c3 + s1 * c2 * s3);
  quat[2] = (double) (s1 * c2 * c3 - c1 * s2 * s3);
  quat[3] = (double) (c1 * c2 * c3 + s1 * s2 * s3);
}

void processTwbTrackingProtoObjectList(rsb::EventPtr event) {
  if (event->getType() != "twbTracking::proto::ObjectList") {
    return;
  }
  //
  boost::shared_ptr<twbTracking::proto::ObjectList> objectList = boost::static_pointer_cast<twbTracking::proto::ObjectList>(event->getData());

  for (int i = 0; i < objectList->object_size(); i++) {
    twbTracking::proto::Object obj = objectList->object(i);
    double rotEuler[3] = { obj.position().rotation().x(), obj.position().rotation().y(), obj.position().rotation().z() };
    double rotQuat[4];
    euler2Quaternion(rotEuler, rotQuat);
    nav_msgs::Odometry odom;
    odom.header.frame_id         = event->getScope().toString();
    odom.header.stamp            = getRosTimeFromRsbEvent(event);
    odom.child_frame_id          = std::string("base_link/") + std::to_string(obj.id());
    odom.pose.pose.position.x    = obj.position().translation().x();
    odom.pose.pose.position.y    = obj.position().translation().y();
    odom.pose.pose.position.z    = obj.position().translation().z();
    odom.pose.pose.orientation.x = rotQuat[0];
    odom.pose.pose.orientation.y = rotQuat[1];
    odom.pose.pose.orientation.z = rotQuat[2];
    odom.pose.pose.orientation.w = rotQuat[3];
    const boost::array<double, 36> covariance = { {
                                                    .1, 0, 0, 0, 0, 0,
                                                    0, .1, 0, 0, 0, 0,
                                                    0, 0, .15, 0, 0, 0,
                                                    0, 0, 0, .01, 0, 0,
                                                    0, 0, 0, 0, .01, 0,
                                                    0, 0, 0, 0, 0, .01
                                                  } };
    odom.pose.covariance = covariance;
    rosPosePub.publish(odom);
  }
} // processTwbTrackingProtoObjectList

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("rsb_listener_scope", rsbListenerScope, "/tracking/merger");
  ROS_INFO("rsb_listener_scope: %s", rsbListenerScope.c_str());
  node.param<string>("ros_publish_topic", rosPublishPoseStamped, "/tracking");
  ROS_INFO("ros_publish_topic: %s", rosPublishPoseStamped.c_str());

  rosPosePub = node.advertise<nav_msgs::Odometry>(rosPublishPoseStamped, 1);

  rsb::Factory& factory = rsb::getFactory();

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::ObjectList> >
  converter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::ObjectList>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB listener
  rsb::ListenerPtr poseListener = factory.createListener(rsbListenerScope);
  poseListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processTwbTrackingProtoObjectList)));

  ros::spin();

  return 0;
}
