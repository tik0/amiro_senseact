// ============================================================================
// Name        : main.cxx
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve images via rsb and publish them via ros.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/vision/LaserScan.pb.h>

using namespace std;

// RSB Listener Scope
static string rsbListenerScope;

// ROS Publish Topic
static string rosPublishLaserScanTopic;

// ros::Publisher rosPublisher;
static ros::Publisher laserScanPublisher;

// program name
const string programName = "rst_vision_laserscan_to_ros_sensormsgs_Laserscan";

static const string rstLaserScan = "rst::vision::LaserScan";

void processLaserScan(rsb::EventPtr event) {
  // cout << event->getData() << endl;
  if (event->getType() == rstLaserScan) {
    boost::shared_ptr<rst::vision::LaserScan> rsbLaserScan = boost::static_pointer_cast<rst::vision::LaserScan>(event->getData());
    sensor_msgs::LaserScan rosLaserScan;
    // rosLaserScan.header.stamp    = event->getMetaData().getCreationTime();
    // cout << event->getMetaData() << endl;
    rosLaserScan.header.frame_id = event->getScope().toString();

    rosLaserScan.angle_min = 0;
    rosLaserScan.angle_max = rsbLaserScan->scan_angle();
    rosLaserScan.angle_increment =  rsbLaserScan->scan_angle() / rsbLaserScan->scan_values().size();
    // rosLaserScan.time_increment
    // rosLaserScan.scan_time
    // rosLaserScan.intensities

    rosLaserScan.range_max = rsbLaserScan->scan_values(0);
    rosLaserScan.range_min = rsbLaserScan->scan_values(0);
    rosLaserScan.ranges.resize(rsbLaserScan->scan_values().size());
    for (int i = 0; i < rsbLaserScan->scan_values().size(); i++) {
      float value = rsbLaserScan->scan_values(i);
      rosLaserScan.ranges[i] = value;
      if(rosLaserScan.range_min > value) rosLaserScan.range_min = value;
      if(rosLaserScan.range_max < value) rosLaserScan.range_max = value;
    }
    // cout << "min value: " << rosLaserScan.range_min << " max value: " << rosLaserScan.range_max << "angle incre: " << rosLaserScan.angle_increment <<  endl;
    // publish ROS message
    laserScanPublisher.publish(rosLaserScan);
  }
} // processLaserScan

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  private_nh.param<string>("rsbListenerScope", rsbListenerScope, "/laserScan");
  private_nh.param<string>("rosPublishLaserScanTopic", rosPublishLaserScanTopic, "/laserScan");

  ROS_INFO("rsbListenerScope: %s", rsbListenerScope.c_str());
  ROS_INFO("rosPublishLaserScanTopic: %s", rosPublishLaserScanTopic.c_str());

  laserScanPublisher = node.advertise<sensor_msgs::LaserScan>(rosPublishLaserScanTopic, 1);

  // Create RSB factory
  rsb::Factory& factory = rsb::getFactory();

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB listener
  rsb::ListenerPtr laserScanListener = factory.createListener(rsbListenerScope);
  laserScanListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processLaserScan)));

  ros::spin();

  return 0;
} // main
