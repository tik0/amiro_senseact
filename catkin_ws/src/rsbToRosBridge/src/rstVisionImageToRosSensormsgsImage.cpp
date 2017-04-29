/**
 * Author: Daniel Rudolph
 */

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/vision/Image.pb.h>

#include "Constants.hpp";

using namespace std;

// RSB Listener Scope
const string rsbListenerScope = constants::rsbImageScope;

// ROS Publish Topic
const string rosPublishTopic = constants::rosImageTopic;
ros::Publisher rosPublisher;

// program name
const string programName = "rstVisionImageToRosSensormsgsImage";

// frame id
const string frameID = "";

// Rsb event type
const string rsbEventType = "rst::vision::Image";

void processImage(rsb::EventPtr event) {

  if(event->getType() != rsbEventType) {
    return;
  }

  boost::shared_ptr<rst::vision::Image> image = boost::static_pointer_cast<rst::vision::Image>(event->getData());
  sensor_msgs::Image Image_msg;

  Image_msg.header.stamp = ros::Time::now();
  Image_msg.header.frame_id = frameID;

	Image_msg.height = image->height();
	Image_msg.width = image->width();
	Image_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	// Image_msg.data = image->data();

  // publish ROS message
  rosPublisher.publish(Image_msg);
}

int main(int argc, char *argv[]) {
  cout << "Start: " << programName << endl;

	// Init ROS
	ros::init(argc, argv, programName);
  ros::NodeHandle node;
  rosPublisher = node.advertise<sensor_msgs::Image>(rosPublishTopic, 1);

	// Create RSB factory
  #if RSB_VERSION_NUMERIC<1200
    rsb::Factory& factory = rsb::Factory::getInstance();
  #else
    rsb::Factory& factory = rsb::getFactory();
  #endif

  // Prepare RSB listener
  rsb::ListenerPtr imageListener = factory.createListener(rsbListenerScope);
  imageListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processImage)));

	ros::spin();

 	return 0;
}
