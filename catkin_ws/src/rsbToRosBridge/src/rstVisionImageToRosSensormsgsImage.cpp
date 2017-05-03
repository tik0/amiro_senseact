/**
 * Author: Daniel Rudolph
 */

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// ROS - OpenCV Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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

//OpenCv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Constants.hpp"

using namespace std;

// RSB Listener Scope
const string rsbListenerScope = constants::rsbImageScope;

// ROS Publish Topic
const string rosPublishTopic = constants::rosImageTopic;
// ros::Publisher rosPublisher;
image_transport::Publisher imagePublisher;

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

	cv::Mat img;
	img.create(image->height(), image->width(), CV_8UC3);
	img.data =  const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(&image->data()[0]));  // conversion from string to  const uchar*
	cv_bridge::CvImage cvImage;
	cvImage.header.stamp = ros::Time::now();
	cvImage.header.frame_id = frameID;
	cvImage.encoding = sensor_msgs::image_encodings::BGR8;
	cvImage.image = img;
  // publish ROS message
  imagePublisher.publish(cvImage.toImageMsg());
}

int main(int argc, char *argv[]) {
  cout << "Start: " << programName << endl;

	// Init ROS
	ros::init(argc, argv, programName);
  ros::NodeHandle node;
  // imagePublisher = node.advertise(rosPublishTopic, 1);
  image_transport::ImageTransport imageTransport(node);
	imagePublisher = imageTransport.advertise(rosPublishTopic, 1);

	// Create RSB factory
  #if RSB_VERSION_NUMERIC<1200
    rsb::Factory& factory = rsb::Factory::getInstance();
  #else
    rsb::Factory& factory = rsb::getFactory();
  #endif

	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::Image> >
		converter(new rsb::converter::ProtocolBufferConverter<rst::vision::Image>());
		rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB listener
  rsb::ListenerPtr imageListener = factory.createListener(rsbListenerScope);
  imageListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processImage)));

	ros::spin();

 	return 0;
}
