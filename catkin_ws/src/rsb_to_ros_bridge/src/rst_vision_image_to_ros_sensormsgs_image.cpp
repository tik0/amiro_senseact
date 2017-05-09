/**
 * Author: Daniel Rudolph
 */

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
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
#include <rst/vision/EncodedImage.pb.h>

// OpenCv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

// RSB Listener Scope
static string rsbListenerScope = "/image";

// ROS Publish Topic
static string rosPublishImageTopic = "/image";
static string rosPublishCompressedImageTopic = "/image/compressed";

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;
static ros::Publisher compressedImagePublisher;

// program name
const string programName = "rst_vision_image_to_ros_sensormsgs_image";

void processImage(rsb::EventPtr event) {
  if (event->getType() == "rst::vision::Image") {
    boost::shared_ptr<rst::vision::Image> image = boost::static_pointer_cast<rst::vision::Image>(event->getData());
    cv::Mat img;
    img.create(image->height(), image->width(), CV_8UC3);
    img.data = const_cast<unsigned char *>(reinterpret_cast<const unsigned char *>(&image->data()[0])); // conversion from string to  const uchar*
    cv_bridge::CvImage cvImage;
    cvImage.header.stamp    = ros::Time::now();
    cvImage.header.frame_id = event->getScope().toString();
    cvImage.encoding        = sensor_msgs::image_encodings::BGR8;
    cvImage.image = img;
    // publish ROS message
    imagePublisher.publish(cvImage.toImageMsg());
  } else if (event->getType() == "rst::vision::EncodedImage" || event->getMetaData().getUserInfo("rsb.wire-schema") == "utf-8-string") {
    sensor_msgs::CompressedImage compressedImage;
    boost::shared_ptr<rst::vision::EncodedImage> encodedImage;
    boost::shared_ptr<std::string> stringImage;
    if (event->getType() == "rst::vision::EncodedImage") {
      encodedImage = boost::static_pointer_cast<rst::vision::EncodedImage>(event->getData());
      std::vector<unsigned char> data(encodedImage->data().begin(), encodedImage->data().end());
      compressedImage.data = data;
    }
    if (event->getMetaData().getUserInfo("rsb.wire-schema") == "utf-8-string") {
      stringImage = boost::static_pointer_cast<std::string>(event->getData());
      std::vector<unsigned char> data(stringImage->begin(), stringImage->end());
      compressedImage.data = data;
    }
    compressedImage.header.stamp    = ros::Time::now();
    compressedImage.header.frame_id = event->getScope().toString();
    compressedImage.format = "jpeg";
    compressedImagePublisher.publish(compressedImage);
  }
} // processImage

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node;

  if (node.getParam("rsbListenerScope", rsbListenerScope)) ROS_INFO("Got param rsbListenerScope: %s", rsbListenerScope.c_str());
  if (node.getParam("rosPublishImageTopic", rosPublishImageTopic)) ROS_INFO("Got param rosPublishImageTopic: %s", rosPublishImageTopic.c_str());
  if (node.getParam("rosPublishCompressedImageTopic", rosPublishCompressedImageTopic)) ROS_INFO("Got param rosPublishCompressedImageTopic: %s", rosPublishCompressedImageTopic.c_str());
  image_transport::ImageTransport imageTransport(node);
  imagePublisher = imageTransport.advertise(rosPublishImageTopic, 1);
  compressedImagePublisher = node.advertise<sensor_msgs::CompressedImage>(rosPublishCompressedImageTopic, 1);

  // Create RSB factory
  rsb::Factory& factory = rsb::getFactory();

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::Image> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::vision::Image>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::EncodedImage> >
  converter1(new rsb::converter::ProtocolBufferConverter<rst::vision::EncodedImage>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter1);

  // Prepare RSB listener
  rsb::ListenerPtr imageListener = factory.createListener(rsbListenerScope);
  imageListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processImage)));

  ros::spin();

  return 0;
}
