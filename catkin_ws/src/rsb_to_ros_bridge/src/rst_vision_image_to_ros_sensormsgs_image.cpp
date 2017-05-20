// ============================================================================
// Name        : main.cxx
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve images via rsb and publish them via ros.
// ============================================================================

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
static string rsbListenerScope;

// ROS Publish Topic
static string rosPublishImageTopic;
static string rosPublishCompressedImageTopic;

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;
static ros::Publisher compressedImagePublisher;

// program name
const string programName = "rst_vision_image_to_ros_sensormsgs_image";

static const string rstVisionImage        = "rst::vision::Image";
static const string rstVisionEncodedImage = "rst::vision::EncodedImage";
static const string rsbWireSchema         = "rsb.wire-schema";
static const string utf8String  = "utf-8-string";
static const string cxx11String = "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >";

void processImage(rsb::EventPtr event) {
  if (event->getType() == rstVisionImage) {
    boost::shared_ptr<rst::vision::Image> image = boost::static_pointer_cast<rst::vision::Image>(event->getData());
    cv::Mat img;
    cv_bridge::CvImage cvImage;
    cout << "channels:" << image->channels() << endl;
    if (image->depth() == rst::vision::Image::DEPTH_8U && image->channels() == 3) {
      cout << "8_3" << endl;
      img.create(image->height(), image->width(), CV_8UC3);
      cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    } else if (image->depth() == rst::vision::Image::DEPTH_8U && image->channels() == 1) {
      cout << "8_1" << endl;
      img.create(image->height(), image->width(), CV_8UC1);
      cvImage.encoding = sensor_msgs::image_encodings::MONO8;
    } else if (image->depth() == rst::vision::Image::DEPTH_16U && image->channels() == 1) {
      cout << "16" << endl;
      img.create(image->height(), image->width(), CV_16UC1);
      cvImage.encoding = sensor_msgs::image_encodings::MONO16;
    } else {
      ROS_INFO("Image encoding is not known! depth: %i, channel: %i", image->depth(), image->channels());
      return;
    }

    img.data = const_cast<unsigned char *>(reinterpret_cast<const unsigned char *>(&image->data()[0])); // conversion from string to  const uchar*
    cout << "debug1" << endl;
    cvImage.header.stamp.nsec = event->getMetaData().getCreateTime() * 1000;
    cout << "debug2" << endl;
    cvImage.header.frame_id = event->getScope().toString();
    cout << "debug3" << endl;
    cvImage.image = img;
    cout << "debug4" << endl;
    // publish ROS message
    imagePublisher.publish(cvImage.toImageMsg());
    cout << "debug5" << endl;
  } else if (event->getType() == rstVisionEncodedImage || event->getMetaData().hasUserInfo(rsbWireSchema) || event->getType() == cxx11String) {
    sensor_msgs::CompressedImage compressedImage;
    boost::shared_ptr<rst::vision::EncodedImage> encodedImage;
    boost::shared_ptr<std::string> stringImage;
    bool doPublish = false;
    if (event->getType() == rstVisionEncodedImage) {
      encodedImage = boost::static_pointer_cast<rst::vision::EncodedImage>(event->getData());
      std::vector<unsigned char> data(encodedImage->data().begin(), encodedImage->data().end());
      compressedImage.data = data;
      doPublish = true;
    } else if (event->getType() == cxx11String || event->getMetaData().getUserInfo(rsbWireSchema) == utf8String) {
      stringImage = boost::static_pointer_cast<std::string>(event->getData());
      std::vector<unsigned char> data(stringImage->begin(), stringImage->end());
      compressedImage.data = data;
      doPublish = true;
    }
    if (doPublish) {
      compressedImage.header.stamp.nsec = event->getMetaData().getCreateTime() * 1000;
      compressedImage.header.frame_id   = event->getScope().toString();
      compressedImage.format = "jpeg";
      compressedImagePublisher.publish(compressedImage);
    }
  }
} // processImage

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  private_nh.param<string>("rsbListenerScope", rsbListenerScope, "/image");
  private_nh.param<string>("rosPublishImageTopic", rosPublishImageTopic, "/image");
  private_nh.param<string>("rosPublishCompressedImageTopic", rosPublishCompressedImageTopic, "/image/compressed");

  ROS_INFO("rsbListenerScope: %s", rsbListenerScope.c_str());
  ROS_INFO("rosPublishImageTopic: %s", rosPublishImageTopic.c_str());
  ROS_INFO("rosPublishCompressedImageTopic: %s", rosPublishCompressedImageTopic.c_str());

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
} // main
