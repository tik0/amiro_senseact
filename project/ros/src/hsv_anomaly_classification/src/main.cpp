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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// Do we need this?
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <rosgraph_msgs/Log.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <rosapi/Topics.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <ros/duration.h>

ros::Publisher publisher;

// OpenCV
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>  // resize

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

// Boost
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>


// HSV classification
#include "../include/hsvAnomalyClassification/rt_nonfinite.h"
#include "../include/hsvAnomalyClassification/hsvAnomalyClassification.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "../include/hsvAnomalyClassification/rtwtypes.h"
#include "../include/hsvAnomalyClassification/hsvAnomalyClassification_types.h"
#include "../include/hsvAnomalyClassification/hsvAnomalyClassification_terminate.h"
#include "../include/hsvAnomalyClassification/hsvAnomalyClassification_initialize.h"
static double dv1[1920000];
static double b_y1[640000];

#include <utils.h>

static std::string inputTopic, outputTopic;
static double upper_boundary, lower_boundary;
static signed char lower_boundary_schar, upper_boundary_schar;
static int debug;

static const cv::Size2i imageSize(800,800);
sensor_msgs::Image msgClass;
cv::Mat img(imageSize.height,imageSize.width, CV_8UC3);

void callback(const sensor_msgs::Image::ConstPtr &msg) {

  // Copy
  msgClass.header = msg->header;
  msgClass.height = msg->height;
  msgClass.width = msg->width;
  msgClass.is_bigendian = msg->is_bigendian;
  msgClass.step = msg->width;
  msgClass.encoding = sensor_msgs::image_encodings::MONO8;

  // Conversion to HSV
  cv::Mat imageBGR, imageHSV;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding); // sensor_msgs::image_encodings::BGR8
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (sensor_msgs::image_encodings::isMono(msg->encoding)) {
      ROS_DEBUG("MONO");
      if (cv_ptr->image.channels() == 1) {
        cv::cvtColor(cv_ptr->image, imageBGR, cv::COLOR_GRAY2BGR);
      } else if (cv_ptr->image.channels() == 3) {
          cv_ptr->image.copyTo(imageBGR);
      } else {
        ROS_ERROR("Unknown channel size for MONO image: %d", cv_ptr->image.channels());
        return;
      }
  } else if (sensor_msgs::image_encodings::isColor(msg->encoding)) {
      ROS_DEBUG("RGB/BGR");
      if (!msg->encoding.compare(sensor_msgs::image_encodings::RGB8)) {
          cv::cvtColor(cv_ptr->image, imageBGR, cv::COLOR_RGB2BGR);
      } else if (!msg->encoding.compare(sensor_msgs::image_encodings::BGR8)) {
          cv_ptr->image.copyTo(imageBGR);
      } else {
          ROS_ERROR("No known conversion for %s", msg->encoding.c_str());
          return;
      }
  } else {
      ROS_ERROR("No known conversion for %s", msg->encoding.c_str());
      return;
  }

  cv::cvtColor(imageBGR, imageHSV, cv::COLOR_BGR2HSV);
  imageHSV.copyTo(img);

  // Sanity check
  if (imageHSV.cols != imageSize.width || imageHSV.rows != imageSize.height) {
      ROS_ERROR("Invalid size image(%d,%d) != desired(%d, %d)", imageHSV.cols, imageHSV.rows, imageSize.width, imageSize.height);
      return;
  }

  // Classify
  ROS_DEBUG("HSV Format %s", utils::conversion::format2str(imageHSV.type()).c_str());

  cv::Mat imageClassified(imageSize.width, imageSize.height, CV_8UC1);
  ROS_DEBUG("HSV content (0) %f, %f, %f\n", double(imageHSV.at<uchar>(0)), double(imageHSV.at<uchar>(1)), double(imageHSV.at<uchar>(2)));

  cv::Mat imageHSVSplit[3]; // OpenCV is always interleaved, and we need non interleaved
  cv::split(imageHSV,imageHSVSplit); // split source
  for (int idy = 0; idy < 3; ++idy) {
    for (int idx = 0; idx < imageSize.width * imageSize.height; ++idx) {
        dv1[idx + idy * imageSize.width * imageSize.height] = double(imageHSVSplit[idy].at<uchar>(idx)) / 255.0;
    }
  }

  hsvAnomalyClassification(dv1, b_y1);


  // Copy result to message
  msgClass.data.resize(imageSize.width * imageSize.height);
  for (int idx = 0; idx < imageSize.width * imageSize.height; ++idx) {
    msgClass.data.at(idx) = b_y1[idx] > 0.5 ? upper_boundary_schar : lower_boundary_schar;
  }

  // Publish
  publisher.publish(msgClass);
}

int main(int argc, char **argv){

  hsvAnomalyClassification_initialize();

  // ROS
  ros::init(argc, argv, "hsvAnomalyClassification");
  ros::NodeHandle n("~");

  n.param<std::string>("input_topic", inputTopic, "");
  n.param<std::string>("output_topic", outputTopic, "");
  n.param<double>("upperBoundary", upper_boundary, 0.8);
  n.param<double>("lowerBoundary", lower_boundary, 0.2);
  n.param<int>("debug", debug, 0); // Enable debug outputs

  upper_boundary_schar = schar(upper_boundary * 100.0);
  lower_boundary_schar = schar(lower_boundary * 100.0);

  ros::Subscriber subscriber = n.subscribe<sensor_msgs::Image>(inputTopic, 2, callback);
  publisher = n.advertise<sensor_msgs::Image>(outputTopic, 2);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate rate(5);
  while (ros::ok()) {
      if (debug) {
        cv::imshow("test", img);
        cv::waitKey(1);
      }
      rate.sleep();
  }
  return 0;
}
