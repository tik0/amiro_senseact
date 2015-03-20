
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "../../includes/MSG.h"

#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <utils.h>

#include <Eigen/Geometry>

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
//#include <rst0.11/stable/rst/vision/LaserScan.pb.h>
#include <types/LocatedLaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>

// Opencv
#include <opencv2/opencv.hpp>
#include <cvplot/cvplot.h>

using namespace utils;
using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;

static rst::geometry::Translation odomTrans;
static rst::geometry::Rotation odomRot;
static Eigen::Quaterniond quat;
static Eigen::AngleAxisd angle;
static Eigen::Vector3d trans;

#include <mutex>          // std::mutex
std::mutex map_to_odom_mutex_;
std::mutex mtxOdom;       // mutex for odometry messages

void convertDataToScan(boost::shared_ptr< rst::vision::LocatedLaserScan > data , rst::vision::LocatedLaserScan &rsbScan) {

  // Copy the whole scan
  rsbScan = *data;
}

void storeOdomData(boost::shared_ptr<rst::geometry::Pose> event) {
  mtxOdom.lock();
    odomTrans = event->translation();
    odomRot = event->rotation();
  mtxOdom.unlock();
}

bool
getOdomPose()
{
  rst::geometry::Translation translation;
  rst::geometry::Rotation rotation;
  mtxOdom.lock();
    translation = odomTrans;
    rotation = odomRot;
  mtxOdom.unlock();

//  std::cout << rotation.qw()<< rotation.qx()<< rotation.qy()<< rotation.qz() << std::endl;
  quat = Eigen::Quaterniond(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
  angle = Eigen::AngleAxisd(quat);
  trans[0] = translation.x();
  trans[1] = translation.y();
  trans[2] = translation.z();

  return true;
}

int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
  std::string odomInScope = "/AMiRo_Hokuyo/gps";
  std::string ogmoutscope = "/AMiRo_Hokuyo/ogm";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data")
    ("odominscope", po::value < std::string > (&odomInScope), "Scope for receiving odometry data")
    ("ogmoutscope", po::value < std::string > (&ogmoutscope), "Scope for sending the inverse model");

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

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();
  
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);


  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
  // Prepare RSB async listener for odometry messages
  rsb::ListenerPtr listener = factory.createListener(odomInScope);
  listener->addHandler(HandlerPtr(new DataFunctionHandler<rst::geometry::Pose> (&storeOdomData)));
  // Prepare RSB informer for sending the map as an compressed image
//  rsb::Informer<std::string>::Ptr informer = factory.createInformer<std::string> ("/image");
//  this->imageInformerEdge = factory.createInformer<std::string> ("/invModelEdge");
  rsb::Informer<std::string>::Ptr imageInformerCrop = factory.createInformer<std::string> (ogmoutscope);
//  this->imageInformerWeed = factory.createInformer<std::string> ("/invModelWeed");
//  this->imageInformerCoiler = factory.createInformer<std::string> ("/invModelCoiler");


  const int size = invertModelSize; // Define the width of the invert model
  const int depth = size / 4; // Define the depth of the invert model
  const double res = invertModelResolution;
  const int holwWidth = static_cast<int>(holWidthSize / res);

  rst::vision::LocatedLaserScan scan;
  while( true ){
    cv::Mat invertCropModel(depth, size, CV_32FC1, cv::Scalar(128));


    // Fetch a new scan and store it to scan
    convertDataToScan(lidarQueue->pop(), scan);
    getOdomPose();
    // Get the pitch of the sensor in the robot base coordinate frame
//    scan.pose().rotation().qw()
    Eigen::Quaterniond lidar_quat(scan.pose().rotation().qw(), scan.pose().rotation().qx(), scan.pose().rotation().qy(), scan.pose().rotation().qz());
    Eigen::AngleAxisd lidar_angle(lidar_quat);
    Eigen::Matrix<double,3,1> rpy = lidar_angle.toRotationMatrix().eulerAngles(0,1,2);
    double pitch = rpy(1);

    // Sensor is reading from the right to the left
    rst::vision::LocatedLaserScan scanTf(scan);  // Transformed scan to the focal lense
    scanTf.mutable_scan_values()->Clear();
    for(int idx = 0; idx < scan.scan_values().size(); ++idx) {
      scanTf.add_scan_values(static_cast<float>(scan.scan_values().Get(idx))
              * sin(pitch) /*Projection on the floor*/
              * cos(   scan.scan_angle_end() - idx * scan.scan_angle_increment())/*projection on the focal axis*/);
      // TODO check if scan order is flipped
//      std::cout << scanTf.scan_values(idx) << " " << scan.scan_values(idx) << std::endl;
    }


    ////////////////////////////////////////////////////////////////////
    // Init the presence of each feature as false
    std::vector<uchar> isFloor(scanTf.scan_values().size(), 0);
    utils::getLidarEstimations(utils::isFloorId, scanTf.scan_values().data(), isFloor, scanTf.scan_values().size());
    ////////////////////////////////////////////////////////////////////

    // Filter the edges to the floor ///////////////////////////////////
    std::vector<uchar> isEdge(scanTf.scan_values().size(), 0);
    std::vector<int> isEdgeAtIdx;
    for(size_t idx = 1 ; idx < scanTf.scan_values().size() - 2; ++idx) {
      if ((isFloor[idx-1] && !isFloor[idx]) || (isFloor[idx+1] && !isFloor[idx])) {
        isEdge[idx] = 1;
        isEdgeAtIdx.push_back(idx);
      }
    }
    ////////////////////////////////////////////////////////////////////
    

    // Declare the invert model for crop //////////////////////////////
    bool skipNextFullWidthDrawing = false;
    // Process every found crop scan
    for (uint rayIdx = 0; rayIdx < isEdge.size(); rayIdx = rayIdx + 1) {
    if (isEdge[rayIdx] == 1) {

      // Calculate the impact in the invert sensor model
      float z_m = static_cast<float>(scan.scan_values().Get(rayIdx)) * cos(scan.scan_angle_end() - rayIdx * scan.scan_angle_increment());
      float y_m = sqrt(pow(static_cast<float>(scan.scan_values().Get(rayIdx)),2) - pow(z_m,2));

      // Get the angle of incedence
      const float angleOfIncedence = -( scan.scan_angle_end() - rayIdx * scan.scan_angle_increment());

      // Look if the edge is on the right or left side of the focal axis
      if (angleOfIncedence >= 0)
        y_m = -y_m;

      // Get the index of the impact
      int zIdx = static_cast<int>(z_m / res);
      int yIdx = static_cast<int>(y_m / res);

      // Define the properties of the gaussian distribution
      cv::Mat sigma(2, 2, CV_64FC1, 0.0);
      sigma.at<double>(0,0) = (1 + 1 * abs(angleOfIncedence)) /*f(\theta)*/ * holwWidth;   // sigma_zz
      sigma.at<double>(1,1) = (1 + 0.5 * abs(angleOfIncedence)) /*g(\theta)*/ * holwWidth;   // sigma_yy

      cv::Mat mu(2, 1, CV_64FC1);
      mu.at<double>(0,0) = zIdx;
      mu.at<double>(1,0) = size / 2 + yIdx;

      // Get the 2x2 rotation of the uncertainty
      cv::Mat rotMat = getRotationMatrix2D(cv::Point2f(0,0), static_cast<double>(angleOfIncedence * 180.0 / M_PI), 1.0);
      rotMat = rotMat * cv::Mat::eye(3, 2, rotMat.type());  // Gets the 2x2 rotation matrix

      // Get the normation value
      double normConst = draw2DGauss(cv::Mat(2, 1, CV_64FC1, cv::Scalar(0.0)), cv::Mat(2, 1, CV_64FC1, cv::Scalar(0.0)), sigma);
      normConst = normConst / 128;  // Get white value

      // Draw the gaussian distribution into the model
      // Set the width end hight of drawing
      int gaussWidth = skipNextFullWidthDrawing ? 10 /* TODO should be dependent on the actual distance between rays*/ : sqrt(sigma.at<double>(1,1)) * 3;
      int gaussDepth = sqrt(sigma.at<double>(0,0)) * 3;
      if (isEdge.size() <= rayIdx + 1) skipNextFullWidthDrawing = isEdge[rayIdx+1] && isEdge[rayIdx+2] ? true : false;
      for (int holeIdxY = size / 2 + yIdx - gaussWidth; holeIdxY <= size / 2 + yIdx + gaussWidth; holeIdxY++) {
        for (int holeIdxZ = zIdx - gaussDepth; holeIdxZ <= zIdx + gaussDepth; holeIdxZ++) {
          // Define the current position x
          cv::Mat x(2, 1, CV_64FC1); x.at<double>(0,0) = static_cast<double>(holeIdxZ); x.at<double>(1,0) = static_cast<double>(holeIdxY);
          // Calculate the value
          float n = 128.0 + draw2DGauss(x, mu, rotMat * sigma) / normConst;
          // Draw the value at x only, if it is bigger, to not overwrite already drawn in impacts
          if (n > invertCropModel.at<float>(holeIdxZ, holeIdxY))
            invertCropModel.at<float>(holeIdxZ, holeIdxY) = n;
        }
      }
    }
    }
    cv::Mat flipImg(depth, size, CV_32FC1);
    flip(invertCropModel, flipImg, 0); // Flipping the map around the y axis, so that it shows up correct
    sendImage (imageInformerCrop, flipImg);

//    sendPlot("Plot", imageInformerCrop, scanTf.scan_values().data(), scanTf.scan_values().size(), 1, 255, 0, 0);
//    sendPlot("Plot", imageInformerCrop, isWeed.data(), isEdge.size(), 1, 255, 0, 0);
  }
}

