#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_rsb_utils.h>

// Opencv
#include <opencv2/opencv.hpp>
#include <cvplot/cvplot.h>

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Proto types
//#include <rst0.11/stable/rst/vision/LaserScan.pb.h>
#include <rst0.11/stable/rst/navigation/OccupancyGrid2DInt.pb.h>
#include <types/LocatedLaserScan.pb.h>

#include <gazebo_plugins/gazebo_rsb_laser_inversemodel_edge.h>

#define PLUGIN_NAME "gazebo_rsb_laser"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRsbLaserInversemodelEdge)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRsbLaserInversemodelEdge::GazeboRsbLaserInversemodelEdge()
{
  this->seed = 0;

  // Register new converter for RSB
  GazeboRsbLaserInversemodelEdge::registerConverter();
}

void GazeboRsbLaserInversemodelEdge::registerConverter() {
  try {
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan> >
        converter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converter);
  } catch (...) {

  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRsbLaserInversemodelEdge::~GazeboRsbLaserInversemodelEdge()
{
//   this->rosnode_->shutdown();
//   delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRsbLaserInversemodelEdge::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{

  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get the world name
  std::string worldName = _parent->GetWorldName();
  // save pointers
  this->world = physics::get_world(worldName);
  this->sdf = _sdf;
  this->parent_ray_sensor = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
  math::Vector3 lidarPose = parent_ray_sensor->GetPose().rot.GetAsEuler();

  physics::EntityPtr parent = world->GetEntity(this->parent_ray_sensor->GetParentName());
    std::cout << "Pose: " << parent->GetRelativePose() << ", " << parent->GetWorldPose() << std::endl;
    std::cout << "Parent: " << parent_ray_sensor->GetParentName() << std::endl;
//  parent_ray_sensor->GetParentId();
  std::cout << "My LIDAR Pose: " << parent_ray_sensor->GetPose().rot.GetAsEuler() << ", " << parent_ray_sensor->GetPose().pos << std::endl;


  if (!this->parent_ray_sensor)
    gzthrow("GazeboRsbLaserInversemodelEdge controller requires a Ray Sensor as its parent\n");


  // RSB
  std::string modelName(getFirstScopeContains(_parent->GetParentName(), std::string("AMiRo")));
  this->rsbScope = rsbScopeFromGazeboFrame(modelName) + rsbScopeFromGazeboFrame(GetSdfElementValue(modelName, _sdf, "lidarSubscope", PLUGIN_NAME));
  PrintPluginInfoString ( modelName, PLUGIN_NAME, "RSB scope: " + this->rsbScope);
  this->informer = factory.createInformer<rst::vision::LocatedLaserScan> (this->rsbScope);
  this->imageInformer = factory.createInformer<std::string> ("/lidarPlot");

  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRsbLaserInversemodelEdge::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRsbLaserInversemodelEdge::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name);

  this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_ray_sensor->GetTopic(),
                                                        &GazeboRsbLaserInversemodelEdge::OnScan, this);

  // sensor generation on by default
  this->parent_ray_sensor->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to RSB message and publish it
void GazeboRsbLaserInversemodelEdge::OnScan(ConstLaserScanStampedPtr &_msg)
{

  // Transformation to the focal lense ///////////////////////////////
  rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScan(new rst::vision::LocatedLaserScan);
  
  // Get the pitch of the sensor, to calculate the projected distance on the floor
  physics::EntityPtr parent = world->GetEntity(this->parent_ray_sensor->GetParentName());
  double pitch = sin(parent->GetWorldPose().rot.GetAsEuler().y);

  // Sensor is reading from the right to the left
  for(size_t idx = 0; idx < _msg->scan().ranges_size(); ++idx) {
    laserScan->add_scan_values(static_cast<float>(_msg->scan().ranges(idx))
            * sin(pitch) /*Projection on the floor*/
            * cos(_msg->scan().angle_max() - idx * _msg->scan().angle_step())/*projection on the focal axis*/);
  }

  ////////////////////////////////////////////////////////////////////

  // ML estimator for the different {Weed, Crop, Floor} features /////
  double muWeed  = 0.0627; /*m*/
  double muCrop  = 0.07277; /*m*/
  double muFloor = 0.116286; /*m*/
  double sigmaLidar = 0.02; /*m²:  s.t. noise of the lidar*/

  // Init the presence of each feature as false
  std::vector<uchar> isWeed(_msg->scan().ranges_size(), 0);
  std::vector<uchar> isCrop(_msg->scan().ranges_size(), 0);
  std::vector<uchar> isFloor(_msg->scan().ranges_size(), 0);

  for(size_t idx = 0; idx < _msg->scan().ranges_size(); ++idx) {
    double nWeed = draw1DGauss(laserScan->scan_values().Get(idx), muWeed, sigmaLidar);
    double nCrop = draw1DGauss(laserScan->scan_values().Get(idx), muCrop, sigmaLidar);
    double nFloor = draw1DGauss(laserScan->scan_values().Get(idx), muFloor, sigmaLidar);
    if (nWeed > nCrop /* && nWeed > nFloor */){
      isWeed[idx] = 1;
    } else if (nWeed > nFloor) {
      isCrop[idx] = 1;
    } else /*nWeed < nFloor*/ {
      isFloor[idx] = 1;
    }
  }
  ////////////////////////////////////////////////////////////////////


  // Filter the edges to the floor ///////////////////////////////////
  std::vector<uchar> isEdge(_msg->scan().ranges_size(), 0);
  std::vector<int> isEdgeAtIdx;
  for(size_t idx = 1 ; idx < _msg->scan().ranges_size() - 2; ++idx) {
    if ((isFloor[idx-1] && !isFloor[idx]) || (isFloor[idx+1] && !isFloor[idx])) {
      isEdge[idx] = 1;
      isEdgeAtIdx.push_back(idx);
    }
  }
  ////////////////////////////////////////////////////////////////////

  // Declare the invert model ////////////////////////////////////////
  const int size = 1024; // Define the width of the invert model
  const int depth = size / 4; // Define the depth of the invert model
  const float res = 0.001;
  const int holwWidth = static_cast<int>(0.1 /*m*/ / res);
  cv::Mat invertEdgeModel(depth, size, CV_32FC1, cv::Scalar(128));

  // Process every found edge
  for (int edgeIdx = 0; edgeIdx < isEdgeAtIdx.size(); ++edgeIdx) {

    // Calculate the impact in the invert sensor model
    float z_m = static_cast<float>(_msg->scan().ranges(isEdgeAtIdx[edgeIdx])) * cos(_msg->scan().angle_max() - isEdgeAtIdx[edgeIdx] * _msg->scan().angle_step());
    float y_m = sqrt(pow(static_cast<float>(_msg->scan().ranges(isEdgeAtIdx[edgeIdx])),2) - pow(z_m,2));

    // Get the angle of incedence
    const float angleOfIncedence = -( _msg->scan().angle_max() - isEdgeAtIdx[edgeIdx] * _msg->scan().angle_step());

    // Look if the edge is on the right or left side of the focal axis
    if (angleOfIncedence >= 0)
      y_m = -y_m;

    // Get the index of the impact
    int zIdx = static_cast<int>(z_m / res);
    int yIdx = static_cast<int>(y_m / res);

    // Define the properties of the gaussian distribution
    cv::Mat sigma(2, 2, CV_64FC1, 0.0);
    sigma.at<double>(0,0) = (1 + 3 * abs(angleOfIncedence)) /*f(\theta)*/ * holwWidth;   // sigma_zz
    sigma.at<double>(1,1) = (1 + 1 * abs(angleOfIncedence)) /*g(\theta)*/ * holwWidth;   // sigma_yy

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
    for (int holeIdxY = size / 2 + yIdx -sqrt(sigma.at<double>(0,0)) * 4; holeIdxY <= size / 2 + yIdx + sqrt(sigma.at<double>(0,0)) * 4; holeIdxY++) {
      for (int holeIdxZ = zIdx - sqrt(sigma.at<double>(1,1)) * 4; holeIdxZ <= zIdx + sqrt(sigma.at<double>(1,1)) * 4; holeIdxZ++) {
        // Define the current position x
        cv::Mat x(2, 1, CV_64FC1); x.at<double>(0,0) = static_cast<double>(holeIdxZ); x.at<double>(1,0) = static_cast<double>(holeIdxY);
        // Calculate the value
        float n = 128.0 + draw2DGauss(x, mu, rotMat * sigma) / normConst;
        // Draw the value at x only, if it is bigger, to not overwrite already drawn in impacts
        if (n > invertEdgeModel.at<float>(holeIdxZ, holeIdxY))
          invertEdgeModel.at<float>(holeIdxZ, holeIdxY) = n;
      }
    }
  }
  ////////////////////////////////////////////////////////////////////


  cv::Mat flipImg(invertEdgeModel.rows, invertEdgeModel.cols, invertEdgeModel.type()); flip(invertEdgeModel, flipImg, 0); // Flipping the map around the y axis, so that it shows up correct
  sendImage (this->imageInformer, flipImg);






  // Delete first and last value
//  temp[0] = temp[1];
//  temp[temp.size()-1] = temp[temp.size()-2];
//  sendPlot("Plot", this->imageInformer, temp.data(), laserScan->scan_values().size(), 1, 255, 0, 0);
//  sendPlot("Plot", this->imageInformer, laserScan->scan_values().data(), laserScan->scan_values().size(), 1, 255, 0, 0);
//  sendPlot("Plot", this->imageInformer, isEdge.data(), laserScan->scan_values().size(), 1, 255, 0, 0);

//  {
//  int maxId = 0; float maxValue = -9999;
//  for(size_t idx = 1 ; idx < _msg->scan().ranges_size() - 1; ++idx) {
//    if (laserScan->scan_values().Get(maxId) < laserScan->scan_values().Get(idx)) {
//      maxValue = laserScan->scan_values().Get(idx);maxId = idx;
//    }
//  }
//  std::cout << "maxOut " << maxValue << std::endl;
//
//  int minId = 0; float minValue = 9999;
//    for(size_t idx = 1 ; idx < _msg->scan().ranges_size() - 1; ++idx) {
//      if (laserScan->scan_values().Get(minId) > laserScan->scan_values().Get(idx)) {
//        minValue = laserScan->scan_values().Get(idx);minId = idx;
//      }
//    }
//  std::cout << "minOut " << minValue << std::endl;
//  }


  laserScan->set_scan_angle(_msg->scan().angle_max() - _msg->scan().angle_min());
  laserScan->set_scan_angle_start(_msg->scan().angle_min());
  laserScan->set_scan_angle_end(_msg->scan().angle_max());
  laserScan->set_scan_values_min(_msg->scan().range_min());
  laserScan->set_scan_values_max(_msg->scan().range_max());
  laserScan->set_scan_angle_increment(_msg->scan().angle_step());
  
//  this->informer->publish(laserScan);


//   for (auto c : intensities)
//     std::cout << c << ' ';
// 
//   std::vector<double> intensities;
//   
//   intensities.resize(_msg->scan().ranges_size());
//   std::copy(_msg->scan().intensities().begin(),
//             _msg->scan().intensities().end(),
//             intensities.begin());
//   std::cout << intensities[0] << ' ' << std::endl;
//   std::cout << _msg->scan().frame() << std::endl;
//   std::cout << _msg->scan().ranges(0) << std::endl;
//   std::cout << "++++++++++++" << std::endl;
//   std::cout << "rMin" << _msg->scan().range_min() << " rMax" << _msg->scan().range_max() << std::endl;
//   for (auto c : intensities)
//     std::cout << c << ' ';
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding RSB message and publish it.
//   sensor_msgs::LaserScan laser_msg;
//   laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
//   laser_msg.header.frame_id = this->frame_name_;
//   laser_msg.angle_min = _msg->scan().angle_min();
//   laser_msg.angle_max = _msg->scan().angle_max();
//   laser_msg.angle_increment = _msg->scan().angle_step();
//   laser_msg.time_increment = 0;  // instantaneous simulator scan
//   laser_msg.scan_time = 0;  // not sure whether this is correct
//   laser_msg.range_min = _msg->scan().range_min();
//   laser_msg.range_max = _msg->scan().range_max();
//   laser_msg.ranges.resize(_msg->scan().ranges_size());
//   std::copy(_msg->scan().ranges().begin(),
//             _msg->scan().ranges().end(),
//             laser_msg.ranges.begin());
//   laser_msg.intensities.resize(_msg->scan().intensities_size());
//   std::copy(_msg->scan().intensities().begin(),
//             _msg->scan().intensities().end(),
//             laser_msg.intensities.begin());
//   this->pub_queue_->push(laser_msg, this->pub_);
}


}
