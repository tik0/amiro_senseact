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
  this->imageInformerEdge = factory.createInformer<std::string> ("/invModelEdge");
  this->imageInformerCrop = factory.createInformer<std::string> ("/invModelCrop");
  this->imageInformerWeed = factory.createInformer<std::string> ("/invModelWeed");
  this->imageInformerCoiler = factory.createInformer<std::string> ("/invModelCoiler");

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
  std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl << std::flush;

  // Declare the invert model for edges //////////////////////////////
  const int size = 1024; // Define the width of the invert model
  const int depth = size / 4; // Define the depth of the invert model
  const float res = 0.001;
  const int holwWidth = static_cast<int>(0.1 /*m*/ / res);
  cv::Mat invertEdgeModel(depth, size, CV_32FC1, cv::Scalar(128));

  // Declare the invert model for coiler //////////////////////////////
//  const int size = 1024; // Define the width of the invert model
//  const int depth = size / 4; // Define the depth of the invert model
//  const float res = 0.001;
//  const int holwWidth = static_cast<int>(0.1 /*m*/ / res);
  cv::Mat invertCoilerModel(depth, size, CV_32FC1, cv::Scalar(128));
  const int coilerWidth = 0.5 /*m*/ / res;

  // Process the with of the coiler

  cv::Point up( size / 2 + coilerWidth / 2, depth / 2 + 10 );
  cv::Point dp( size / 2 - coilerWidth / 2, depth / 2 - 10 );
  rectangle(invertCoilerModel, dp, up, cv::Scalar( 0, 0, 0) , -1, 8, 0 );
  for ( int i = 1; i < 10 /*MAX_KERNEL_LENGTH*/; i = i + 2 ) {
    GaussianBlur( invertCoilerModel, invertCoilerModel, cv::Size( i, i ), 0, 0 );
  }
  ////////////////////////////////////////////////////////////////////

  cv::Mat flipImg(invertEdgeModel.rows, invertEdgeModel.cols, invertEdgeModel.type());
//  flip(invertEdgeModel, flipImg, 0); // Flipping the map around the y axis, so that it shows up correct
//  sendImage (this->imageInformerEdge, flipImg);
//  flip(invertCropModel, flipImg, 0); // Flipping the map around the y axis, so that it shows up correct
//  sendImage (this->imageInformerCrop, flipImg);
//  flip(invertWeedModel, flipImg, 0); // Flipping the map around the y axis, so that it shows up correct
//  sendImage (this->imageInformerWeed, flipImg);
  flip(invertCoilerModel, flipImg, 0); // Flipping the map around the y axis, so that it shows up correct
  sendImage (this->imageInformerCoiler, flipImg);





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


//  laserScan->set_scan_angle(_msg->scan().angle_max() - _msg->scan().angle_min());
//  laserScan->set_scan_angle_start(_msg->scan().angle_min());
//  laserScan->set_scan_angle_end(_msg->scan().angle_max());
//  laserScan->set_scan_values_min(_msg->scan().range_min());
//  laserScan->set_scan_values_max(_msg->scan().range_max());
//  laserScan->set_scan_angle_increment(_msg->scan().angle_step());
  
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
