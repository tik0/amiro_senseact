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

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Proto types
//#include <rst/vision/LaserScan.pb.h>
#include <types/LocatedLaserScan.pb.h>

#include <gazebo_plugins/gazebo_rsb_laser.h>

#define PLUGIN_NAME "gazebo_rsb_laser"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRsbLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRsbLaser::GazeboRsbLaser()
{
  this->seed = 0;

  // Register new converter for RSB
  GazeboRsbLaser::registerConverter();
}

void GazeboRsbLaser::registerConverter() {
  try {
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan> >
        converter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converter);
  } catch (...) {

  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRsbLaser::~GazeboRsbLaser()
{
//   this->rosnode_->shutdown();
//   delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRsbLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get the world name
  std::string worldName = _parent->GetWorldName();
  // save pointers
  this->parent = _parent;
  this->world = physics::get_world(worldName);
  this->sdf = _sdf;
  this->parent_ray_sensor = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
  this->parent_link = world->GetEntity(this->parent_ray_sensor->GetParentName());

  if (!this->parent_ray_sensor)
    gzthrow("GazeboRsbLaser controller requires a Ray Sensor as its parent\n");

  // RSB
  std::string modelName(getFirstScopeContains(_parent->GetParentName(), std::string("AMiRo")));
  this->rsbScope = rsbScopeFromGazeboFrame(modelName) + rsbScopeFromGazeboFrame(GetSdfElementValue(modelName, _sdf, "lidarSubscope", PLUGIN_NAME));
  PrintPluginInfoString ( modelName, PLUGIN_NAME, "RSB scope: " + this->rsbScope);
  this->informer = factory.createInformer<rst::vision::LocatedLaserScan> (this->rsbScope);

  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRsbLaser::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRsbLaser::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name);

  this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_ray_sensor->GetTopic(),
                                                        &GazeboRsbLaser::OnScan, this);

  // sensor generation on by default
  this->parent_ray_sensor->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to RSB message and publish it
void GazeboRsbLaser::OnScan(ConstLaserScanStampedPtr &_msg)
{

  rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScan(new rst::vision::LocatedLaserScan);
  
  for(size_t idx = 0; idx < _msg->scan().ranges_size(); ++idx) {
    laserScan->add_scan_values(static_cast<float>(_msg->scan().ranges(idx)));
  }
  laserScan->set_scan_angle(_msg->scan().angle_max() - _msg->scan().angle_min());
  laserScan->set_scan_angle_start(_msg->scan().angle_min());
  laserScan->set_scan_angle_end(_msg->scan().angle_max());
  laserScan->set_scan_values_min(_msg->scan().range_min());
  laserScan->set_scan_values_max(_msg->scan().range_max());
  laserScan->set_scan_angle_increment(_msg->scan().angle_step());

  // HACK TODO Set the angles from the original pose
  math::Pose poseHack;
  math::Vector3 positionHack(0,0,0);
  math::Vector3 rotationHack(0,0.707,0);
  poseHack.Set(positionHack, rotationHack);
  laserScan->mutable_pose()->mutable_rotation()->set_qw(this->parent_ray_sensor->GetPose().rot.w);
  laserScan->mutable_pose()->mutable_rotation()->set_qx(this->parent_ray_sensor->GetPose().rot.x);
  laserScan->mutable_pose()->mutable_rotation()->set_qy(this->parent_ray_sensor->GetPose().rot.y);
  laserScan->mutable_pose()->mutable_rotation()->set_qz(this->parent_ray_sensor->GetPose().rot.z);
  laserScan->mutable_pose()->mutable_translation()->set_x(this->parent_ray_sensor->GetPose().pos.x);
  laserScan->mutable_pose()->mutable_translation()->set_y(this->parent_ray_sensor->GetPose().pos.y);
  laserScan->mutable_pose()->mutable_translation()->set_z(this->parent_ray_sensor->GetPose().pos.z);
//  std::cout << this->parent_link->GetWorldPose().rot.w << this->parent_link->GetWorldPose().rot.x<< this->parent_link->GetWorldPose().rot.y<< this->parent_link->GetWorldPose().rot.z << std::endl;

//  std::cout << laserScan->pose().rotation().qw() << laserScan->pose().rotation().qx()<< laserScan->pose().rotation().qy()<< laserScan->pose().rotation().qz() << std::endl;
//  std::cout <<   this->parent_link->GetWorldPose().rot.GetAsEuler().y << std::endl;

//  std::cout << parent_link->GetWorldPose().rot.GetAsEuler().y << std::endl;

  this->informer->publish(laserScan);

}


}
