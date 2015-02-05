/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 */

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
#include <rst0.11/sandbox/rst/vision/LaserScan.pb.h>

#include <gazebo_plugins/gazebo_rsb_laser.h>

#define PLUGIN_NAME "gazebo_rsb_laser"

namespace gazebo
{
bool GazeboRsbLaser::converterRegistered = false;
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
    if (GazeboRsbLaser::converterRegistered == false) {
      GazeboRsbLaser::converterRegistered = true;
      boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan> >
          converter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan>());
      rsb::converter::converterRepository<std::string>()->registerConverter(converter);
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
  // Get the world name.
  std::string worldName = _parent->GetWorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  this->parent_ray_sensor_ =
    boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRsbLaser controller requires a Ray Sensor as its parent\n");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  printf ( "Starting Laser Plugin (ns = %s)!\n", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRsbLaser::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRsbLaser::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);
  
  this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(),
                                                        &GazeboRsbLaser::OnScan, this);

  // sensor generation on by default
  this->parent_ray_sensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to RSB message and publish it
void GazeboRsbLaser::OnScan(ConstLaserScanStampedPtr &_msg)
{
  // Set the topic name of the infomer
  // Create an informer that is capable of sending events containing string data on the given scope.
  
  if (this->defineInfomerOnce == false) {
    this->defineInfomerOnce = true;
    std::string rsbScope(rsbScopeFromGazeboFrame(_msg->scan().frame()));
//     rsbScope = "/" + getParentScope(rsbScope) + "/" + getSubScope(rsbScope);
    PrintPluginInfoString ( getParentScope(rsbScope), PLUGIN_NAME, "RSB scope: " + rsbScope);
    this->informer = factory.createInformer<rst::vision::LaserScan> (rsbScope);
  }
  

  rsb::Informer<rst::vision::LaserScan>::DataPtr laserScan(new rst::vision::LaserScan);
  
  for(size_t idx = 0; idx < _msg->scan().ranges_size(); ++idx) {
    laserScan->add_scan_values(static_cast<float>(_msg->scan().ranges(idx)));
  }

  laserScan->set_scan_angle(_msg->scan().angle_max() - _msg->scan().angle_min());
  
  this->informer->publish(laserScan);

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
