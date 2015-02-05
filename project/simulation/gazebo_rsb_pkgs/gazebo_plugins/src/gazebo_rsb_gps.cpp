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
#include <gazebo/sensors/GpsSensor.hh>
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
#include <rst0.11/stable/rst/geometry/Pose.pb.h>

#include <gazebo_plugins/gazebo_rsb_gps.h>

#define PLUGIN_NAME "gazebo_rsb_gps"

namespace gazebo
{
bool GazeboRsbGps::converterRegistered = false;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRsbGps::GazeboRsbGps()
{
  std::cout << "-------------------------------sadjiandsklansdokansod" << std::flush << std::endl;
  // Register new converter for RSB
  GazeboRsbGps::registerConverter();
  std::cout << "-------------------------------sadjiandsklansdokansod" << std::flush << std::endl;
}

void GazeboRsbGps::registerConverter() {
    if (GazeboRsbGps::converterRegistered == false) {
      GazeboRsbGps::converterRegistered = true;
      boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
          converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
      rsb::converter::converterRepository<std::string>()->registerConverter(converter);
  }
}

void GazeboRsbGps::Reset()
{
  updateTimer.Reset();
//   position_error_model_.reset();
//   velocity_error_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRsbGps::~GazeboRsbGps()
{
//   this->rosnode_->shutdown();
//   delete this->rosnode_;
}


////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRsbGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "-------------------------------sadjiandsklansdokansod" << std::flush << std::endl;
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }

//   if (!link)
//   {
//     ROS_FATAL("GazeboRosGps plugin error: bodyName: %s does not exist\n", link_name_.c_str());
//     return;
//   }

  // default parameters
  frame_id_ = "/world";
  fix_topic_ = "fix";
  velocity_topic_ = "fix_velocity";

  reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
  reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
  reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

//   fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
//   fix_.status.service = sensor_msgs::NavSatStatus::STATUS_FIX;

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    fix_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("velocityTopicName"))
    velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("referenceLatitude"))
    _sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);

  if (_sdf->HasElement("referenceLongitude"))
    _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);

  if (_sdf->HasElement("referenceHeading"))
    if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
      reference_heading_ *= M_PI/180.0;

  if (_sdf->HasElement("referenceAltitude"))
    _sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);

//   if (_sdf->HasElement("status"))
//     _sdf->GetElement("status")->GetValue()->Get(fix_.status.status);

//   if (_sdf->HasElement("service"))
//     _sdf->GetElement("service")->GetValue()->Get(fix_.status.service);

//   fix_.header.frame_id = frame_id_;
//   velocity_.header.frame_id = frame_id_;

//   position_error_model_.Load(_sdf);
//   velocity_error_model_.Load(_sdf, "velocity");

  // calculate earth radii
  double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
  double prime_vertical_radius = equatorial_radius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
  radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

  // RSB
//     std::string rsbScope(rsbScopeFromGazeboFrame(_msg->scan().frame()));
//     PrintPluginInfoString ( getParentScope(rsbScope), PLUGIN_NAME, "RSB scope: " + rsbScope);
    this->informer = factory.createInformer<rst::geometry::Pose> (/*rsbScope*/ "/TEST");


  // connect Update function
//   updateTimer.setUpdateRate(4.0);
//   updateTimer.Load(world, _sdf);
//   updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGps::Update, this));
    
    // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRsbGps::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRsbGps::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  math::Pose pose = link->GetWorldPose();

  rsb::Informer<rst::geometry::Pose>::DataPtr gpsData(new rst::geometry::Pose);
  
  rst::geometry::Translation *translation = gpsData->mutable_translation();
  rst::geometry::Rotation *rotation = gpsData->mutable_rotation();
  translation->set_x(pose.pos.x);
  translation->set_y(pose.pos.y);
  translation->set_z(pose.pos.z);
  rotation->set_qw(pose.rot.w);
  rotation->set_qx(pose.rot.x);
  rotation->set_qy(pose.rot.y);
  rotation->set_qz(pose.rot.z);

  this->informer->publish(gpsData);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRsbGps)


}
