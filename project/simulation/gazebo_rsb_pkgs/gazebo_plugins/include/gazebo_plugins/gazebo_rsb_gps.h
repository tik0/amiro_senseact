/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_RSB_GPS_HH
#define GAZEBO_RSB_GPS_HH

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

// #include <ros/ros.h>
// #include <ros/advertise_options.h>
// #include <sensor_msgs/LaserScan.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
// #include <gazebo/plugins/ModelPlugin.hh>
#include <gazebo_plugins/gazebo_rsb_utils.h>
#include <gazebo_plugins/update_timer.h>

// ?? Brauche ich eine eigene Queue
// #include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRsbGps : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRsbGps();

    /// \brief Destructor
    public: virtual ~GazeboRsbGps();

    /// \brief Load the plugin
    /// \param take in SDF root element
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  public: virtual void Reset();
  public: virtual void Update();

  private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

//   ros::NodeHandle* node_handle_;
//   ros::Publisher fix_publisher_;
//   ros::Publisher velocity_publisher_;

//   sensor_msgs::NavSatFix fix_;
//   geometry_msgs::Vector3Stamped velocity_;

  std::string namespace_;
  std::string link_name_;
  std::string frame_id_;
  std::string fix_topic_;
  std::string velocity_topic_;

  double reference_latitude_;
  double reference_longitude_;
  double reference_heading_;
  double reference_altitude_;

  double radius_north_;
  double radius_east_;

//   SensorModel3 position_error_model_;
//   SensorModel3 velocity_error_model_;



//   boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_position_, dynamic_reconfigure_server_velocity_;
//   boost::shared_ptr<dynamic_reconfigure::Server<GNSSConfig> > dynamic_reconfigure_server_status_;
  
  
  
  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;
  
  
    
    // RSB
//     public: rsb::Informer<rst::vision::LaserScan>::Ptr informer;
    public: rsb::Factory& factory = rsb::getFactory();
//     public: rsb::Informer<rst::vision::LaserScan>::DataPtr laserScan(new rst::vision::LaserScan);
  public: static void registerConverter();
  public: static bool converterRegistered;
  public: rsb::Informer<rst::geometry::Pose>::Ptr informer;
  public: bool defineInfomerOnce = false;
  };
}
#endif
