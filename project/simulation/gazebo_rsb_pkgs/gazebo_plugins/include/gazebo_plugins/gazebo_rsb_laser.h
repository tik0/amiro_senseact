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

#ifndef GAZEBO_RSB_LASER_HH
#define GAZEBO_RSB_LASER_HH

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
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/gazebo_rsb_utils.h>

// ?? Brauche ich eine eigene Queue
// #include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRsbLaser : public RayPlugin
  {
    /// \brief Constructor
    public: GazeboRsbLaser();

    /// \brief Destructor
    public: ~GazeboRsbLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    // Pointer to the model
//     GazeboRosPtr gazebo_ros_;
    private: std::string world_name_;
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
//     private: ros::NodeHandle* rosnode_;
//     private: ros::Publisher pub_;
//     private: PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: void OnScan(ConstLaserScanStampedPtr &_msg);

    /// \brief prevents blocking
//     private: PubMultiQueue pmq;
    private: event::ConnectionPtr updateConnection;
    
    // RSB
//     public: rsb::Informer<rst::vision::LaserScan>::Ptr informer;
    public: rsb::Factory& factory = rsb::getFactory();
//     public: rsb::Informer<rst::vision::LaserScan>::DataPtr laserScan(new rst::vision::LaserScan);
  public: static void registerConverter();
  public: static bool converterRegistered;
  };
}
#endif
