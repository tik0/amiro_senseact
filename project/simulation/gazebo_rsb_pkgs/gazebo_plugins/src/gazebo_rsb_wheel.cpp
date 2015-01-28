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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Proto types
#include <rst0.11/sandbox/rst/kinematics/Twist.pb.h>

// #include <gazebo_plugins/gazebo_rsb_laser.h>

// Basewidth in cm TODO Replace with include/constants.h
#define baseWidth 0.06f
#define wheelRadius 0.03f
namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // registerConverter
      ModelPush::registerConverter();
      // Get a listener TODO: Get the scope from SDF file
      this->listener = this->factory.createListener("/test");
      this->queueRemoteVelocities.reset(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::kinematics::Twist> >(1));

      this->listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::kinematics::Twist>(this->queueRemoteVelocities)));
      
      // Store the pointer to the model
      this->model = _parent;

      // Get the joint from the AMiRo.sdf
      this->leftWheel = this->model->GetJoint("left_wheel_hinge");
      this->rightWheel = this->model->GetJoint("right_wheel_hinge");
      // Set the allowed forces TODO: Set correct forces here, and kinematics in the AMiRo.sdf
      this->leftWheel->SetMaxForce(0, 0);
      this->leftWheel->SetMaxForce(1, 0);
      this->leftWheel->SetMaxForce(2, 1); // nm
      this->rightWheel->SetMaxForce(0, 0);
      this->rightWheel->SetMaxForce(1, 0);
      this->rightWheel->SetMaxForce(2, 1); // nm
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Process the RSB message
      if (!this->queueRemoteVelocities->empty() ) {
        rst::kinematics::Twist twist = *this->queueRemoteVelocities->tryPop().get();
        rst::kinematics::AngularVelocities angVel = twist.angular();
        rst::kinematics::LinearVelocities  linVel = twist.linear();
        // Set angular velocity in rad/s of the left wheel
        this->leftWheel->SetVelocity(2, (linVel.x() - baseWidth / 2.0f * angVel.c()) / wheelRadius );
        // Set angular velocity in rad/s of the right wheel
        this->rightWheel->SetVelocity(2, (linVel.x() + baseWidth / 2.0f * angVel.c() ) / wheelRadius );

        // Print out target velocities
        std::cout << "Vx = " << linVel.x() << " m/s , Wz = " << angVel.c() << " rad/s" << std::endl;
        // Print out current real velocities
        this->vecLin = this->model->GetRelativeLinearVel();
        this->vecAng = this->model->GetRelativeAngularVel();
        std::cout << "Vx = " << this->vecLin[0] << " m/s , Wz = " << this->vecAng[2] << " rad/s" << std::endl;
      }
    }

    // Register RSB converter once
    public: static void registerConverter() {
    if (ModelPush::converterRegistered == false) {
      ModelPush::converterRegistered = true;
      boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::kinematics::Twist> >
          converter(new rsb::converter::ProtocolBufferConverter<rst::kinematics::Twist>());
      rsb::converter::converterRepository<std::string>()->registerConverter(converter);
      }
    }

    // RSB listener
    public: static bool converterRegistered;
    public: rsb::Factory& factory = rsb::getFactory();
    public: boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::kinematics::Twist> > > queueRemoteVelocities;
    rsb::ListenerPtr listener;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the two wheels of the differential kinematic
    private: physics::JointPtr leftWheel;
    private: physics::JointPtr rightWheel;

    // Robot real velocities
    math::Vector3 vecLin, vecAng;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  bool ModelPush::converterRegistered = false;
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}