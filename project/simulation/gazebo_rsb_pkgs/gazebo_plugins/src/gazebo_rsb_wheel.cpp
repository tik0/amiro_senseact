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
#include <gazebo_plugins/gazebo_rsb_utils.h>

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

#define PLUGIN_NAME "gazebo_rsb_wheel"
namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

      // Store the pointer to the model
      this->model = _parent;

      // registerConverter
      ModelPush::registerConverter();
      // Get a listener
      this->rsbScope =  rsbScopeFromGazeboFrame(this->model->GetName()) + "/" + GetSdfElementValue(this->model->GetName(), _sdf, "differentialKinematicSubscope", PLUGIN_NAME);
      PrintPluginInfoString ( this->model->GetName(), PLUGIN_NAME, "RSB scope: " + this->rsbScope);
      this->listener = this->factory.createListener(this->rsbScope);
      this->queueRemoteVelocities.reset(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::kinematics::Twist> >(1));
      this->listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::kinematics::Twist>(this->queueRemoteVelocities)));
      
      // Get model paramter
      this->baseWidth = std::stof(GetSdfElementValue(this->model->GetName(), _sdf, "baseWidth", PLUGIN_NAME));
      this->wheelRadius = std::stof(GetSdfElementValue(this->model->GetName(), _sdf, "wheelRadius", PLUGIN_NAME));
      this->leftWheelMaxForceN = std::stof(GetSdfElementValue(this->model->GetName(), _sdf, "leftWheelMaxForceN", PLUGIN_NAME));
      this->rightWheelMaxForceN = std::stof(GetSdfElementValue(this->model->GetName(), _sdf, "rightWheelMaxForceN", PLUGIN_NAME));

      
      // Get all joints from the model
      this->joints = this->model->GetJoints();
      
//       std::cout << "Num Joints " << joints.size() << std::endl;
      
      // Get the left and right wheel join
      for (int idx = 0; idx < this->joints.size(); ++idx) {
        if(getSubScope(this->joints[idx]->GetName()).compare("left_wheel_hinge") == 0)
          this->leftWheel = this->joints[idx];
      }
      if (this->leftWheel == NULL)
        gzthrow("Unable to get left_wheel_hinge\n");
      for (int idx = 0; idx < this->joints.size(); ++idx) {
        if(getSubScope(this->joints[idx]->GetName()).compare("right_wheel_hinge") == 0)
          this->rightWheel = this->joints[idx];
      }
      if (this->rightWheel == NULL)
        gzthrow("Unable to get right_wheel_hinge\n");

      // Set the allowed forces
      this->leftWheel->SetMaxForce(0, 0);
      this->leftWheel->SetMaxForce(1, 0);
      this->leftWheel->SetMaxForce(2, this->leftWheelMaxForceN); // nm
      this->rightWheel->SetMaxForce(0, 0);
      this->rightWheel->SetMaxForce(1, 0);
      this->rightWheel->SetMaxForce(2, this->rightWheelMaxForceN); // nm

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
        this->leftWheel->SetVelocity(2, (linVel.x() - baseWidth / 2.0f * angVel.c()) / this->wheelRadius );
        // Set angular velocity in rad/s of the right wheel
        this->rightWheel->SetVelocity(2, (linVel.x() + baseWidth / 2.0f * angVel.c() ) / this->wheelRadius );

        // Print out target velocities
        std::cout << "Vx = " << linVel.x() << " m/s , Wz = " << angVel.c() << " rad/s" << std::endl;
        // Print out current real velocities
        this->vecLin = this->model->GetRelativeLinearVel();
        this->vecAng = this->model->GetRelativeAngularVel();
        std::cout << "Vx = " << this->vecLin[0] << " m/s , Wz = " << this->vecAng[2] << " rad/s" << std::endl;
        // Get the odometry
//         math::Pose pose = this->model->GetInitialRelativePose();
        math::Pose pose = this->model->GetRelativePose();
//         math::Pose pose = this->model->GetWorldPose();
        std::cout << pose << std::flush << std::endl;
        pose = this->model->GetWorldPose();
        std::cout << pose << std::flush << std::endl;
      }

    }

    // Register RSB converter once
    public: static void registerConverter() {
      try {
        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::kinematics::Twist> >
            converter(new rsb::converter::ProtocolBufferConverter<rst::kinematics::Twist>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter);
      } catch (...) {

      }
    }

    // RSB listener
    public: rsb::Factory& factory = rsb::getFactory();
    public: boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::kinematics::Twist> > > queueRemoteVelocities;
    public: rsb::ListenerPtr listener;
    public: std::string rsbScope;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Model parameters in meter
    private: float baseWidth;
    private: float wheelRadius;
    private: float leftWheelMaxForceN;
    private: float rightWheelMaxForceN;
    
    // Vector of all joints of the model
    private: physics::Joint_V joints;

    // Pointer to the two wheels of the differential kinematic
    private: physics::JointPtr leftWheel;
    private: physics::JointPtr rightWheel;

    // Robot real velocities
    math::Vector3 vecLin, vecAng;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
