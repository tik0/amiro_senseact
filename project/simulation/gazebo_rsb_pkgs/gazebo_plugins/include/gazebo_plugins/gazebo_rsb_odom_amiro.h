#ifndef GAZEBO_RSB_GPS_HH
#define GAZEBO_RSB_GPS_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo_plugins/gazebo_rsb_utils.h>
#include <gazebo_plugins/update_timer.h>
#include <gazebo_plugins/matrix.h>

namespace gazebo
{
  class GazeboRsbOdomAmiro : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRsbOdomAmiro();

    /// \brief Destructor
    public: virtual ~GazeboRsbOdomAmiro();

    /// \brief Load the plugin
    /// \param take in SDF root element
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  public: virtual void Reset();
  public: virtual void Update();

  private:
  /// \brief The parent World
  physics::ModelPtr model;
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;
  
  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;
  // Noisy pose calculations for odometry
  private: math::Pose calcOdomFailure( math::Pose pose, math::Pose deltaPose, float updatePeriod);
  // pose Jacobian, see Siegwart/Nourbakhsh p.189
  private: void poseJacobian( math::Vector3 eulerAnglePose, float translation, float rotation, Matrix &poseJacobian);
  private: void motionJacobian( math::Vector3 eulerAnglePose, float translation, float rotation, Matrix &motionJacobian);
  private: void motionCovariance( math::Vector3 eulerAnglePose, float translation, float rotation, Matrix &motionCovariance);
  private: void rndMultiVarGauss(Vector &rnd, Matrix &Sigma, Vector &Mean);
  float kl = 0.1;
  float kr = 0.1;
  math::Pose poseNoisy;
  math::Pose poseOld;
  float baseWidth;
  std::default_random_engine generator;

    // RSB
    public: rsb::Factory& factory = rsb::getFactory();
    public: static void registerConverter();
    public: rsb::Informer<rst::geometry::Pose>::Ptr informerOdom;
    private: std::string rsbScope;
  };
}
#endif
