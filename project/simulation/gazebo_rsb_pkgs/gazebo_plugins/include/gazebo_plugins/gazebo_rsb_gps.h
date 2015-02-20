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
  physics::ModelPtr model;
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;
  
  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;

    // RSB
    public: rsb::Factory& factory = rsb::getFactory();
    public: static void registerConverter();
    public: rsb::Informer<rst::geometry::Pose>::Ptr informerGT;
    private: std::string rsbScope;
  };
}
#endif
