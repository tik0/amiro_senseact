#include <algorithm>
#include <string>
#include <assert.h>

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
  // Register new converter for RSB
  GazeboRsbGps::registerConverter();
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
  this->model = _model;
  this->world = _model->GetWorld();

  // RSB
  this->rsbScope =  rsbScopeFromGazeboFrame(this->model->GetName()) + "/" + GetSdfElementValue(this->model, _sdf, "gpsSubscope", PLUGIN_NAME);
  PrintPluginInfoString ( this->model->GetName(), PLUGIN_NAME, "RSB scope: " + this->rsbScope);
  this->informer = factory.createInformer<rst::geometry::Pose> (this->rsbScope);

  // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(this->world, _sdf);  // Overwrites setUpdateRate if requierd argument in sdf is present
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRsbGps::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRsbGps::Update()
{

  math::Pose pose = this->model->GetWorldPose();

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
