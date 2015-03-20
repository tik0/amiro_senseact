#include "gazebo_plugins/gazebo_rsb_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRsbCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRsbCamera::GazeboRsbCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRsbCamera::~GazeboRsbCamera()
{

}

void GazeboRsbCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the RSB is initialized
  if (false)
  {
    std::cout << "A RSB informer for Gazebo has not been initialized, unable to load plugin. "
      << "(Load the Gazebo system plugin 'libgazebo_rsb_api_plugin.so' in the gazebo_ros package)" << std::endl;
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

//  // Get the world name.
//  std::string world_name = _parent->GetWorldName();
//
//  // Get the world_
//  this->world = physics::get_world(world_name);
//
//  // save pointers
//  this->sdf = _sdf;
//
//  std::stringstream ss;
//  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Camera");
//
//  this->image_topic_name_ = "image_raw";
//  if (this->sdf->HasElement("imageTopicName"))
//    this->image_topic_name_ = this->sdf->Get<std::string>("imageTopicName");
//
//  this->camera_info_topic_name_ = "camera_info";
//  if (this->sdf->HasElement("cameraInfoTopicName"))
//    this->camera_info_topic_name_ =
//      this->sdf->Get<std::string>("cameraInfoTopicName");
//
//  if (!this->sdf->HasElement("cameraName"))
//    ROS_DEBUG("Camera plugin missing <cameraName>, default to empty");
//  else
//    this->camera_name_ = this->sdf->Get<std::string>("cameraName");
//
//  // overwrite camera suffix
//  // example usage in gazebo_ros_multicamera
//  this->camera_name_ += _camera_name_suffix;
//
//  if (!this->sdf->HasElement("frameName"))
//    ROS_DEBUG("Camera plugin missing <frameName>, defaults to /world");
//  else
//    this->frame_name_ = this->sdf->Get<std::string>("frameName");
//
//  if (!this->sdf->HasElement("updateRate"))
//  {
//    ROS_DEBUG("Camera plugin missing <updateRate>, defaults to unlimited (0).");
//    this->update_rate_ = 0;
//  }
//  else
//    this->update_rate_ = this->sdf->Get<double>("updateRate");
//
//  if (!this->sdf->HasElement("CxPrime"))
//  {
//    ROS_DEBUG("Camera plugin missing <CxPrime>, defaults to 0");
//    this->cx_prime_ = 0;
//  }
//  else
//    this->cx_prime_ = this->sdf->Get<double>("CxPrime");
//
//  if (!this->sdf->HasElement("Cx"))
//  {
//    ROS_DEBUG("Camera plugin missing <Cx>, defaults to 0");
//    this->cx_= 0;
//  }
//  else
//    this->cx_ = this->sdf->Get<double>("Cx");
//
//  if (!this->sdf->HasElement("Cy"))
//  {
//    ROS_DEBUG("Camera plugin missing <Cy>, defaults to 0");
//    this->cy_= 0;
//  }
//  else
//    this->cy_ = this->sdf->Get<double>("Cy");
//
//  if (!this->sdf->HasElement("focalLength"))
//  {
//    ROS_DEBUG("Camera plugin missing <focalLength>, defaults to 0");
//    this->focal_length_= 0;
//  }
//  else
//    this->focal_length_ = this->sdf->Get<double>("focalLength");
//
//  if (!this->sdf->HasElement("hackBaseline"))
//  {
//    ROS_DEBUG("Camera plugin missing <hackBaseline>, defaults to 0");
//    this->hack_baseline_= 0;
//  }
//  else
//    this->hack_baseline_ = this->sdf->Get<double>("hackBaseline");
//
//  if (!this->sdf->HasElement("distortionK1"))
//  {
//    ROS_DEBUG("Camera plugin missing <distortionK1>, defaults to 0");
//    this->distortion_k1_= 0;
//  }
//  else
//    this->distortion_k1_ = this->sdf->Get<double>("distortionK1");
//
//  if (!this->sdf->HasElement("distortionK2"))
//  {
//    ROS_DEBUG("Camera plugin missing <distortionK2>, defaults to 0");
//    this->distortion_k2_= 0;
//  }
//  else
//    this->distortion_k2_ = this->sdf->Get<double>("distortionK2");
//
//  if (!this->sdf->HasElement("distortionK3"))
//  {
//    ROS_DEBUG("Camera plugin missing <distortionK3>, defaults to 0");
//    this->distortion_k3_= 0;
//  }
//  else
//    this->distortion_k3_ = this->sdf->Get<double>("distortionK3");
//
//  if (!this->sdf->HasElement("distortionT1"))
//  {
//    ROS_DEBUG("Camera plugin missing <distortionT1>, defaults to 0");
//    this->distortion_t1_= 0;
//  }
//  else
//    this->distortion_t1_ = this->sdf->Get<double>("distortionT1");
//
//  if (!this->sdf->HasElement("distortionT2"))
//  {
//    ROS_DEBUG("Camera plugin missing <distortionT2>, defaults to 0");
//    this->distortion_t2_= 0;
//  }
//  else
//    this->distortion_t2_ = this->sdf->Get<double>("distortionT2");
//
//  if ((this->distortion_k1_ != 0.0) || (this->distortion_k2_ != 0.0) ||
//      (this->distortion_k3_ != 0.0) || (this->distortion_t1_ != 0.0) ||
//      (this->distortion_t2_ != 0.0))
//  {
//    ROS_WARN("gazebo_ros_camera_ simulation does not support non-zero"
//             " distortion parameters right now, your simulation maybe wrong.");
//  }
//
//  // initialize shared_ptr members
//  if (!this->image_connect_count_) this->image_connect_count_ = boost::shared_ptr<int>(new int(0));
//  if (!this->image_connect_count_lock_) this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
//  if (!this->was_active_) this->was_active_ = boost::shared_ptr<bool>(new bool(false));

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosCameraUtils::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRsbCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
//  this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
//
//  if (!this->parentSensor->IsActive())
//  {
//    if ((*this->image_connect_count_) > 0)
//      // do this first so there's chance for sensor to run once after activated
//      this->parentSensor->SetActive(true);
//  }
//  else
//  {
//    if ((*this->image_connect_count_) > 0)
//    {
//      common::Time cur_time = this->world_->GetSimTime();
//      if (cur_time - this->last_update_time_ >= this->update_period_)
//      {
//        this->PutCameraData(_image);
//        this->PublishCameraInfo();
//        this->last_update_time_ = cur_time;
//      }
//    }
//  }
}
}
