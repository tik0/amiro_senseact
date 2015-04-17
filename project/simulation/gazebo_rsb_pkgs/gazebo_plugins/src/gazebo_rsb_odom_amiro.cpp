#include <algorithm>
#include <string>
#include <assert.h>
#include <random>

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Proto types
#include <rst/geometry/Pose.pb.h>

#include <gazebo_plugins/gazebo_rsb_odom_amiro.h>
#include <gazebo_plugins/matrix.h>

#define PLUGIN_NAME "gazebo_rsb_odom_amiro"

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRsbOdomAmiro::GazeboRsbOdomAmiro()
{
  // Register new converter for RSB
  GazeboRsbOdomAmiro::registerConverter();
}

void GazeboRsbOdomAmiro::registerConverter() {
  try {
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
        converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converter);
  } catch (...) {

  }
}

void GazeboRsbOdomAmiro::Reset()
{
  updateTimer.Reset();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRsbOdomAmiro::~GazeboRsbOdomAmiro()
{

}


////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRsbOdomAmiro::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  this->model = _model;
  this->world = _model->GetWorld();

  // RSB
  this->rsbScope =  rsbScopeFromGazeboFrame(this->model->GetName()) + rsbScopeFromGazeboFrame(GetSdfElementValue(this->model->GetName(), _sdf, "odomSubscope", PLUGIN_NAME));
  PrintPluginInfoString ( this->model->GetName(), PLUGIN_NAME, "RSB scope: " + this->rsbScope);
  this->informerOdom = factory.createInformer<rst::geometry::Pose> (this->rsbScope);

  // Get the initial pose of the robot for the noisy calculations
  this->poseNoisy = this->model->GetWorldPose();
  this->poseOld = this->model->GetWorldPose();
  this->baseWidth = std::stof(GetSdfElementValue(this->model->GetName(), _sdf, "baseWidth", PLUGIN_NAME));
  this->kl = std::stof(GetSdfElementValue(this->model->GetName(), _sdf, "kl", PLUGIN_NAME));
  this->kr = std::stof(GetSdfElementValue(this->model->GetName(), _sdf, "kr", PLUGIN_NAME));

  // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(this->world, _sdf);  // Overwrites setUpdateRate if requierd argument in sdf is present
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRsbOdomAmiro::Update, this));
}

void GazeboRsbOdomAmiro::poseJacobian( math::Vector3 eulerAnglePose, float translation, float rotation, Matrix &poseJacobian) {
  float tt = eulerAnglePose.z + rotation / 2.0f;

  // First Row
  poseJacobian(0,0) =  0.0; // 1.0
  poseJacobian(0,1) =  0.0;
  poseJacobian(0,2) =  -translation * sin(tt);
  // Second Row
  poseJacobian(1,0) =  0.0;
  poseJacobian(1,1) =  0.0; // 1.0
  poseJacobian(1,2) =  translation * cos(tt);
  // Third Row
  poseJacobian(2,0) =  0.0;
  poseJacobian(2,1) =  0.0;
  poseJacobian(2,2) =  0.0; // 1.0
}

void GazeboRsbOdomAmiro::motionJacobian( math::Vector3 eulerAnglePose, float translation, float rotation, Matrix &motionJacobian) {
  float tt = eulerAnglePose.z + rotation / 2.0f;
  float ss = translation / this->baseWidth;
  float c  = 0.5 * cos(tt);
  float s  = 0.5 * sin(tt);

  // First Row
  motionJacobian(0,0) =  c - ss * s;
  motionJacobian(0,1) =  c + ss * s;
  // Second Row
  motionJacobian(1,0) =  s + ss * c;
  motionJacobian(1,1) =  s - ss * c;
  // Third Row
  motionJacobian(2,0) =  1.0 / this->baseWidth;
  motionJacobian(2,1) =  -1.0 / this->baseWidth;
}

void GazeboRsbOdomAmiro::motionCovariance( math::Vector3 eulerAnglePose, float translation, float rotation, Matrix &motionCovariance) {

  float sl = this->baseWidth * rotation / 2.0f + translation;
  float sr = - this->baseWidth * rotation / 2.0f + translation;

  // First Row
  motionCovariance(0,0) =  this->kr * abs(sr);
  motionCovariance(0,1) =  0.0;
  // Second Row
  motionCovariance(1,0) =  0.0;
  motionCovariance(1,1) =  this->kl * abs(sl);
}

void GazeboRsbOdomAmiro::rndMultiVarGauss(Vector &rnd, Matrix &Sigma, Vector &Mean) {
  // Get the eigen vector and values
  Matrix eVector;
  Vector eValues;
  sym_eig(Sigma, eVector, eValues);
  Vector rndTmp(eValues.size());

  // Calculate the random values
  for (int idx=0; idx < eValues.size(); ++idx) {
    std::normal_distribution<float> distribution(0.0,1.0);
    if (eValues[idx] > 0.0)
      rnd[idx] = distribution(this->generator) * sqrt(eValues[idx]);
    else
      rnd[idx] = 0.0f;
  }

  rnd = eVector * rnd + Mean;
}

math::Pose GazeboRsbOdomAmiro::calcOdomFailure( math::Pose pose, math::Pose deltaPose, float updatePeriod) {
  math::Pose poseNoisy;  // The new noisy position

  // Get current values
  math::Vector3 eulerAngle = pose.rot.GetAsEuler();
  math::Vector3 eulerAngleDelta = deltaPose.rot.GetAsEuler();
  float translation = sqrt(pow(deltaPose.pos.x,2) + pow(deltaPose.pos.y,2));
  float rotation = eulerAngleDelta.z;

  // Calculate the error (Only steering error is used)
//  Matrix Fp(3,3);
//  this->poseJacobian(eulerAngle, translation, rotation, Fp);
  // Calculate the translation error
//  std::cout << "Fp " << Fp << std::endl;
  Matrix Fs(3,2);
  this->motionJacobian(eulerAngle, translation, rotation, Fs);
//  std::cout << "Fs " << Fs << std::endl;
  // Get the motion covariance
  Matrix Cs(2,2);
  this->motionCovariance(eulerAngle, translation, rotation, Cs);
//  std::cout << "Cs " << Cs << std::endl;
  // Get the new uncertainty
  Matrix pCp(3,3);
//  pCp =  Fp * CpOld * pseudoinverse(Fp) + Fs * Cs * pseudoinverse(Fs);
  pCp =  Fs * Cs * pseudoinverse(Fs);

  Vector rnd(3), Mean(3);
  Mean[0] = deltaPose.pos.x;
  Mean[1] = deltaPose.pos.y;
  Mean[2] = eulerAngleDelta.z;
  this->rndMultiVarGauss(rnd, pCp, Mean);
  // Store the new x, y, f_z
  poseNoisy.pos.x = pose.pos.x + rnd[0];
  poseNoisy.pos.y = pose.pos.y + rnd[1];
  poseNoisy.pos.z = 0.0f;  // Is not be estimated in the setup
  eulerAngle.x = 0.0f;  // Is not be estimated in the setup
  eulerAngle.y = 0.0f;  // Is not be estimated in the setup
  eulerAngle.z += rnd[2];
  poseNoisy.rot = math::Quaternion::EulerToQuaternion(eulerAngle);

  return poseNoisy;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRsbOdomAmiro::Update()
{
  // Get the ground truth data
  math::Pose pose = this->model->GetWorldPose();

  // Calculate the odometry failure (We assume the AMiRo robot for all errors)
  // Calculate the difference in position
  math::Vector3 positionDelta = pose.pos - this->poseOld.pos;
  math::Vector3 eulerDelta = pose.rot.GetAsEuler() - this->poseOld.rot.GetAsEuler();
  math::Quaternion quatDelta;
  quatDelta.SetFromEuler(eulerDelta);
  math::Pose poseDelta(positionDelta, quatDelta);
  // Calculate the new noisy position
  this->poseNoisy = calcOdomFailure(this->poseNoisy, poseDelta, updateTimer.getUpdatePeriod().Float());
  // Store the old ground truth pose
  this->poseOld = pose;


  // Store the new pose the informer type
  rsb::Informer<rst::geometry::Pose>::DataPtr odomData(new rst::geometry::Pose);

  rst::geometry::Translation *odomtranslation = odomData->mutable_translation();
  rst::geometry::Rotation *odomrotation = odomData->mutable_rotation();
  odomtranslation->set_x(this->poseNoisy.pos.x);
  odomtranslation->set_y(this->poseNoisy.pos.y);
  odomtranslation->set_z(this->poseNoisy.pos.z);
  odomrotation->set_qw(this->poseNoisy.rot.w);
  odomrotation->set_qx(this->poseNoisy.rot.x);
  odomrotation->set_qy(this->poseNoisy.rot.y);
  odomrotation->set_qz(this->poseNoisy.rot.z);

  // Publish
  this->informerOdom->publish(odomData);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRsbOdomAmiro)


}
