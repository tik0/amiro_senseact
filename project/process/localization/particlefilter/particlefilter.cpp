using namespace std;

#include "particlefilter.h"
#include <cstdlib>
#include <cmath>
#include <utils.h>
#include <assert.h>

#include <MSG.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ParticleFilter::ParticleFilter(size_t sampleCount, rst::vision::LocatedLaserScan scanConfig, rst::geometry::Pose odom, Map *map, SensorModel *sensorModel)
{
    this->sampleCount = sampleCount;
    this->height = map->size().height;
    this->width = map->size().width;
    this->scanConfig = scanConfig;
    this->prevOdom = convertPose(odom);
    this->sensorModel = sensorModel;

    samples = new sample_t[sampleCount];

    this->map = map;

    initSamples();
}

ParticleFilter::~ParticleFilter()
{
    delete[] samples;
}

void
ParticleFilter::initSamples()
{
    float initialImportance = 1.0f / sampleCount;

    srand(time(NULL));

    for (size_t i = 0; i < sampleCount; ++i) {
        samples[i].pose.x = fmod(rand(), width * map->meterPerCell);
        samples[i].pose.y = fmod(rand(), height * map->meterPerCell);
        samples[i].pose.theta = fmod(rand(), 2 * M_PI) - M_PI;
        samples[i].importance = initialImportance;
    }
}

void
ParticleFilter::normalizeImportanceFactors(float logSum)
{
    for (size_t i = 0; i < sampleCount; ++i) {
//        float imp = samples[i].importance - logSum;
//        samples[i].importance = exp(imp);
        samples[i].importance /= logSum;

        assert(samples[i].importance >= 0.0f);
        assert(samples[i].importance <= 1.0f);
    }
}

sample_t
ParticleFilter::rouletteWheelSelection()
{
    float r = rand() / (float)RAND_MAX;
    float sum = 0;

    for (size_t i = 0; i < sampleCount; ++i) {
        sample_t s = samples[i];
        sum += s.importance;

        if (sum >= r) {
            return s;
        }
    }

    return samples[sampleCount - 1];
}

void
ParticleFilter::sampling(sample_t &sample)
{
    // Update pose
    updatePose(sample);

    // Sample
    // TODO: rotate covariance with sample theta
    Eigen::Matrix<double,Eigen::Dynamic,-1> offset = sampler.samples(1);
    sample.pose.x += offset(0, 0);
    sample.pose.y += offset(1, 0);
    sample.pose.theta = normalizeAngle(sample.pose.theta + offset(2, 0));

    // Make sure samples are still on the map
    sample.pose.x = clamp(sample.pose.x, 0, width * map->meterPerCell);
    sample.pose.y = clamp(sample.pose.y, 0, height * map->meterPerCell);
}

void ParticleFilter::preparePoseUpdate(pose_t &newOdom)
{
    drivingDirection = atan2(newOdom.y - prevOdom.y, newOdom.x - prevOdom.x);
    phi1 = normalizeAngle(drivingDirection - prevOdom.theta);
    odometryIncrement = sqrt( pow(newOdom.x - prevOdom.x, 2) + pow(newOdom.y - prevOdom.y, 2) );
    phi2 = normalizeAngle(newOdom.theta - drivingDirection);

    DEBUG_MSG("poseUpdate: drivingDirection: " << drivingDirection
              << " phi1: " << phi1
              << " odometryIncrement: " << odometryIncrement
              << " phi2: " << phi2);
}

void
ParticleFilter::updatePose(sample_t &sample)
{
    // first: rotation: old odometry direction -> driving direction
    sample.pose.theta = normalizeAngle(sample.pose.theta + phi1);

    // second: go straight
    sample.pose.x += odometryIncrement * cos(sample.pose.theta);
    sample.pose.y += odometryIncrement * sin(sample.pose.theta);

    // lastly: rotate: driving direction -> new odometry direction
    sample.pose.theta = normalizeAngle(sample.pose.theta + phi2);
}

void
ParticleFilter::importanceSampling(sample_t &sample, const rst::vision::LocatedLaserScan &scan)
{
    sensorModel->computeWeight(sample, scan);
}

pose_t
ParticleFilter::convertPose(const rst::geometry::Pose &odom)
{
    pose_t pose;

    rst::geometry::Rotation rotation = odom.rotation();
    Eigen::Quaterniond lidar_quat(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
    Eigen::Matrix<double,3,1> rpy;
    conversion::quaternion2euler(&lidar_quat, &rpy);

    pose.x = odom.translation().x();
    pose.y = odom.translation().y();
    pose.theta = rpy(2);

    return pose;
}

void
ParticleFilter::update(const rst::vision::LocatedLaserScan &scan, const rst::geometry::Pose &odom)
{
    sample_t *newSamples = new sample_t[sampleCount];
    //float normFactor = std::numeric_limits<float>::lowest();
    float normFactor = 0;

    // convert odometry and pre-compute deltas for pose update
    pose_t newOdom = convertPose(odom);
    preparePoseUpdate(newOdom);
    prevOdom = newOdom;

    for (size_t i = 0; i < sampleCount; ++i)
    {
        // Re-sampling
        sample_t sample = rouletteWheelSelection();
        // Sampling
        sampling(sample);
        // Importance sampling
        importanceSampling(sample, scan);

        // Update normalization factor
        //normFactor = logAdd(normFactor, sample.importance);
        normFactor += sample.importance;

        // Add sample to new sample set
        newSamples[i] = sample;
    }

    // Delete the old sample set
    delete[] samples;
    // Adopt the new sample set
    samples = newSamples;
    // and normalize importance weights
    normalizeImportanceFactors(normFactor);
}

float
ParticleFilter::logAdd(float a, float b)
{
    // swap variables so that b is greater than a
    if (a > b) {
        float tmp = a;
        a = b;
        b = tmp;
    }

    if (a == std::numeric_limits<float>::lowest()) {
        return b;
    }

    /*
     * log( exp(a) + exp(b) )
     *   = log( exp(b) * ( exp(a) / exp(b) + 1 ) )
     *   = b + log( exp(a) / exp(b) + 1 )
     *   = b + log( exp(a - b) + 1 )
     */
    return b + log( exp(a - b) + 1.0f );
}
