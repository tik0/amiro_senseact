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

ParticleFilter::ParticleFilter(size_t maxSampleCount, rst::geometry::Pose odom, Map *map, SensorModel *sensorModel, bool doKLDSampling)
{
    this->maxSampleCount = maxSampleCount;
    this->height = map->size().height;
    this->width = map->size().width;
    this->prevOdom = convertPose(odom);
    this->sensorModel = sensorModel;
    this->doKLDSampling = doKLDSampling;

    sampleSet = new sample_set_t;
    sampleSet->size = 0;
    sampleSet->samples = new sample_t[maxSampleCount];

    newSampleSet = new sample_set_t;
    newSampleSet->size = 0;
    newSampleSet->samples = new sample_t[maxSampleCount];

    this->map = map;

    initSamples();

    if (this->doKLDSampling) {
        nbBinsY = ceil((map->rows * map->meterPerCell) / binSizeY);
        nbBinsX = ceil((map->cols * map->meterPerCell) / binSizeX);
        nbBinsRot = ceil(M_PI / binSizeRot);

        bins = new bool**[nbBinsY];
        for (int i = 0; i < nbBinsY; ++i) {
            bins[i] = new bool*[nbBinsX];

            for (int j = 0; j < nbBinsX; ++j) {
                bins[i][j] = new bool[nbBinsRot];

                for (int k = 0; k < nbBinsRot; ++k) {
                    bins[i][j][k] = false;
                }
            }
        }
    }
}

ParticleFilter::~ParticleFilter()
{
    delete[] sampleSet->samples;
    delete sampleSet;

    delete[] newSampleSet->samples;
    delete newSampleSet;

    if (doKLDSampling) {
        for (int i = 0; i < nbBinsY; ++i) {
            delete[] bins[i];
        }

        delete[] bins;
    }
}

void
ParticleFilter::initSamples()
{
    float initialImportance = 1.0f / maxSampleCount;

    srand(time(NULL));

    sampleSet->size = maxSampleCount;
    for (size_t i = 0; i < sampleSet->size; ++i) {
        sampleSet->samples[i].pose.x = fmod(rand(), width * map->meterPerCell);
        sampleSet->samples[i].pose.y = fmod(rand(), height * map->meterPerCell);
        sampleSet->samples[i].pose.theta = fmod(rand(), 2 * M_PI) - M_PI;
        sampleSet->samples[i].importance = initialImportance;
    }
}

void
ParticleFilter::normalizeImportanceFactors(float logSum)
{
    for (size_t i = 0; i < sampleSet->size; ++i) {
//        float imp = samples[i].importance - logSum;
//        samples[i].importance = exp(imp);
        sampleSet->samples[i].importance /= logSum;

        assert(sampleSet->samples[i].importance >= 0.0f);
        assert(sampleSet->samples[i].importance <= 1.0f);
    }
}

sample_t
ParticleFilter::rouletteWheelSelection()
{
    float r = rand() / (float)RAND_MAX;
    float sum = 0;

    for (size_t i = 0; i < sampleSet->size; ++i) {
        sample_t s = sampleSet->samples[i];
        sum += s.importance;

        if (sum >= r) {
            return s;
        }
    }

    return sampleSet->samples[sampleSet->size - 1];
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
    //float normFactor = std::numeric_limits<float>::lowest();
    float normFactor = 0;

    // convert odometry and pre-compute deltas for pose update
    pose_t newOdom = convertPose(odom);
    preparePoseUpdate(newOdom);
    prevOdom = newOdom;

    if (doKLDSampling) {
        binsWithSupport = 0;
        for (int i = 0; i < nbBinsY; ++i) {
            for (int j = 0; j < nbBinsX; ++j) {
                for (int k = 0; k < nbBinsRot; ++k) {
                    bins[i][j][k] = false;
                }
            }
        }
    }

    for (size_t n = 0; n < maxSampleCount; ++n)
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
        newSampleSet->samples[n] = sample;
        newSampleSet->size++;

        if (doKLDSampling) {
            updateBins(sample);

            /*
             * Equation (7) from KLD-Sampling by Dieter Fox
             *
             * "z is the upper 1 - delta quantile of the standard normal N(0,1) distribution"
             * with delta = 0.99 type in octave/matlab:
             * > norminv(0.99, 0, 1)
             * ans = 2.3263
             */
            float z = 2.3263f;
            float epsilon = 0.1f; // error
            float k = binsWithSupport;
            float kldSampleCount = (k - 1) / (2 * epsilon) * pow(1 - 2 / (9.0 * (k - 1)) + sqrt(2 / (9.0 * (k - 1))) * z, 3);

            if (kldSampleCount < 10) {
                kldSampleCount = 10;
            }

            if (n >= kldSampleCount) {
                DEBUG_MSG("======================");
                DEBUG_MSG("KLD sampling ends early!");
                DEBUG_MSG("bins with support: " << binsWithSupport);
                DEBUG_MSG("kld sample count: " << kldSampleCount);
                DEBUG_MSG("======================");
                break;
            }
        }
    }

    DEBUG_MSG("sample set size: " << newSampleSet->size);

    // Adopt the new sample set
    // and reuse the old sample set (
    sample_set_t *tmp = sampleSet;
    sampleSet = newSampleSet;
    newSampleSet = tmp;
    newSampleSet->size = 0;
    // and normalize importance weights
    normalizeImportanceFactors(normFactor);
}

void
ParticleFilter::updateBins(sample_t &sample)
{
    int idxx = clamp(round(sample.pose.x / binSizeX), 0, nbBinsX - 1);
    int idxy = clamp(round(sample.pose.y / binSizeY), 0, nbBinsY - 1);
    int idxr = clamp(round(sample.pose.theta / binSizeRot), 0, nbBinsY - 1);

    if (!bins[idxy][idxx][idxr]) { // bin is empty
        bins[idxy][idxx][idxr] = true;
        binsWithSupport++;
    }
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
