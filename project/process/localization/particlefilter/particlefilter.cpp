using namespace std;

#include "particlefilter.h"
#include <cstdlib>
#include <cmath>
#include <utils.h>
#include <iostream>
#include <assert.h>
#include <time.h>

#include <MSG.h>

ParticleFilter::ParticleFilter(size_t sampleCount, float meterPerCell, rst::vision::LocatedLaserScan scanConfig, rst::geometry::Pose odom, cv::Mat1b map)
{
    this->sampleCount = sampleCount;
    this->height = map.size().height;
    this->width = map.size().width;
    this->meterPerCell = meterPerCell;
    this->scanConfig = scanConfig;
    this->prevOdom = convertPose(odom);

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
        samples[i].pose.x = fmod(rand(), width * meterPerCell);
        samples[i].pose.y = fmod(rand(), height * meterPerCell);
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

ParticleFilter::sample_t
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
    sample.pose.x = clamp(sample.pose.x, 0, width * meterPerCell);
    sample.pose.y = clamp(sample.pose.y, 0, height * meterPerCell);
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
    vector<float> sampleScan = simulateScan(sample.pose);

    // TODO: set sigma
    float sigma = 0.2f;

    // compare scans
    float importance = 0;
    for (int i = 0; i < scan.scan_values_size(); ++i) {
        float measuredRay = scan.scan_values(i);
        float expectedRay = sampleScan[i];

        if (measuredRay > scan.scan_values_max()) {
            measuredRay = scan.scan_values_max();
        }

        if (expectedRay > scan.scan_values_max()) {
            expectedRay = scan.scan_values_max();
        }

//        /*
//         * Instead of simple gaussian, we use the logarithmic of it. This way
//         * the importance factors don't become too small.
//         */
//        importance -= log(sigma * sqrt(2 * M_PI)) - 0.5 * pow((measuredRay - expectedRay) / sigma, 2);
        //importance += pdf_gaussian(measuredRay, expectedRay, sigma);
        // simplified version of gaussian, neglecting constanst factor
        importance += exp( -0.5 * pow( (measuredRay - expectedRay)/sigma, 2.0 ) );
    }

    sample.importance = importance;
    //std::cout << "importance: " << importance << std::endl;
}

float
ParticleFilter::simulateRay(const pose_t &pose, float globalAngle)
{
    // start point
    int x0 = poseToIndex(pose.x);
    int y0 = poseToIndex(pose.y);

    // target point
    int x1 = poseToIndex( pose.x + scanConfig.scan_values_max() * cos(globalAngle) );
    int y1 = poseToIndex( pose.y + scanConfig.scan_values_max() * sin(globalAngle) );

    // determine the fast growing direction
    bool steep;
    if(abs(y1 - y0) > abs(x1 - x0)) {
        steep = true;
    } else {
        steep = false;
    }

    // swap x and y, so that x is the fast growing direction
    if (steep) {
        int tmp = x0;
        x0 = y0;
        y0 = tmp;

        tmp = x1;
        x1 = y1;
        y1 = tmp;
    }

    // differences
    int deltax = abs(x1 - x0);
    int deltay = abs(y1 - y0);

    int error = 0;
    int deltaerr = deltay;

    int x = x0;
    int y = y0;

    // determine direction
    int xstep;
    if (x0 < x1) {
        xstep = 1;
    } else {
        xstep = -1;
    }

    int ystep;
    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    // first check if start positions are valid (in map and not occupied)
    if (steep) {
        if (!isInMap(y,x) || mapIsOccupied(y,x)) {
            return 0;
        }
    } else {
        if (!isInMap(x,y) || mapIsOccupied(x,y)) {
            return 0;
        }
    }

    // until target point reached
    while (x != (x1 + xstep * 1)) {
        x += xstep;
        error += deltaerr;

        if (2 * error >= deltax) {
            y += ystep;
            error -= deltax;
        }

        if (steep) {
            if (!isInMap(y,x) || mapIsOccupied(y,x)) {
                return sqrt( (x-x0)*(x-x0) + (y-y0)*(y-y0)) * meterPerCell;
            }
        } else {
            if (!isInMap(x,y) || mapIsOccupied(x,y)) {
                return sqrt( (x-x0)*(x-x0) + (y-y0)*(y-y0)) * meterPerCell;
            }
        }
    }

    return scanConfig.scan_values_max();
}

vector<float>
ParticleFilter::simulateScan(const pose_t &pose)
{
    vector<float> scan(scanConfig.scan_values_size());

    size_t i = 0;
    for (float angle = scanConfig.scan_angle_start();
        angle < scanConfig.scan_angle_end();
        angle += scanConfig.scan_angle_increment()) {

        float globalAngle = normalizeAngle(pose.theta + angle); // no normalization necessary cause it is passed to cos/sin

        // cast ray
        float rayLength = simulateRay(pose, globalAngle);

        scan[i++] = rayLength;
    }

    return scan;
}


ParticleFilter::pose_t
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

    for (size_t i = 0; i < sampleCount; ++i) {
        //std::cout << "sample " << i << ": " << newSamples[i].importance << std::endl;
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
