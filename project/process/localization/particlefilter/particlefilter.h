#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

using namespace std;

#include <cstddef>
#include <Eigen/Core>
#include <types/LocatedLaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>
#include <opencv2/core/core.hpp>
#include "eigenmvn/eigenmvn.h"

#include <iostream>

class ParticleFilter
{
public:

    /**
     * @brief ParticleFilter
     * @param sampleCount Number of particles used in this filter.
     * @param meterPerCell Resolution of the map in m/pixel.
     * @param scanConfig Used to determine number of rays, minimum and maximum values of scans etc...
     * @param odom The initial odometry data used to form odometry deltas.
     * @param map A openCV mat to represent to map.
     */
    ParticleFilter(size_t sampleCount, float meterPerCell, rst::vision::LocatedLaserScan scanConfig, rst::geometry::Pose odom, cv::Mat1b map);
    ~ParticleFilter();

    void setScanConfig(const rst::vision::LocatedLaserScan &scan) {
        scanConfig = scan;
    }

    rst::vision::LocatedLaserScan &getScanConfig() {
        return scanConfig;
    }

    /**
     * @brief The pose_t struct holds a pose in the map in meters and radians.
     */
    struct pose_t {
        /**
         * @brief x position along the x-axis in meter.
         */
        float x;
        /**
         * @brief y position along the y-axis in meter.
         */
        float y;
        /**
         * @brief theta rotation in radians.
         */
        float theta;
    };

    /**
     * @brief The sample_t struct represents a particle with its pose and importance weight.
     */
    struct sample_t {
        pose_t pose;
        float importance;
    };

    /**
     * @brief update updates the believe of the particle filter.
     * @param scan currently measured laser scan used for importance sampling.
     * @param odom currently measured odometry to compute odometry delta in the sampling step.
     */
    void update(const rst::vision::LocatedLaserScan &scan, const rst::geometry::Pose &odom);

    sample_t *getSamples() {
        return samples;
    }

private:
    size_t sampleCount;
    size_t width, height;
    float meterPerCell;

    sample_t *samples;

    pose_t prevOdom = {0,0,0};
    pose_t odometryDelta;

    cv::Mat1b map;
    const uchar mapOccupied = (numeric_limits<uchar>::max() - numeric_limits<uchar>::min()) / 2;

    Eigen::Vector3d mean = Eigen::Vector3d(0,0,0);
    Eigen::Matrix3d covar = Eigen::DiagonalMatrix<double,3,3>(0.01, 0.01, 0.01);
    Eigen::EigenMultivariateNormal<double> sampler = Eigen::EigenMultivariateNormal<double>(mean, covar);

    rst::vision::LocatedLaserScan scanConfig;

    void initSamples();
    void normalizeImportanceFactors(float logSum);
    sample_t rouletteWheelSelection();

    // angle of vector from old odometry to new odometry
    float drivingDirection;
    // angle from old odometry theta to driving direction
    float phi1;
    // angle from driving direction to new new odometry theta
    float phi2;
    // distance between old and new odometry
    float odometryIncrement;
    // pre-computes above variables
    void preparePoseUpdate(pose_t &newOdom);
    void updatePose(sample_t &sample);
    void sampling(sample_t &sample);

    float simulateRay(const pose_t &pose, float globalAngle);
    vector<float> simulateScan(const pose_t &pose);
    void importanceSampling(sample_t &sample, const rst::vision::LocatedLaserScan &scan);

    pose_t convertPose(const rst::geometry::Pose &odom);

    float logAdd(float a, float b);

    // normalizes an angle to [-PI;+PI)
    inline float normalizeAngle(float theta)
    {
        if (theta < -M_PI || theta >= M_PI) {
            theta = fmod(theta + M_PI, 2*M_PI);
            if (theta < 0)
                theta += 2*M_PI;
            theta = theta - M_PI;
        }

        return theta;
    }

    inline float pdf_gaussian(float x, float mean, float sigma)
    {
        return ( 1 / ( sigma * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x - mean)/sigma, 2.0 ) );
    }

    inline float clamp(float x, float min, float max)
    {
        return std::max(min, std::min(x, max));
    }

    inline bool isInMap(size_t x, size_t y)
    {
        return (x >= 0 && x < width && y >= 0 && y < height);
    }

    inline bool mapIsOccupied(int x, int y)
    {
        return (map.at<uchar>(y,x) < mapOccupied);
    }

    inline size_t poseToIndex(const float &x)
    {
        return round(x / meterPerCell);
    }

};

#endif // PARTICLEFILTER_H
