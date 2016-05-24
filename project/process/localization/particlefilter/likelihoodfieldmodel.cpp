#include "likelihoodfieldmodel.h"

#include <MSG.h>

#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

LikelihoodFieldModel::LikelihoodFieldModel(Map *map, const rst::vision::LocatedLaserScan &scanConfig, int beamskip, IMPORTANCE_TYPE importance_type)
{
    this->map = map;
    this->beamskip = beamskip;
    this->importance_type = importance_type;

    switch (importance_type) {
        case MULT_GAUSS:
           initialImportance = 1.0f;
           break;
        case MULT_GAUSS_LOG:
            initialImportance = 0.0f; // log(1) = 0
            break;
        case ADD_GAUSS:
            initialImportance = 0.0f;
            break;
        case INVERSE_DISTANCE:
            initialImportance = 0.0f;
            break;
        default:
            ERROR_MSG("Unknown importance type.");
            exit(0);
    }

    distanceToObstacle = new float[map->rows * map->cols];

    INFO_MSG("starting computing distances to obstacles...");
    time_t start = time(NULL);
    computeLikelihoodFieldTree();
    INFO_MSG("done. Took " << (time(NULL) - start) << " seconds");

#ifndef __arm__
    showDistanceMap();
#endif

    // pre-compute cos/sin for laser beams
    float angle = scanConfig.scan_angle_start();
    float increment = scanConfig.scan_angle_increment();
    if (scanConfig.scan_angle_start() > scanConfig.scan_angle_end()) {
        increment = -increment;
    }

    scan_cos = new float[scanConfig.scan_values_size()];
    scan_sin = new float[scanConfig.scan_values_size()];
    for (int i = 0; i < scanConfig.scan_values_size(); ++i) {
        angle += increment;
        scan_cos[i] = cos(angle);
        scan_sin[i] = sin(angle);
    }
}

LikelihoodFieldModel::~LikelihoodFieldModel()
{
    delete[] distanceToObstacle;
    delete[] scan_cos;
    delete[] scan_sin;
}

void
LikelihoodFieldModel::showDistanceMap()
{
    float maxValue = 0;
    for (int row = 0; row < map->rows; row++) {
        for (int col = 0; col < map->cols; col++) {
            if (distanceToObstacle[row * map->cols + col] > maxValue) {
                maxValue = distanceToObstacle[row * map->cols + col];
            }
        }
    }

    DEBUG_MSG("maxValue: " << maxValue);

    cv::Mat1b dbgImg(map->size());

    for (int row = 0; row < map->rows; row++) {
        for (int col = 0; col < map->cols; col++) {
            dbgImg.at<uchar>(row, col) = 255.0f * distanceToObstacle[row * map->cols + col] / maxValue;
        }
    }

    cv::imshow("distance map", dbgImg);
    cv::waitKey(0);
}

void
LikelihoodFieldModel::computeWeight(sample_t &sample, const rst::vision::LocatedLaserScan &scan)
{
    float importance = initialImportance;

    /*
     * Sigma for the gaussian:
     * https://www.hokuyo-aut.jp/02sensor/07scanner/urg_04lx.html
     * "Accuracy 60 to 1,000mm : ±10mm, 1,000 to 4,095mm : 1% of measurement"
     *
     * 4m * 0.01 = 0.04m
     */
    const float sigma = 0.1f;

    float pose_cos = cos(sample.pose.theta);
    float pose_sin = sin(sample.pose.theta);

    uint validScanValues = 0;
    for (int i = 0; i < scan.scan_values_size(); i += beamskip) {

        // discard invalid rays
        if (scan.scan_values(i) > scan.scan_values_max() || scan.scan_values(i) < scan.scan_values_min()) {
            continue;
        }

        /*
         * This is the same as:
         *
         * float angle = scan.scan_angle_start() + i * scan.scan_angle_increment(); (or - depending wether increment is positive or negative)
         * float globalAngle = sample.pose.theta + angle;
         * int idxx = map->poseToIndex( sample.pose.x + scan.scan_values(i) * cos(globalAngle) );
         * int idxy = map->poseToIndex( sample.pose.y + scan.scan_values(i) * sin(globalAngle) );
         *
         * since:
         * cos(a + b) = cos(a) * cos(b) - sin(a) * sin(b)
         * sin(a + b) = sin(a) * cos(b) + cos(a) * sin(b)
         *
         * where a = sample.pose.theta
         * and b = angle
         */
        int idxx = map->poseToIndex( sample.pose.x + scan.scan_values(i) * (pose_cos * scan_cos[i] - pose_sin * scan_sin[i]) );
        int idxy = map->poseToIndex( sample.pose.y + scan.scan_values(i) * (pose_sin * scan_cos[i] + pose_cos * scan_sin[i]) );

        float d;
        if (map->isValid(idxx, idxy)) {
            d = distanceToObstacle[idxy * map->cols + idxx];
        } else {
            // find a point on map by clamping
            int onMapX = std::max(0, std::min(idxx, map->cols - 1));
            int onMapY = std::max(0, std::min(idxx, map->rows - 1));
            // distance to next obstacle is distance from on-map point + on-map points distance to next obstacle
            d = distanceToObstacle[onMapY * map->cols + onMapY];
            d += map->meterPerCell * sqrt( pow(onMapX - idxx, 2) + pow(onMapY - idxy, 2) );
        }

        // map the distance to importance
        switch (importance_type) {
            case MULT_GAUSS:
                // importance *= exp( - pow(d / sigma, 2) );
                // just save the exponent, since exp(a) * exp(b) = exp(a + b), exp needs only to be applied once
                importance += - pow(d / sigma, 2);
                break;

            case MULT_GAUSS_LOG:
                importance -= pow(d / sigma, 2);
                break;

            case ADD_GAUSS:
                importance += exp( - pow(d / sigma, 2) );
                break;

            case INVERSE_DISTANCE:
                importance += d;
                break;
        }

        validScanValues++;
    }

    switch (importance_type) {
        case MULT_GAUSS:
            sample.importance = exp(importance);
            break;

        case MULT_GAUSS_LOG:
            sample.importance = importance;
            break;

        case ADD_GAUSS:
            sample.importance = importance;
            break;

        case INVERSE_DISTANCE:
            if (validScanValues > 0) {
                // use the average distance instead of sum, so the importance does not become too small
                float averageDistance = (importance / validScanValues);
                if (averageDistance > 0) {
                    sample.importance = 1.0f / averageDistance;
                } else {
                    // average distance is zero, so this sample must be perfect
                    sample.importance = std::numeric_limits<float>::max() / 2; //std::numeric_limits<float>::max(); <- bad cause division etc will cause NaNs
                }
            } else {
                sample.importance = 0.0f;
            }
            break;
    }
}

void
LikelihoodFieldModel::computeLikelihoodFieldNaive()
{
    // First find all occupied cells
    std::vector<cv::Point> occupiedCells;

    for (int row = 0; row < map->rows; ++row) {
        for (int col = 0; col < map->cols; ++col) {
            if (map->isOccupied(col, row)) {
                occupiedCells.push_back(cv::Point(col, row));
            }
        }
    }

    DEBUG_MSG("found all occupied cells (" << occupiedCells.size() << ")");

    // For every cells compute distance to closest cell
    for (int row = 0; row < map->rows; ++row) {
        DEBUG_MSG("progress: " << (row / (float)map->rows));
        for (int col = 0; col < map->cols; ++col) {

            if (map->isOccupied(col, row)) {
                distanceToObstacle[row * map->cols + col] = 0.0f;
            } else {
                // find closest cell
                float minDist = std::numeric_limits<float>::max();
                for (const cv::Point &q : occupiedCells) {
                    cv::Point p(col, row);
                    float currentDist = cv::norm(q - p);
                    minDist = std::min(minDist, currentDist);
                }
                distanceToObstacle[row * map->cols + col] = minDist * map->meterPerCell;
            }
        }
    }
}

void
LikelihoodFieldModel::computeLikelihoodFieldRadius()
{
    // For every cells compute distance to closest cell
    for (int row = 0; row < map->rows; ++row) {
        DEBUG_MSG("progress: " << (row / (float)map->rows));
        for (int col = 0; col < map->cols; ++col) {

            // Point from which we compute the distance
            cv::Point p(col, row);

            // Corner case: point itself is occupied
            if (map->isOccupied(col, row)) {
                distanceToObstacle[row * map->cols + col] = 0.0f;
                continue;
            }

            // The cursor is sweeping in a square around the point p.
            cv::Point cursor(p.x + 1, p.y + 1);
            // Distance to closest occupied cell
            float minDist = std::numeric_limits<float>::max();

            // Increase layer until we find a occupied cell
            for (int layer = 1; layer <= minDist; layer++) {

                // east border
                cursor.x = p.x + layer;
                cursor.y = p.y - layer;

                for (; cursor.y <= p.y + layer; ++cursor.y) {
                    if (map->isValid(cursor.x, cursor.y) && map->isOccupied(cursor.x, cursor.y)) {
                        float dist = cv::norm(cursor - p);

                        if (dist < minDist) {
                            minDist = dist;
                        }
                    }
                }

                // west border
                cursor.x = p.x - layer;
                cursor.y = p.y - layer;

                for (; cursor.y <= p.y + layer; ++cursor.y) {
                    if (map->isValid(cursor.x, cursor.y) && map->isOccupied(cursor.x, cursor.y)) {
                        float dist = cv::norm(cursor - p);

                        if (dist < minDist) {
                            minDist = dist;
                        }
                    }
                }

                // south border
                cursor.y = p.y - layer;
                cursor.x = p.x - layer;

                for (; cursor.x <= p.x + layer; ++cursor.x) {
                    if (map->isValid(cursor.x, cursor.y) && map->isOccupied(cursor.x, cursor.y)) {
                        float dist = cv::norm(cursor - p);

                        if (dist < minDist) {
                            minDist = dist;
                        }
                    }
                }

                // north border
                cursor.y = p.y + layer;
                cursor.x = p.x - layer;

                for (; cursor.x <= p.x + layer; ++cursor.x) {
                    if (map->isValid(cursor.x, cursor.y) && map->isOccupied(cursor.x, cursor.y)) {
                        float dist = cv::norm(cursor - p);

                        if (dist < minDist) {
                            minDist = dist;
                        }
                    }
                }
            }

            // Save minimal distance
            distanceToObstacle[row * map->cols + col] = minDist * map->meterPerCell;
        }
    }
}


void
LikelihoodFieldModel::computeLikelihoodFieldTree()
{
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;

    typedef bg::model::point<int, 2, bg::cs::cartesian> point;

    std::vector<point> occupiedCells;

    // find all occupied cells and insert them in the index
    for (int row = 0; row < map->rows; ++row) {
        for (int col = 0; col < map->cols; ++col) {
            if (map->isOccupied(col, row)) {
                point p = point(col, row);
                occupiedCells.push_back(p);
            }
        }
    }

    bgi::rtree<point, bgi::quadratic<16>> rtree(occupiedCells);

    DEBUG_MSG("Inserted all occupied cells in the index (" << rtree.size() << ")");

    for (int row = 0; row < map->rows; ++row) {
        DEBUG_MSG("progress: " << (row) / (float)(map->rows));
        for (int col = 0; col < map->cols; ++col) {
            //DEBUG_MSG("row: " << row << " col: " <<

            if (map->isOccupied(col, row)) {
                //DEBUG_MSG("is occupied -> distance = 0");
                distanceToObstacle[row * map->cols + col] = 0.0f;
            } else {
                std::vector<point> result_n;
                //DEBUG_MSG("Searching nearest neighbor...");
                rtree.query(bgi::nearest(point(col, row), 1), std::back_inserter(result_n));
                //DEBUG_MSG("found nearest neighbor");

                float distance = sqrt( pow(col - result_n.at(0).get<0>(), 2) + pow(row - result_n.at(0).get<1>(), 2) );
                //DEBUG_MSG("distance: " << distance);
                distanceToObstacle[row * map->cols + col] = distance * map->meterPerCell;
            }
        }
    }

    DEBUG_MSG("Computed all distances");
}

void
LikelihoodFieldModel::normalizeWeights(sample_set_t *sampleSet)
{
    float normFactor;

    switch (importance_type) {
        case MULT_GAUSS:
            normFactor = 1.0f;
            break;
        case MULT_GAUSS_LOG:
            normFactor = std::numeric_limits<float>::lowest();
            break;
        case ADD_GAUSS:
            normFactor = 0.0f;
            break;
        case INVERSE_DISTANCE:
            normFactor = 0.0f;
            break;
        default:
            ERROR_MSG("Unknown importance_type: " << importance_type);
            exit(0);
            break;
    }

    // calculate norm factor
    for (size_t i = 0; i < sampleSet->size; ++i) {
        DEBUG_MSG(i << ": " << sampleSet->samples[i].importance);
        switch (importance_type) {
            case MULT_GAUSS_LOG:
                normFactor = logAdd(normFactor, sampleSet->samples[i].importance);
                break;
            default:
                normFactor += sampleSet->samples[i].importance;
                break;
        }
    }

    float sum = 0;
    for (size_t i = 0; i < sampleSet->size; ++i) {
        switch (importance_type) {
            case MULT_GAUSS_LOG: {
                // division is substraction in log-space
                float imp = sampleSet->samples[i].importance - normFactor;
                sampleSet->samples[i].importance = exp(imp);
                break;
            }
            default:
                sampleSet->samples[i].importance /= normFactor;
                break;
        }

        DEBUG_MSG(i << ": " << sampleSet->samples[i].importance);
        assert(sampleSet->samples[i].importance >= 0.0f);
        assert(sampleSet->samples[i].importance <= 1.0f);

        sum += sampleSet->samples[i].importance;
    }
    DEBUG_MSG("sum: " << sum);
}

inline float
LikelihoodFieldModel::logAdd(float a, float b)
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
