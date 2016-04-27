#include "likelihoodfieldmodel.h"

#include <MSG.h>

#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

LikelihoodFieldModel::LikelihoodFieldModel(Map *map)
{
    this->map = map;
    distanceToObstacle = cv::Mat1f(map->size());

    INFO_MSG("starting computing distances to obstacles...");
    time_t start = time(NULL);
    computeLikelihoodFieldTree();
    INFO_MSG("done. Took " << (time(NULL) - start) << " seconds");

    showDistanceMap();
}

void
LikelihoodFieldModel::showDistanceMap()
{
    float maxValue = 0;
    for (int row = 0; row < map->rows; row++) {
        for (int col = 0; col < map->cols; col++) {
            if (distanceToObstacle.at<float>(row, col) > maxValue) {
                maxValue = distanceToObstacle.at<float>(row, col);
            }
        }
    }

    DEBUG_MSG("maxValue: " << maxValue);

    cv::Mat1b dbgImg(distanceToObstacle.size());

    for (int row = 0; row < map->rows; row++) {
        for (int col = 0; col < map->cols; col++) {
            dbgImg.at<uchar>(row, col) = 255.0f * distanceToObstacle.at<float>(row, col) / maxValue;
        }
    }

    cv::imshow("distance map", dbgImg);
    cv::waitKey(0);
}

void
LikelihoodFieldModel::computeWeight(sample_t &sample, const rst::vision::LocatedLaserScan &scan)
{
    float importance = 0;

    float sigma = 0.2f;

    size_t i = 0;
    for (float angle = scan.scan_angle_start();
        angle < scan.scan_angle_end();
        angle += scan.scan_angle_increment()) {

        float globalAngle = sample.pose.theta + angle;

        int idxx = map->poseToIndex( sample.pose.x + scan.scan_values(i) * cos(globalAngle) );
        int idxy = map->poseToIndex( sample.pose.y + scan.scan_values(i) * sin(globalAngle) );

        if (map->isValid(idxx, idxy)) {
            float d = distanceToObstacle.at<float>(idxy, idxx);
            importance += exp( - pow(d / sigma, 2) );
        }

        i++;
    }

    sample.importance = importance;
    //DEBUG_MSG("importance: " << importance);
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
            cv::Point p(col, row);

            if (map->isOccupied(col, row)) {
                distanceToObstacle.at<float>(p) = 0.0f;
            } else {
                // find closest cell
                float minDist = std::numeric_limits<float>::max();
                for (const cv::Point &q : occupiedCells) {
                    float currentDist = cv::norm(q - p);
                    minDist = std::min(minDist, currentDist);
                }
                distanceToObstacle.at<float>(p) = minDist * map->meterPerCell;
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
                distanceToObstacle.at<float>(p) = 0.0f;
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
            distanceToObstacle.at<float>(p) = minDist * map->meterPerCell;
        }
    }
}


void
LikelihoodFieldModel::computeLikelihoodFieldTree()
{
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;

    typedef bg::model::point<int, 2, bg::cs::cartesian> point;

    bgi::rtree<point, bgi::quadratic<16>> rtree;

    // find all occupied cells and insert them in the index
    for (int row = 0; row < map->rows; ++row) {
        for (int col = 0; col < map->cols; ++col) {
            if (map->isOccupied(col, row)) {
                point p = point(col, row);
                rtree.insert(p);
            }
        }
    }

    DEBUG_MSG("Inserted all occupied cells in the index (" << rtree.size() << ")");

    for (int row = 0; row < map->rows; ++row) {
        DEBUG_MSG("progress: " << (row) / (float)(map->rows));
        for (int col = 0; col < map->cols; ++col) {
            //DEBUG_MSG("row: " << row << " col: " <<

            if (map->isOccupied(col, row)) {
                //DEBUG_MSG("is occupied -> distance = 0");
                distanceToObstacle.at<float>(row, col) = 0.0f;
            } else {
                std::vector<point> result_n;
                //DEBUG_MSG("Searching nearest neighbor...");
                rtree.query(bgi::nearest(point(col, row), 1), std::back_inserter(result_n));
                //DEBUG_MSG("found nearest neighbor");

                float distance = sqrt( pow(col - result_n.at(0).get<0>(), 2) + pow(row - result_n.at(0).get<1>(), 2) );
                //DEBUG_MSG("distance: " << distance);
                distanceToObstacle.at<float>(row, col) = distance * map->meterPerCell;
            }
        }
    }

    DEBUG_MSG("Computed all distances");
}
