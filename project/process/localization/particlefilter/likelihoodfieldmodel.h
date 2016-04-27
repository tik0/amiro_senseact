#ifndef LIKELIHOODFIELDMODEL_H
#define LIKELIHOODFIELDMODEL_H

#include "sensormodel.h"
#include "map.h"

class LikelihoodFieldModel : public SensorModel
{
public:
    LikelihoodFieldModel(Map *map);
    void computeWeight(sample_t &sample, const rst::vision::LocatedLaserScan &scan);

private:
    Map *map;
    cv::Mat1f distanceToObstacle;

    void computeLikelihoodFieldNaive();
    void computeLikelihoodFieldRadius();
    void computeLikelihoodFieldTree();

    void showDistanceMap();
};

#endif // LIKELIHOODFIELDMODEL_H
