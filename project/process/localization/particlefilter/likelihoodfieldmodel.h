#ifndef LIKELIHOODFIELDMODEL_H
#define LIKELIHOODFIELDMODEL_H

#include "sensormodel.h"
#include "map.h"

class LikelihoodFieldModel : public SensorModel
{
public:
    enum IMPORTANCE_TYPE {
        MULT_GAUSS,         // multiply gaussians of each ray, does not work since importance converges to zero
        MULT_GAUSS_LOG,     // same as above but in log space
        ADD_GAUSS,          // add gaussians of each (as in ROS' AMCL)
        INVERSE_DISTANCE    // 1 / (sum of distances)
    };

    LikelihoodFieldModel(Map *map, const rst::vision::LocatedLaserScan &scanConfig, int beamskip, IMPORTANCE_TYPE importance_type);
    ~LikelihoodFieldModel();
    void computeWeight(sample_t &sample, const rst::vision::LocatedLaserScan &scan);
    void normalizeWeights(sample_set_t *sampleSet);

private:
    Map *map;
    float *distanceToObstacle;
    int beamskip = 1;

    float *scan_cos;
    float *scan_sin;

    IMPORTANCE_TYPE importance_type;
    float initialImportance;

    void computeLikelihoodFieldNaive();
    void computeLikelihoodFieldRadius();
    void computeLikelihoodFieldTree();

    void showDistanceMap();

    inline float logAdd(float a, float b);
};

#endif // LIKELIHOODFIELDMODEL_H
