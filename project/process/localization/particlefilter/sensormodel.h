#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include "sampleset.h"
#include <types/LocatedLaserScan.pb.h>

class SensorModel
{
public:
    virtual void computeWeight(sample_t &sample, const rst::vision::LocatedLaserScan &scan) = 0;
    virtual void normalizeWeights(sample_set_t *sampleSet) = 0;
};

#endif // SENSORMODEL_H
