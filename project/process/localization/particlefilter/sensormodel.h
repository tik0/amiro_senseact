#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include "particlefilter.h"
#include <types/LocatedLaserScan.pb.h>

class SensorModel
{
public:
    virtual void computeWeight(sample_t &sample, const rst::vision::LocatedLaserScan &scan) = 0;
};

#endif // SENSORMODEL_H
