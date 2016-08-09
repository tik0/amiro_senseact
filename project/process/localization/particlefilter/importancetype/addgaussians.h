#ifndef ADDGAUSSIANS_H
#define ADDGAUSSIANS_H

#include "importancetype.h"

/**
 * @brief The AddGaussians class adds gaussians like the player/ROS AMCL implementation does.
 */
class AddGaussians : public ImportanceType
{
private:
    /**
     * @brief importance aggregation helper variable for importance of a scan
     */
    float importance = 1.0f;

    /**
     * @brief sigma for the gaussian:
     * https://www.hokuyo-aut.jp/02sensor/07scanner/urg_04lx.html
     * "Accuracy 60 to 1,000mm : ±10mm, 1,000 to 4,095mm : 1% of measurement"
     *
     * 4m * 0.01 = 0.04m
     */
    const float sigma = 0.1f;

public:
    inline void addBeamDistance(float distance) final
    {
        importance += exp( -pow(distance / sigma, 2));
    }

    inline float aggregateScanImportance() final
    {
        float ret = importance;
        // reset importance
        importance = 1.0f;

        return ret;
    }
};

#endif // ADDGAUSSIANS_H
