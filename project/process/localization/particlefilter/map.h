#ifndef MAP_H
#define MAP_H

#include <opencv2/core/core.hpp>

class Map : public cv::Mat1b
{
public:
    Map() : cv::Mat1b()
    {

    }

    Map(const cv::Mat &m) : cv::Mat1b(m)
    {

    }

    float meterPerCell = 0;

    inline bool isValid(const int x, const int y)
    {
        return (x >= 0 && x < cols && y >= 0 && y < rows);
    }

    inline bool isOccupied(const int x, const int y)
    {
        return (at<uchar>(y,x) < occupied);
    }

    inline size_t poseToIndex(const float &x)
    {
        return round(x / meterPerCell);
    }

private:
    const uchar occupied = (numeric_limits<uchar>::max() - numeric_limits<uchar>::min()) / 2;
};

#endif // MAP_H
