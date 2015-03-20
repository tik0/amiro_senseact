#ifndef UTILS_H
#define UTILS_H

// Opencv
#include <opencv2/opencv.hpp>
#include <cvplot/cvplot.h>

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

#include <math.h>

namespace utils
{

// 2D Gauss function
inline double draw2DGauss(cv::Mat x /*2x1 matrix*/, cv::Mat mu /*2x1 matrix*/, cv::Mat sigma /*2x2 matrix*/) {

  cv::Mat sigmaInv(2, 2, CV_64FC1);
  const double sigmaDet = determinant(sigma);
  sigmaInv.at<double>(0,0) = sigma.at<double>(1,1) / sigmaDet;
  sigmaInv.at<double>(1,0) = -sigma.at<double>(1,0) / sigmaDet;
  sigmaInv.at<double>(0,1) = -sigma.at<double>(0,1) / sigmaDet;
  sigmaInv.at<double>(1,1) = sigma.at<double>(0,0) / sigmaDet;

  cv::Mat tmp = x - mu;
  cv::Mat tmpT; cv::transpose(tmp, tmpT);
  cv::Mat exponent =  tmpT * sigmaInv * tmp;  // Should result in a 1x1 matrix
  return 1 / (2.0 * M_PI * sigmaDet) * exp(-0.5 * exponent.at<double>(0));
}

// 1D Gauss function
inline double draw1DGauss(double x, double mu, double sigma) {
  double tmp = pow(x - mu, 2);
  double exponent =  tmp / sigma;
  return 1 / (sqrt(2.0 * M_PI) * sigma) * exp(-0.5 * exponent);
}

inline void sendImage (const rsb::Informer<std::string>::Ptr informer, cv::Mat img)
{
  std::vector<uchar> buf;
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(100);
  cv::imencode(".jpg", img, buf, compression_params);

  rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
  informer->publish(frameJpg);
}

template<typename T>
inline void sendPlot (const std::string figure_name, const rsb::Informer<std::string>::Ptr informer, const T* p, int count, int step, int R, int G, int B)
{
  if(count == 0 || p == NULL || informer == NULL)
    return;

  IplImage * plot = CvPlot::plot(figure_name, p, count, step, R, G, B);
  sendImage(informer, cv::Mat(plot));
  CvPlot::clear(figure_name);
  cvReleaseImage(&plot);
}

enum esitmatorId{
  isCropId = 1,
  isFloorId = 2,
  isWeedId = 3,
};

template<typename T>
inline T odds(T P) {
  return P / (1-P);
}

template<typename T>
inline T lOdds(T P) {
  return log(P) - log(1-P);
}

template<typename T>
inline T pFromLOdds(T lR) {
  return exp(lR) / (1.0 + exp(lR));
}

const int invertModelSize = 128;
const double invertModelResolution = 0.01; /*m*/
const double holWidthSize = 0.1; /*m*/

template<typename T>
inline T sumLOdds(T P1, T P2) {
  return lOdds(P1) + lOdds(P2);
}

void addLocalToGlobalMap(cv::Mat& localMap, cv::Mat& globalMap, cv::Point2f robotPosition) {
  double cellSize = invertModelResolution;
  // Get the position of the local map in the global map (Left upper corner).
  // The robot is in the middle of the local map.
  int localMapX = (int) (robotPosition.x / cellSize - localMap.cols / 2.0 + globalMap.cols / 2.0);
  int localMapY = (int) (robotPosition.y / cellSize - localMap.rows / 2.0 + globalMap.rows / 2.0);

  // check if the localMap is positioned in the global map
  if (localMapX > -localMap.cols && localMapX < globalMap.cols && localMapY > -localMap.rows
      && localMapY < globalMap.rows) {
    // determine the size of the overlap
    cv::Size s(min(min( localMap.cols, localMap.cols + localMapX), globalMap.cols - localMapX ),
        min(min( localMap.rows, localMap.rows + localMapY) , globalMap.rows - localMapY ));

    // Add the local to the global map. Note: OpenCV automatically will clip the values in the global map between -128 and 127.
    cv::Mat globalMapTmp = globalMap(cv::Rect(cv::Point(max(localMapX, 0), max(localMapY, 0)), s));
    cv::Mat localMapTmp = localMap(cv::Rect(cv::Point(max(0, -localMapX), max(0, -localMapY)), s));
    for (int idxy = 0; idxy < globalMapTmp.cols - 1; idxy++) {
      for (int idxx = 0; idxx < globalMapTmp.rows - 1; idxx++) {
//        for (int channel = 0; channel < 3; channel++) {
//          std::cout << "d :" << globalMapTmp.depth() << std::endl << std::flush;
//          std::cout << "ch :" << globalMapTmp.channels() << std::endl << std::flush;
//          globalMapTmp.at<uchar>(idxy, idxx)  = 0;//;cv::Scalar(255, 255, 255);
//          globalMapTmp.at<cv::Vec3b>(idxx, idxy)  = cv::Vec3b(255, 0, 0);
//          globalMapTmp.at<uchar>(512, 513)  = 255;//;cv::Scalar(255, 255, 255);
//          globalMapTmp.at<uchar>(512, 514)  = 255;//;cv::Scalar(255, 255, 255);
//          globalMapTmp.at<uchar>(512, 515)  = 255;//;cv::Scalar(255, 255, 255);
          cv::Vec3b g = globalMapTmp.at<cv::Vec3b>(idxx, idxy);
          cv::Vec3b l = localMapTmp.at<cv::Vec3b>(idxx, idxy);

//          globalMapTmp.at<cv::Vec3b>(idxx, idxy) = cv::Vec3b( uchar(pFromLOdds(sumLOdds(double(g[0]) / 255.0, double(l[0]) / 255.0) * 255.0)),
//                                                              uchar(pFromLOdds(sumLOdds(double(g[1]) / 255.0, double(l[1]) / 255.0) * 255.0)),
//                                                              uchar(pFromLOdds(sumLOdds(double(g[2]) / 255.0, double(l[2]) / 255.0) * 255.0)));
          globalMapTmp.at<cv::Vec3b>(idxx, idxy) = cv::Vec3b( uchar(pFromLOdds(sumLOdds(0.5,0.5)) * 255.0),
                                                              uchar(pFromLOdds(sumLOdds(0.5,0.5)) * 255.0),
                                                              uchar(pFromLOdds(sumLOdds(0.5,0.5)) * 255.0));
//              pFromLOdds(sumLOdds(localMapTmp.at<double>(idxy, idxx) / 255.0, globalMapTmp.at<double>(idxy, idxx) / 255.0)) * 250.0;
//        }
      }
    }
    std::cout << "P"<< pFromLOdds(sumLOdds(0.5,0.5)) << std::endl;
              std::cout << "R"<< sumLOdds(0.5,0.5) << std::endl;
//    globalMapTmp.at<uchar>(512, 512)  = cv::Scalar(255, 255, 255);
//    globalMap(cv::Rect(cv::Point(max(localMapX, 0), max(localMapY, 0)), s)) += localMap(
//        cv::Rect(cv::Point(max(0, -localMapX), max(0, -localMapY)), s));

//    std::cout << s << std::endl;
  }
}

// ML estimator for the lidar
void getLidarEstimations(utils::esitmatorId id, const float *scan_values ,std::vector<unsigned char> &isPresent, int size) {
  // ML estimator for the different {Weed, Crop, Floor} features /////
  double muWeed  = 0.0627; /*m*/
  double muCrop  = 0.07277; /*m*/
  double muFloor = 0.116286; /*m*/
  double sigmaLidar = 0.02; /*m²:  s.t. noise of the lidar*/

  for(int idx = 0; idx < size; ++idx) {
    double nWeed = draw1DGauss(scan_values[idx], muWeed, sigmaLidar);
    double nCrop = draw1DGauss(scan_values[idx], muCrop, sigmaLidar);
    double nFloor = draw1DGauss(scan_values[idx], muFloor, sigmaLidar);
    if (nWeed > nCrop /* && nWeed > nFloor */){
      isPresent[idx] = id == utils::isWeedId ? true : false;
    } else if (nCrop > nFloor) {
      isPresent[idx] = id == utils::isCropId ? true : false;
    } else /*nWeed < nFloor*/ {
      isPresent[idx] = id == utils::isFloorId ? true : false;
    }
  }

}

}
#endif



