/*
 * mapGenerator.cxx
 *
 *  Created on: Dec 12, 2014
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */
#include "mapGenerator.hpp"

using namespace std;
using namespace cv;

const int MapGenerator::GRID_VALUE_MAX = 255;//65500;
const int MapGenerator::GRID_VALUE_MIN = 0;
const int MapGenerator::GRID_VALUE_UNSURE = 255/2;//65500/2;

// constructor
MapGenerator::MapGenerator(float cellSize) :
		cellSize(cellSize) {
}

// generate an obstacle map using thresholding and dilate the obstacles
void MapGenerator::generateObstacleMap(cv::Mat &map, cv::Mat &obstacleMap) {
	cv::Mat o0, o1, f0, f1;

//	// calculate blocked areas
//	map.convertTo(o0, CV_16SC1);
//	o0 += 128;
//	o0.convertTo(o1, CV_8UC1);
	cv::threshold(map, o1, BLOCKED_THESHOLD, GRID_VALUE_MAX /*65500*/, THRESH_BINARY_INV);

	// erode blocked areas
	int erosion_size = (int) (7) + 1;
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));
	dilate(o1, o1, element);

	// calculate map without obstacles
	//map.convertTo(f0, CV_16SC1);
	//f0.convertTo(f1, CV_8UC1);
	cv::threshold(map, f1,OPEN_THESHOLD, GRID_VALUE_UNSURE , THRESH_BINARY);
	f1 +=GRID_VALUE_UNSURE;
	// calculate the final map with eroded obstacles and show it
	cv::subtract(f1, o1, obstacleMap);
	//obstacleMap = o1;
}
