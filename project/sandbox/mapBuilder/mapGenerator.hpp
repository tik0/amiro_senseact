/*
 * mapGenerator.hpp
 *
 *  Created on: Dec 12, 2014
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */

#ifndef MAP_GENERATOR_H_
#define MAP_GENERATOR_H_

#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/shared_ptr.hpp>

// Class that contains methods to update a obstacle grid map.
class MapGenerator {
public:
	// Constructor, cellSize in m
	MapGenerator(float cellSize);
	void generateObstacleMap(cv::Mat &map, cv::Mat &obstacleMap);

private:
		// size of a cell in m
	const float cellSize;

	// map constants
	static const int GRID_VALUE_MAX;
	static const int GRID_VALUE_MIN;
	static const int GRID_VALUE_UNSURE;


	static const int BLOCKED_THESHOLD = 255/2 -10;//65500/2 -10;
	static const int OPEN_THESHOLD = 255/2+1;//65500/2 +1;

};

#endif  // MAP_GENERATOR_H_
