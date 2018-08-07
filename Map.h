/*
 * Map.h
 *
 *  Created on: Jun 6, 2018
 *      Author: NetBurner
 */

#ifndef MAP_H_
#define MAP_H_

#include <basictypes.h>
#include "Drivers/introspec.h"
#define GRID_SIZE_X 128
#define GRID_SIZE_Y 128
#define GRID_SIDE_LEN 250 //Side of a grid square in mm

class Map { //Class defines unknown features in a known 2D map
public:
	Map();
	virtual ~Map();

	//LINE SEGMENT BASED MAPPING
	void featureUpdate();
	void segmentLidar();
	struct LidarSegmentation {
		int startingDegree;
		int endingDegree;
	};
	LidarSegmentation lidarseglist[256];
	int numLidarSegs;
	void recursiveSplitAndFit(LidarSegmentation& seg);
	void fastLinearFit(LidarSegmentation& seg);
	//Map is relative to a constant starting position, coordinates in meters
	struct Circle {
		float x; //currently estimated x-position
		float y; //currently estimated y-position
		float radius;
	};
	struct Segment {
		//Represents a line segment y=mx+b that extends from x1 to x2
		float b;
		float m;
		float x1;
		float x2;
	};
	Circle circleList[256];
	int numCircles;
	Segment segmentList[256];
	int numSegments;

	//GRID OF LOG(OCCUPANCY PROBABILITY)
	//Initialize a grid in the local Cartesian frame: model car pointing upwards toward greater y
	//upon initialization. Default position is centered on x-axis, bottom 1/4 of screen on y-axis
	void initLocalGrid(int x = GRID_SIZE_X/2, int y = GRID_SIZE_Y/4);
	void updateOGrid();
	void logGridEdit(int x, int y, int prob);
	//Each grid location represents a .25m x .25m area
	uint8_t oGrid[GRID_SIZE_X][GRID_SIZE_Y];
	int carX;
	int carY;
	int x_localoffset;
	int y_localoffset;
	int head_localoffset;

	START_INTRO_OBJ(OccGridEditObj,"OccGridEdit")
	double_element time{"time"};
	uint8_element gridx{"gridx"};
	uint8_element gridy{"gridy"};
	uint8_element prob{"prob"};
	END_INTRO_OBJ;
	OccGridEditObj occGridEdit;
};

#endif /* MAP_H_ */
