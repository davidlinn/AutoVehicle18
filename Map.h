/*
 * Map.h
 *
 *  Created on: Jun 6, 2018
 *      Author: NetBurner
 */

#ifndef MAP_H_
#define MAP_H_

#include <basictypes.h>

class Map { //Class defines unknown features in a known 2D map
public:
	Map();
	virtual ~Map();
	void featureUpdate();
	void segmentLidar();

	struct LidarSegmentation {
		int startingDegree;
		int endingDegree;
	};
	LidarSegmentation lidarseglist[20];

	//Map is relative to a constant starting position, coordinates in meters
	struct Circle {
		float x; //currently estimated x-position
		float y; //currently estimated y-position
		float radius;
		float uncertainty; //radius of uncertainty around (x,y) position
		float estimateCount; //num times this feature has been estimated
	};
	struct Segment {
		//Represents a line segment y=mx+b that extends from x1 to x2
		float b;
		float m;
		float x1;
		float x2;
	};

	Circle circleList[40];
	Segment segmentList[80];
};

#endif /* MAP_H_ */
