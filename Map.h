/*
 * Map.h
 *
 *  Created on: Jun 6, 2018
 *      Author: NetBurner
 */

#ifndef MAP_H_
#define MAP_H_

#include <basictypes.h>
#include <vector>
using std::vector;

class Map { //Class defines unknown features in a known 2D map
public:
	Map();
	virtual ~Map();

	//Map is relative to a constant starting position, coordinates in meters
	struct Feature {
		double x; //currently estimated x-position
		double y; //currently estimated y-position
		double uncertainty; //radius of uncertainty around (x,y) position
		double estimateCount; //num times this feature has been estimated
	};
	vector<Feature> featureList;

};

#endif /* MAP_H_ */
