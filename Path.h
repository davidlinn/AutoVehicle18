/*
 * Path.h
 *
 *  Created on: Jul 12, 2018
 *      Author: NetBurner
 */

#ifndef PATH_H_
#define PATH_H_

#include <vector>
using std::vector;

class Path {
public:
	Path();
	virtual ~Path();
	struct Coordinate { float x; float y; };
	vector<Coordinate> coordinates;
	void addToPath(Coordinate c);
};

#endif /* PATH_H_ */
