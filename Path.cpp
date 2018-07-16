/*
 * Path.cpp
 *
 *  Created on: Jul 12, 2018
 *      Author: NetBurner
 */

#include "Path.h"

Path::Path() {
	// TODO Auto-generated constructor stub

}

Path::~Path() {
	// TODO Auto-generated destructor stub
}

void Path::addToPath(Coordinate c) {
	coordinates.push_back(c);
}
