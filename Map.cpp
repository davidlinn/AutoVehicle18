/*
 * Map.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: NetBurner
 */

#include "Map.h"
#include "FastMath.h"
#include "Drivers/SpinningLidar.h"
#include "Nav.h"
#include <math.h>
#include <stdlib.h>
#include "Profiler.h"
#include "Drivers/IMUSetupAndSample.h"
#include "Utility.h"
extern Nav nav;
#define OGRIDLOG 1
extern double getGlobalTime();

Map::Map() {
	// TODO Auto-generated constructor stub
}

Map::~Map() {
	// TODO Auto-generated destructor stub
}

void Map::featureUpdate() {
	//Profiler::start();
	//Profiler::tic(0);
	numLidarSegs = 0;
	segmentLidar(); //Builds initial lidarseglist: populates startingDegree and endingDegree based on lidar reading continuity
	int num = numLidarSegs; //Save local copy of numLidarSegs before recursiveSplitAndFit increments it
	for (int i = 0; i < num; ++i)
		recursiveSplitAndFit(lidarseglist[i]); //Further break down each segment in lidarseglist
	//Now that we know which points belong to which lines, let's fit the lines in the global cartesian coordinate system
	numCircles=0; numSegments=0;
	for (int i = 0; i < numLidarSegs; ++i)
		fastLinearFit(lidarseglist[i]);
	//Profiler::toc(0);
	//Profiler::stop();
}

void Map::segmentLidar() {
	//All done in LIDAR coordinate frame
	int findEnd = 0;
	if (SpinningLidar::dist[0] != 0) //If 0 degrees is in the middle of a segmentation, look for end
		findEnd = 1;
	int degree = 1;
	LidarSegmentation lidarseg; //temporary object that represents a starting degree and ending degree
	int segmentNum = 0;
	while (degree < 360) { //Loop through 360 degrees, find the start or end of a segment
		switch (findEnd) {
		case 0: if (SpinningLidar::dist[degree] == 0) break;
				else {lidarseg.startingDegree = degree; findEnd = true; break;}
		case 1: if (SpinningLidar::dist[degree] != 0) break;
				else { lidarseg.endingDegree = degree-1; findEnd = false;
					lidarseglist[segmentNum++]=lidarseg; break; }
		}
		++degree;
	}
	if (findEnd) lidarseglist[0].startingDegree = lidarseg.startingDegree; //Wrap around degree 0
	numLidarSegs = segmentNum; //store num lidar segs to avoid looping through array and setting elems to null
}

void Map::recursiveSplitAndFit(LidarSegmentation& seg) {
	//LIDAR coordinate system (0 is directly in front of car, positive in CW dir)
	if (seg.startingDegree==seg.endingDegree) return; //Base case for recursion
	float THRESHOLD = 130; //13 cm
	//Consider the line formed by the endpoints
	float mag1 = SpinningLidar::dist[seg.startingDegree];
	float x1 = mag1*fastcos(seg.startingDegree);
	float y1 = mag1*fastsin(seg.startingDegree);
	float mag2 = SpinningLidar::dist[seg.endingDegree];
	float x2 = mag2*fastcos(seg.endingDegree);
	float y2 = mag2*fastsin(seg.endingDegree);
	float m = (y2-y1)/(x2-x1);
	//y-y1=m(x-x1)
	//y=mx-mx1+y1
	//0=mx-y-(mx1+y1)  0=Ax+By+C  A=m,B=-1,C=-(mx1+y1)
	//Find point with max dist from line
	float maxDistFromLine;
	int degreeToSplit = -1;
	for (int i = seg.startingDegree+1; i < seg.endingDegree; ++i) { //for each point
		float mag = SpinningLidar::dist[i];
		float x0 = mag*fastcos(i);
		float y0 = mag*fastsin(i);
		float distPtToLine = abs(m*x0-y0-(m*x1+y1))/sqrt(pow(m,2)+1);
		if (distPtToLine > THRESHOLD && distPtToLine > maxDistFromLine) {
			maxDistFromLine = distPtToLine;
			degreeToSplit = i;
		}
	}
	if (degreeToSplit == -1) return;
	else {
		//Split segs at degreeToSplit and recurse
		LidarSegmentation tempseg = seg; //Local var initialized with copy constructor
		seg.endingDegree = degreeToSplit;
		tempseg.startingDegree = degreeToSplit;
		lidarseglist[numLidarSegs] = tempseg; //Copy memory into array
		++numLidarSegs;
		recursiveSplitAndFit(seg);
		recursiveSplitAndFit(lidarseglist[numLidarSegs]);
	}
}

void Map::fastLinearFit(LidarSegmentation& seg) {
	//Puts Segments and Circles in GLOBAL CARTESIAN coordinate system
	int numPoints = seg.endingDegree-seg.startingDegree+1;
	//If only one point, model it as a circle
	if (numPoints == 1) {
		Circle c;
		float mag = SpinningLidar::dist[seg.startingDegree];
		c.x = nav.getX()+mag*fastcos(seg.startingDegree);
		c.y = nav.getY()+mag*fastsin(seg.startingDegree);
		c.radius = 4;
		circleList[numCircles] = c;
		++numCircles;
		return;
	}
	//Otherwise, find the model parameters m and b that correspond with the line y=mx+b
	float mag1 = SpinningLidar::dist[seg.startingDegree];
	float degree = getHeading()-seg.startingDegree; //heading plus lidar offset
	float x1 = nav.getX()+mag1*fastcos(degree);  //fasttrig does zero to 360 mapping
	float y1 = nav.getY()+mag1*fastsin(degree);
	float mag2 = SpinningLidar::dist[seg.endingDegree];
	degree = getHeading()-seg.endingDegree;
	float x2 = nav.getX()+mag2*fastcos(degree);
	float y2 = nav.getY()+mag2*fastsin(degree);
	//Define line segment: y-y1=m(x-x1)  y=m*x(-m*x1+y1)  y=mx+b
	Segment s;
	s.x1 = x1;
	s.x2 = x2;
	s.m = (y2-y1)/(x2-x1);
	s.b = -s.m*x1+y1;
	//If there's
	segmentList[numSegments] = s;
	++numSegments;
}

//OCCUPANCY GRID
void Map::logGridEdit(int x, int y, int prob) {
	occGridEdit.time=getGlobalTime();
	occGridEdit.gridx=x;
	occGridEdit.gridy=y;
	occGridEdit.prob=prob;
	occGridEdit.Log();
}

void Map::initLocalGrid() {
	carX = x_offset; //left side of screen
	carY = y_offset; //vertically centered
	x_start = nav.getX();
	y_start = nav.getY();
	if (OGRIDLOG) {
		occGridInit.xStart = x_start;
		occGridInit.yStart = y_start;
		occGridInit.gridSideLen = GRID_SIDE_LEN;
		occGridInit.xOffset = x_offset;
		occGridInit.yOffset = y_offset;
		occGridInit.Log();
	}
	//Initialize grid to .2 probability
	for (int i = 0; i < GRID_SIZE_X; ++i)
		for (int j = 0; j < GRID_SIZE_Y; ++j)
			oGrid[i][j] = 2;
	/*//For each lidar hit, set probability = .1 if before hit location, .9 at hit location, and .7 around hit location
	for (int i = 0; i < 360; ++i) {
		int mag = SpinningLidar::dist[i];
		int dir = getHeading()-i;
		float cosdir = fastcos(dir); //fasttrig does zero to 360 degree mapping for us
		float sindir = fastsin(dir);
		int hitX = carX+(mag*cosdir/GRID_SIDE_LEN);
		int hitY = carY+(mag*sindir/GRID_SIDE_LEN);
		float dX = .5*cosdir;
		float dY = .5*sindir;
		float gridX = carX + dX; //move .5 grid units in direction each iteration
		float gridY = carY + dY;
		//Set all spaces away before lidar hit equal to .1 probability
		while (abs(gridX-hitX)>1 && abs(gridY-hitY)>1) {
			oGrid[(int)gridX][(int)gridY] = 1; //.1 probability
			if (OGRIDLOG) logGridEdit(gridX,gridY,1);
			gridX += dX;
			gridY += dY;
		}
		//Set spaces around lidar hit to .7
		for (int a = hitX-1; a <= hitX+1; ++a) {
			for (int b = hitY-1; b <= hitY+1; ++b) {
				oGrid[a][b] = 7;
				if (OGRIDLOG) logGridEdit(a,b,7);
			}
		}
		//Set lidar hit to .9
		oGrid[hitX][hitY] = 9;
		if (OGRIDLOG) logGridEdit(hitX,hitY,9);
		//While occ prob over .1 and still on board, initialize cells behind lidar hit to linearly decreasing prob from .7
		int prob = 7;
		gridX = hitX+cosdir; //Move one grid unit past hit
		gridY = hitY+sindir;
		while (prob > 1 && gridX >= 0 && gridX < GRID_SIZE_X && gridY >= 0 && gridY) {
			oGrid[(int)gridX][(int)gridY] = prob;
			if (OGRIDLOG) logGridEdit(gridX,gridY,prob);
			--prob;
			gridX += dX;
			gridY += dY;
		}
	}*/
}

void Map::updateOGrid() {
	carX = ((nav.getX()-x_start)/GRID_SIDE_LEN)+x_offset;
	carY = ((nav.getY()-y_start)/GRID_SIDE_LEN)+y_offset;
	if (carX>GRID_SIZE_X || carX<0 || carY>GRID_SIZE_Y || carY<0) initLocalGrid(); //if car out of bounds, reinitialize grid
	for (int i = 0; i < 360; i+=2) { //for every other lidar reading
		int mag = SpinningLidar::dist[i];
		if (mag < 1 || SpinningLidar::sampleQuality[i] < 1) continue; //if lidar reading bad, skip to next lidar reading
		int dir = getHeading()-i;
		float cosdir = fastcos(dir);
		float sindir = fastsin(dir);
		int hitX = carX+(mag*cosdir/GRID_SIDE_LEN);
		int hitY = carY+(mag*sindir/GRID_SIDE_LEN);
		if (hitX>GRID_SIZE_X || hitX<0 || hitY>GRID_SIZE_Y || hitY<0) continue; //if lidar hit out of bounds, skip lidar reading
		float dX = .5*cosdir;
		float dY = .5*sindir;
		float gridX = carX + dX; //move .5 grid units in direction each iteration
		float gridY = carY + dY;
		//Set current space to probability .1
		if (oGrid[carX][carY] > 0) {
			oGrid[carX][carY] = 1;
			if (OGRIDLOG) logGridEdit(carX,carY,1);
		}
		//Subtract 2 or 4 from spaces before lidar hit depending on how much lidar angle crosses space
		while (abs(gridX-hitX)>1 && abs(gridY-hitY)>1) {
			if (oGrid[(int)gridX][(int)gridY] > 1) {
				oGrid[(int)gridX][(int)gridY] -= 2;
				if (OGRIDLOG) logGridEdit(gridX,gridY,oGrid[(int)gridX][(int)gridY]);
			}
			gridX += dX;
			gridY += dY;
		}
		//Add 3 to actual lidar hit
		if (oGrid[hitX][hitY] < 8) {
			oGrid[hitX][hitY] += 3;
			if (OGRIDLOG) logGridEdit(gridX,gridY,oGrid[hitX][hitY]);
		}
	}
}
