//
// Created by himalaya on 2/11/19.
//
// Last modified : 3/20/19
//

#ifndef SPACE_CARVING_BG_CONSTRAINTS_CARVING_H
#define SPACE_CARVING_BG_CONSTRAINTS_CARVING_H

#include "carving.h"
#include "extract_contour.h"
#include <list>

using namespace std;

struct Border{
	Border();
	dtype xmin, xmax, ymin, ymax, zmin, zmax;
};

void bgConstraintsCarving(const Mat &contour, const Mat &rotation, const Mat &translation, list<Point3> &pointcloud, Border &border);
void bgConstraintsCarving(const Mat &contour, const Mat &rotation, const Mat &translation, list<Point3> &pointcloud);

#endif //SPACE_CARVING_BG_CONSTRAINTS_CARVING_H
