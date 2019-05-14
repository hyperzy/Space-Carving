//
// Created by himalaya on 2/11/19.
//

#ifndef SPACE_CARVING_SHOW3D_H
#define SPACE_CARVING_SHOW3D_H

#include "carving.h"
#include <vector>
#include <list>

using namespace std;

// read existing data to show 3D point cloud
void Show3D(string pc_filepath);

// using data on memory to show 3D point cloud
void Show3D(const vector<Mat> &rotation, const vector<Mat> &translation, const list<Point3> &point_cloud);

#endif //SPACE_CARVING_SHOW3D_H
