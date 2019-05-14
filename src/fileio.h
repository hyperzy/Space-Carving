//
// Created by himalaya on 3/28/19.
//

#ifndef SPACE_CARVING_FILEIO_H
#define SPACE_CARVING_FILEIO_H

#include "carving.h"
#include <vector>
#include <list>

using namespace std;


void GetFileNames(string folder_path, vector<string> &imgs_path);
void ReadParam(string filename, vector<Mat> &rotation, vector<Mat> &translation, int type);
void SaveParam(string filename, vector<Mat> &rotation, vector<Mat> &translation, list<Point3> &pointcloud);
void SaveParam(string filename, const vector<Mat> &rotation, const vector<Mat> &translation, const list<Point3> &pointcloud, const list<Vec3b> &colors);
void SavePCD(const vector<vector<vector<Voxel>>> &bd_box, const vector<Mat> &input_img, const vector<Mat> &rotation, const vector<Mat> &translation);
void SavePCD(string filename, const list<Point3> &pointcloud, const list<Vec3b> &colors);


#endif //SPACE_CARVING_FILEIO_H
