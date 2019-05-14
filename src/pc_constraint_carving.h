//
// Created by himalaya on 3/21/19.
//

#ifndef SPACE_CARVING_PC_CONSTRAINT_CARVING_H
#define SPACE_CARVING_PC_CONSTRAINT_CARVING_H

#include "carving.h"
#include "bg_constraints_carving.h"
#include <vector>

using namespace std;

struct Camera{

};

enum SweepDir{ xpos, xneg, ypos, yneg, zpos, zneg};

void pcConstraintsCarving(vector<vector<vector<Voxel>>> &bd_box, list<Point3> &pointcloud, const Border &border, const vector<Mat> &cam_pos, const vector<Mat> &input_img, const vector<Mat> &rotation);

// Creating a bounding box around the model after caving with background constraints. And fill it with existing points
// param 1 is a 3D vector storing voxel coord and photo consistent info
// param 2 is the limits info
// param 3 is the pointcloud list
// return 0 if success
int Init_Bounding(vector<vector<vector<Voxel>>> &bd_box, const Border &border, const list<Point3> &pointcloud);

// Divide space into six pyramidal regions.
// Due to occlusion relationship, we only consider points at the border.
// complete calculating photo-consistency info
void SweepPlane(const vector<Mat> &cam_pos, const vector<Mat> &input_img_rgb, vector<vector<vector<Voxel>>> &bd_box, const vector<Mat> &rotation);

bool isVisible(const dtype &x_cam, const dtype &y_cam, const dtype &z_cam, int i, int j, int k, const vector<vector<vector<Voxel>>> &bd_box, SweepDir dir);

bool isBraced(const vector<vector<vector<Voxel>>> &bd_box, int i, int j, int k);

void CalculateColorInfo(Voxel &voxel, const vector<int> &vis_cam_index, const vector<Vec3> &tvec, const vector<Mat> &input_img, const vector<Vec3> &rvec, const vector<Mat> &input_img_rgb);

bool CheckConsistency(vector<vector<vector<Voxel>>> &bd_box, dtype threshold);

void FinalizePointcloud(list<Point3> &pointcloud, list<Vec3b> &colors, vector<vector<vector<Voxel>>> &bd_box);

#endif //SPACE_CARVING_PC_CONSTRAINT_CARVING_H
