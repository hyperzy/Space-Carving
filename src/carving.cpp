//
// Created by himalaya on 2/12/19.
//

#include "carving.h"
#include "extract_contour.h"
#include "bg_constraints_carving.h"
#include "pc_constraint_carving.h"
#include "show3d.h"
#include "fileio.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>

//#define TESTING

using namespace std;

void SaveBorder(const string filename, const Border &border)
{
	FileStorage fs(filename, FileStorage::WRITE);
	if (!fs.isOpened())
		cerr << "cannot open file " << filename << endl;
	fs << "xmin" << border.xmin;
	fs << "xmax" << border.xmax;
	fs << "ymin" << border.ymin;
	fs << "ymax" << border.ymax;
	fs << "zmin" << border.zmin;
	fs << "zmax" << border.zmax;
	fs.release();
}
void ReadCloud(const string filename1, const string filename2, list<Point3> &pointcloud, Border &border)
{
	FileStorage fs(filename1, FileStorage::READ);
	if (!fs.isOpened())
		cerr << "cannot open file " << filename1 << endl;

	FileNode p_node = fs["Points"];
	for (auto it = p_node.begin(); it != p_node.end(); it++)
	{
		Point3 point;
		*it >> point;
		pointcloud.push_back(point);
	}

	fs.release();
	fs.open(filename2, FileStorage::READ);
	if (!fs.isOpened())
		cerr << "cannot open file " << filename2 << endl;
	fs["xmin"] >> border.xmin;
	fs["xmax"] >> border.xmax;
	fs["ymin"] >> border.ymin;
	fs["ymax"] >> border.ymax;
	fs["zmin"] >> border.zmin;
	fs["zmax"] >> border.zmax;
	fs.release();
}

void ChangeContainer(list<Point3> &pointcloud, const vector<vector<vector<Voxel>>> &bd_box)
{
	// here we should use reference. I have not figured out the exact reason.
	for (auto &i : bd_box)
	{
		for (auto &j : i)
		{
			for (auto &k : j)
			{
				if (k.coord3 != nullptr)
				{
					Point3 point;
					point.x = k.coord3->x;
					point.y = k.coord3->y;
					point.z = k.coord3->z;
					pointcloud.push_back(point);
				}

			}
		}
	}
}
void Carving(std::vector<std::string> &img_paths, std::vector<Mat> &rotation, std::vector<Mat> &translation, std::list<Point3> &pointcloud)
{
#ifndef TESTING
//	Show3D(rotation, translation,pointcloud);
	vector<Mat> contour_set;
	Border border;
//	img_paths.resize(4);
	for (int i = 0; i < img_paths.size(); i++)
	{
		Mat contour = ExtractContour(img_paths[i], i, USE_CONTOUR);
		contour_set.push_back(contour);
	}
	for (int i = 0; i < img_paths.size(); i++)
	{
		cout << "Carving img" << i << "   ..." << endl;
		bgConstraintsCarving(contour_set[i], rotation[i], translation[i], pointcloud);
		if (i == img_paths.size() - 1)
			bgConstraintsCarving(contour_set[i], rotation[i], translation[i], pointcloud, border);
//		if (i >= 32)
//		{Show3D(rotation, translation,pointcloud); namedWindow("contour",0); imshow("contour", contour_set[i]); waitKey(0); cout << rotation[i] << "\n" << translation[i] <<endl;}
	}
//	Show3D(rotation, translation,pointcloud);
	SaveBorder("border.yml", border);
	SaveParam("output.yml", rotation, translation, pointcloud);
//#else
	vector<Mat> input_img; //  temporarily usage
//	img_paths.resize(4);
	for (auto path : img_paths)
	{
		input_img.emplace_back(cv::imread(path));
	}
//	Border border;
// 	pointcloud.clear();
//	ReadCloud("output.yml", "border.yml", pointcloud, border);
//	Show3D(rotation, translation, pointcloud);
	vector<vector<vector<Voxel>>> bd_box;
	pcConstraintsCarving(bd_box, pointcloud, border, translation, input_img, rotation);
	// here clear pointcloud to free more space because we dont need it anymore but need bd_box instead
//	pointcloud.clear();
//	ChangeContainer(pointcloud, bd_box);
	Show3D(rotation, translation, pointcloud);
	SaveParam("output-pc.yml", rotation, translation, pointcloud);

#endif
}

Voxel::Voxel()
{
	coord3 = nullptr;
	info_consistency = nullptr;
	color = nullptr;
}
void Voxel::initial()
{
	coord3 = new Coord3;
	info_consistency = new infoConsistency;
//	color = new Color;
	info_consistency->cam_count = 0;
	info_consistency->sum_color_a = 0;
	info_consistency->sum_color2_a = 0;
	info_consistency->sum_color_b = 0;
	info_consistency->sum_color2_b = 0;
}
Voxel::~Voxel()
{
	if (info_consistency != nullptr)
	{
		delete info_consistency;
	}
	if (coord3 != nullptr)
	{
		delete coord3;
	}
	if (color != nullptr)
		delete color;
}

void Voxel::remove()
{
	if (info_consistency != nullptr)
	{
		delete info_consistency;
		info_consistency = nullptr;
	}
	if (coord3 != nullptr)
	{
		delete coord3;
		coord3 = nullptr;
	}
	if (color != nullptr)
	{
		delete color;
		color = nullptr;
	}
}
