#include <cstdlib>
//#include <stdlib.h>
#include <iostream>
#include "src/carving.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <list>
#include "src/carving.h"
#include "src/fileio.h"

using namespace std;

void InitShape(list<Point3> &pointcloud);

// used to change traslation vector to camera position
void ChangeTranslation(string filepath);


int main() {
	string folder_path = "imags";
	vector<string> imgs_paths;
	GetFileNames(folder_path, imgs_paths);

//	ChangeTranslation("structure.yml");
    vector<Mat> rotation, translation;
    ReadParam("test4.yml", rotation, translation, 0);
//	ReadParam("out.yml", rotation, translation, 0);
	list<Point3> pointcloud;
    InitShape(pointcloud);
    Carving(imgs_paths, rotation, translation, pointcloud);
//	Show3D("output.yml");
//	Show3D(rotation, translation, pointcloud);
//	bgConstraintsCarving(contour, rotation, translation, pointcloud);
//	Show3D(rotation, translation, pointcloud);


	return 0;
}



void InitShape(list<Point3> &pointcloud)
{
    for (double x = XRANGE0; x <= XRANGE1; x += STEPSIZE)
    {
        for (double y = YRANGE0; y <= YRANGE1; y+= STEPSIZE)
        {
            for (double z = ZRANGE0; z <= ZRANGE1; z+= STEPSIZE)
            {
                pointcloud.push_back(Point3(x, y, z));
            }
        }
    }

}

void ChangeTranslation(string filename)
{
	vector<Mat> rotation, translation, cam_pos;
	FileStorage fs(filename, FileStorage::READ);
	FileNode r_node = fs["Rotations"];
	FileNode t_node = fs["Motions"];
	if (r_node.type() != FileNode::SEQ || t_node.type() != FileNode::SEQ) {
		cout << "Node type error!" << endl;
		return;
	}
	for (FileNodeIterator it1 = r_node.begin(), it2 = t_node.begin();
		 it1 != r_node.end() && it2 != t_node.end(); it1++, it2++) {
		Mat r, t;
		*it1 >> r;
		*it2 >> t;
		rotation.push_back(r);
		translation.push_back(t);
	}
	fs.release();

	for (int i = 0; i < rotation.size(); i++)
	{
		cam_pos.emplace_back(- rotation[i].inv() * translation[i]);
	}

	FileStorage fout("out.yml", FileStorage::WRITE);
	int n = (int)rotation.size();
	fout << "Camera Count" << n;
	fout << "Rotations" << "[";
	for (int i = 0; i < n; i++)
	{
		fout << rotation[i];
	}
	fout << "]";

	fout << "Motions" << "[";
	for (int i = 0; i < n; i++)
	{
		fout << cam_pos[i];
	}
	fout << "]";

	fout.release();
}


