//
// Created by himalaya on 2/11/19.
//

#include "show3d.h"
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#define CERES_FOUND true
#define DEBUG_INFO false

using namespace std;
using namespace cv;


void Show(const vector<Affine3d> &path, const vector<Vec3f> &point_cloud_set)
{
	viz::Viz3d window("3Drecon");
	window.setBackgroundColor();
	window.showWidget("Coordinator", viz::WCoordinateSystem(5));

	if (point_cloud_set.size() > 0)
	{
		cout << "Rendering points   ...";
		viz::WCloud cloud_widget(point_cloud_set, viz::Color::green());
		window.showWidget("point cloud", cloud_widget);
		cout << "[DONE]" << endl;
		cout << "Point number: " << point_cloud_set.size() << endl;
	}
	else
	{
		cout << "Cannot render points: Empty pointcloud" << endl;
	}
	if (path.size() > 0)
	{
		cout << "Rendering Cameras  ... ";

		window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
		window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));

		window.setViewerPose(path[0]);

		cout << "[DONE]" << endl;
	}
	else
	{
		cout << "Cannot render the cameras: Empty path" << endl;
	}
	window.spin();
}

void Show3D(string pc_filepath)
{
	FileStorage fs(pc_filepath, FileStorage::READ);
	vector<Vec3f> point_cloud_set;
	vector<Affine3d> path;
	//Matx33d K = Matx33d(2873.841523698976, 0, 1647.510385915514,
	//	0, 2868.390255492409, 1207.559607221417,
	//	0, 0, 1);
	/*Matx33d K = Matx33d(2769.316363, 0, 1632.0,
		0, 2769.316363, 1224.0,
		0, 0, 1);*/
	int camera_count;
	fs["Camera Count"] >> camera_count;
	cout << camera_count << endl;
	FileNode r_node = fs["Rotations"];
	FileNode t_node = fs["Translations"];
	FileNode p_node = fs["Points"];
	if (r_node.type() != FileNode::SEQ || t_node.type() != FileNode::SEQ)
	{
		cerr << "Node type error!" << endl;
		exit(1);
	}
	for (FileNodeIterator it1 = r_node.begin(), it2 = t_node.begin(); it1 != r_node.end() && it2 !=t_node.end(); it1++, it2++)
	{
		Mat rotation, translation;
		*it1 >> rotation;
		*it2 >> translation;
		path.push_back(Affine3d(rotation, translation));
#if DEBUG_INFO
		cout << rotation[i] << endl;
		printf("The %d matrix was pushed into rotation matrix vector\n", i);
#endif
	}

	for (FileNodeIterator it = p_node.begin(); it != p_node.end(); it++)
	{
		Point3f p;
		*it >> p;
		if (fabs(p.x) > 100 || fabs(p.y) > 100 || fabs(p.z) > 100)
			continue;
		point_cloud_set.push_back((Vec3f)p);
	}

	Show(path, point_cloud_set);

}

void Show3D(const vector<Mat> &rotation, const vector<Mat> &translation, const list<Point3> &point_cloud)
{
	vector<Vec3f> point_cloud_set;
	vector<Affine3d> path;
	//Matx33d K = Matx33d(2873.841523698976, 0, 1647.510385915514,
	//	0, 2868.390255492409, 1207.559607221417,
	//	0, 0, 1);
	/*Matx33d K = Matx33d(2769.316363, 0, 1632.0,
		0, 2769.316363, 1224.0,
		0, 0, 1);*/

	for (int i = 0; i < rotation.size(); i++)
	{
		Vec3f r;
		Rodrigues(rotation[i], r);
		path.push_back(Affine3d(r, translation[i]));
	}

	for (auto iter : point_cloud)
	{
		if (fabs(iter.x) > 100 || fabs(iter.y) > 100 || fabs(iter.z) > 100)
			continue;
		point_cloud_set.push_back((Vec3f)iter);
	}

	Show(path, point_cloud_set);
}
