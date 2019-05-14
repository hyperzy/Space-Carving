//
// Created by himalaya on 3/28/19.
//

#include "fileio.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <opencv2/calib3d/calib3d.hpp>
//#include <pcl/io/pcd_io.h>



using namespace std;
using namespace cv;

void GetFileNames(string folder_path, vector<string> &imgs_path)
{
	imgs_path.clear();
//	string cmd = "pwd";
	string cmd = "cd " + folder_path;
	cmd = cmd + "&& dir -b -1 *.jpg>name.txt";
	system(cmd.c_str());

	string indexFile = folder_path + "/name.txt";
	ifstream in(indexFile);
	string line;
	stringstream ss;
	if (!in.is_open())
	{
		cerr << "cannot open index file" << endl;
		exit(1);
	}
	else
	{
		getline(in, line);
	}
	ss.str(line);
	string temp;
	while (!ss.eof())
	{
		ss >>  temp;
		if (temp.empty() || line.empty())
			break;
		imgs_path.push_back(folder_path + "/" + temp);
		ss.clear();
		getline(in, line);
		ss.str(line);
	}
	in.close();
}

void ReadParam(string filename, vector<Mat> &rotation, vector<Mat> &translation, int type) {
	if (type == 0) // photos taken by iPhone6p
	{
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
		return;
	}
	if (type == 1) // dinosaur data
	{
		ifstream fin("dinoSR_par.txt");
		string line;
		stringstream ss;
		getline(fin, line);
		for (int i = 0; i < 16; i++)
		{
			getline(fin, line);
			ss.str(line);
			string tmp_s;
			float tmp_f;
			ss >> tmp_s;
			for (int j = 0; j < 9; j++)
				ss >> tmp_f;
			Mat r = Mat::zeros(3, 3, CV_64FC1);
			for (int j = 0; j < 3; j ++)
			{
				for (int k = 0; k < 3; k++)
					ss >> r.at<double>(j, k);
			}
//			cout << r << endl;
			rotation.push_back(r);
			Mat t = Mat::zeros(3, 1, CV_64FC1);
			for (int j = 0; j < 3; j++)
			{
				ss >> t.at<double>(j, 0);
			}
//			cout << t << endl;
			translation.push_back(t);
			ss.clear();
//			cout << t.depth() << t.type();
		}

	}
	if (type == 2) // squirrel data
	{
		FileStorage fs("/home/himalaya/Desktop/Voxel-Carving-master/assets/viff.xml", FileStorage::READ);
		Mat P;
		stringstream ss;
		string s;
		for (int i = 0; i < 36; i++)
		{
			ss << setfill('0') << setw(3) << i;
			ss >> s;
			fs["viff" + s + "_matrix"] >> P;
//			cout << P << endl;
			Mat k, R, T;
//			Mat R = Mat::zeros(3, 3, DataType<double>::type);
//			Mat T = Mat::zeros(4, 1, DataType<double>::type);
			decomposeProjectionMatrix(P, k, R, T);
			Mat T3 = Mat::zeros(3, 1, T.type());
//			Mat R3 = Mat::zeros(3, 3, DataType<double>::type);
//			for (int j = 0; j < 3; j++)
//			{
//				for (int k = 0; k < 3; k++)
//				{
//					R3.at<double>(j, k) = R.at<double>(j, k);
//				}
//			}
//			cout << R << R.depth() << R.type() << T.depth() << T.type() << endl;

			T3.at<float>(0, 0) = T.at<float>(0, 0) / T.at<float>(3, 0);
			T3.at<float>(1, 0) = T.at<float>(1, 0) / T.at<float>(3, 0);
			T3.at<float>(2, 0) = T.at<float>(2, 0) / T.at<float>(3, 0);
//			cout << T << endl << T3 << endl;
//			cout << R  << endl;
			rotation.push_back(R);
			translation.push_back(T3);
			ss.clear();
		}
		list<Point3> a;
		SaveParam("test.yml", rotation, translation, a);
		rotation.clear();
		translation.clear();
		ReadParam("test.yml", rotation, translation, 0);
	}
}

void SaveParam(string filename, vector<Mat> &rotation, vector<Mat> &translation, list<Point3> &pointcloud)
{
	int n = (int)rotation.size();
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "Camera Count" << n;
	fs << "Rotations" << "[";
	for (int i = 0; i < n; i++)
	{
		fs << rotation[i];
	}
	fs << "]";

	fs << "Motions" << "[";
	for (int i = 0; i < n; i++)
	{
		fs << translation[i];
	}
	fs << "]";

	fs << "Points" << "[";
	for (auto iter : pointcloud)
	{
		fs << iter;
	}
	fs << "]";

	fs.release();
}

void SaveParam(string filename, const vector<Mat> &rotation, const vector<Mat> &translation, const list<Point3> &pointcloud, const list<Vec3b> &colors)
{
	int n = (int)rotation.size();
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "Camera Count" << n;
	fs << "Rotations" << "[";
	for (int i = 0; i < n; i++)
	{
		fs << rotation[i];
	}
	fs << "]";

	fs << "Motions" << "[";
	for (int i = 0; i < n; i++)
	{
		fs << translation[i];
	}
	fs << "]";

	fs << "Points" << "[";
	for (auto iter : pointcloud)
	{
		fs << iter;
	}
	fs << "]";

	fs << "Colors" << "[";
	for (auto iter : colors)
	{
		fs << iter;
	}
	fs << "]";

	fs.release();


}

void SavePCD(string filename, const list<Point3> &pointcloud, const list<Vec3b> &colors)
{
	if (pointcloud.size() != colors.size())
		cerr << "pointcloud size is not equal to size of color structure\n";
	ofstream fout(filename, ios::out);
	fout << "# .PCD v0.7 - Point Cloud Data file format" << endl;
	fout << "VERSION 0.7" << endl;
	fout << "FIELDS x y z rgb" << endl;
	fout << "SIZE 4 4 4 4" << endl;
	fout << "TYPE F F F F" << endl;
	fout << "COUNT 1 1 1 1" << endl;
	fout << "WIDTH " << pointcloud.size() << endl;
	fout << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	fout << "POINTS " << pointcloud.size() << endl;
	fout << "DATA ascii" << endl;
	auto iter1 = pointcloud.begin();
	auto iter2 = colors.begin();
	for (; iter1 != pointcloud.end(); iter1++, iter2++)
	{
		fout << (*iter1).x << " " << (*iter1).y << " " << (*iter1).z << " ";
		Vec3b v_bgr = *iter2;
		int bgr = (int)v_bgr[2] << 16 | (int)v_bgr[1] << 8 | (int)v_bgr[0];
		float fbgr = *reinterpret_cast<float*>(&bgr);
		fout << fbgr << endl;
	}
	fout.close();
}