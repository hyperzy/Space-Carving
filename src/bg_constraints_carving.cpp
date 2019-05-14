//
// Created by himalaya on 2/11/19.
//

#include "bg_constraints_carving.h"
#include <opencv2/opencv.hpp>
#include <list>
#include <cmath>
#include <cfloat>

using namespace std;

// determin if the projection is inside the silhouette.
// return true if the point is inside it.
// *** NOTES: x corresponds with width, y corresponds with height. ******** IMPORTANT ********
inline bool isInside(const Mat contour, const int &x, const int &y)
{
	if (x < 0 || y < 0 || x >= contour.size().width || y >= contour.size().height)
		return false;
	else if (contour.at<uchar>(y, x) == 0)
		return false;
	else
		return true;
}


void bgConstraintsCarving(const Mat &contour, const Mat &rotation, const Mat &translation, list<Point3> &pointcloud, Border &border)
{
	Vec3 rvec, tvec;
	Rodrigues(rotation, rvec);
	Mat temp = -rotation * translation;
//	cout << temp << endl;
	tvec = (Vec3)temp;
	Mat img_point;
	for (auto it = pointcloud.begin(); it != pointcloud.end(); )
	{
		Mat point{Mat_<Point3>((*it))};
//		cout << point << " " << point.size() << " " << point.type() << " " << point.channels() << " " << point.depth() << " " << point.checkVector(3) << endl;
		projectPoints(point, rvec, tvec, K, noArray(), img_point);
//		Mat E = Mat::zeros(3, 4, rotation.type());
//		for (int i = 0; i < 3; i++)
//		{
//			for (int j = 0; j < 3; j++)
//			{
//				E.at<float>(i, j) = rotation.at<float>(i,j);
//			}
//		}
//		for (int i = 0; i < 3; i++)
//			E.at<float>(i, 3) = translation.at<float>(i, 0);

//		cout << E << endl;
//		Mat k = (Mat)K;
//		cout << K;
//		Mat P = K * E;
//		cout << P << endl;
//		Mat proj = P * point;
//		cout << proj;
		int x = cvRound(img_point.at<Vec2>(0)[0]);
		int y = cvRound(img_point.at<Vec2>(0)[1]);
//		cout << contour.type() << " " << contour.channels() << contour.depth() << endl;
		if (!isInside(contour, x, y))
		{
			it = pointcloud.erase(it);
			continue;
		}
		// if the projection is inside the contour
		else
		{
			if ((*it).x > border.xmax) border.xmax = (*it).x;
			if ((*it).x < border.xmin) border.xmin = (*it).x;
			if ((*it).y > border.ymax) border.ymax = (*it).y;
			if ((*it).y < border.ymin) border.ymin = (*it).y;
			if ((*it).z > border.zmax) border.zmax = (*it).z;
			if ((*it).z < border.zmin) border.zmin = (*it).z;
		}
//		cout << img_point << " " << img_point.channels() << endl;
		int pp = 0;
		it++;

	}
//	cout << point0 << " " << point0.channels() << endl;

	cout << "after bg, xmin: " << border.xmin << endl;
}

void bgConstraintsCarving(const Mat &contour, const Mat &rotation, const Mat &translation, list<Point3> &pointcloud)
{
	Vec3 rvec, tvec;
	Rodrigues(rotation, rvec);
	Mat temp = -rotation * translation;
//	cout << temp << endl;
	tvec = (Vec3)temp;
	Mat img_point;
	for (auto it = pointcloud.begin(); it != pointcloud.end(); )
	{
		Mat point{Mat_<Point3>((*it))};
//		cout << point << " " << point.size() << " " << point.type() << " " << point.channels() << " " << point.depth() << " " << point.checkVector(3) << endl;
		projectPoints(point, rvec, tvec, K, noArray(), img_point);
//		Mat E = Mat::zeros(3, 4, rotation.type());
//		for (int i = 0; i < 3; i++)
//		{
//			for (int j = 0; j < 3; j++)
//			{
//				E.at<float>(i, j) = rotation.at<float>(i,j);
//			}
//		}
//		for (int i = 0; i < 3; i++)
//			E.at<float>(i, 3) = translation.at<float>(i, 0);

//		cout << E << endl;
//		Mat k = (Mat)K;
//		cout << K;
//		Mat P = K * E;
//		cout << P << endl;
//		Mat proj = P * point;
//		cout << proj;
		int x = cvRound(img_point.at<Vec2>(0)[0]);
		int y = cvRound(img_point.at<Vec2>(0)[1]);
//		cout << contour.type() << " " << contour.channels() << contour.depth() << endl;
		if (!isInside(contour, x, y))
		{
			it = pointcloud.erase(it);
			continue;
		}
//		cout << img_point << " " << img_point.channels() << endl;
		int pp = 0;
		it++;

	}
//	cout << point0 << " " << point0.channels() << endl;

}

Border::Border()
{
	xmin = ymin = zmin = FLT_MAX;
	xmax = ymax = zmax = FLT_MIN;
}