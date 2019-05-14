//
// Created by himalaya on 2/9/19.
//

#ifndef SPACE_CARVING_EXTRACT_CONTOUR_H
#define SPACE_CARVING_EXTRACT_CONTOUR_H

#include <opencv2/core.hpp>
#include <set>

using namespace std;
using namespace cv;
//Mat PreProcess(Mat &src);
//Mat SobelGradient(Mat &blurred_img);
//Mat FindSilhouette(Mat &gradXY);

//class MyPoint: public Point
//{
//public:
//	MyPoint(int a, int b)
//	{
//		x = a;
//		y = b;
//	}
//	bool operator < (const Point &rhs) const
//	{
//		if (this->x < rhs.x)
//			return true;
//		else if (this->x == rhs.x) {
//			if (this->y < rhs.y)
//				return true;
//			else
//				return false;
//		}
//		else
//			return false;
//	}
//	bool operator == (const Point &rhs) const
//	{
//		return (this->x == rhs.x) && (this->y == rhs.y);
//	}
//
//
//};

class PointSort
{
public:
	bool operator()(const Point &a, const Point &b) const;
};

#define EXTRACTION 1
#define USE_CONTOUR 2

// Extract the contour of one image.
// return binary image of contour.
Mat ExtractContour(string &img_path, int idx, int type);



#endif //SPACE_CARVING_EXTRACT_CONTOUR_H
