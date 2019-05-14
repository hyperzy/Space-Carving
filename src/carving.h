//
// Created by himalaya on 2/11/19.
//

#ifndef SPACE_CARVING_CARVING_H
#define SPACE_CARVING_CARVING_H

#include <opencv2/core.hpp>
#include <list>
#include <vector>

using namespace cv;
typedef Point3f Point3;
typedef Point2f Point2;
typedef Vec2f	Vec2;
typedef Vec3f	Vec3;
typedef float	dtype;

// although in Lab color space, a and b channel are denoted by uchar, but we have to comput the summation
struct infoConsistency{
	unsigned int cam_count;
	uint sum_color_a;
	uint sum_color2_a;
	uint sum_color_b;
	uint sum_color2_b;
};

struct Coord3{
	dtype x, y ,z;
};

struct Color{
	uchar b, g, r;
};
struct Voxel{
	Coord3 *coord3;
	infoConsistency *info_consistency;
	Color *color;
	void initial();
	void remove();
	Voxel();
	~Voxel();
};
//const Matx33d K = Matx33d(2720, 0, 1632.0,
//					0, 2720, 1224.0,
//					0, 0, 1);
//const Matx33d K = Matx33d(3310.4, 0, 316.73,
//						  0, 3325.5, 200.55,
//						  0, 0, 1);

const Matx33d K = Matx33d(1680.2631413061415, 0, 621.59194200994375,
							0, 1676.1202869984309, 467.7223229477861,
							0, 0, 1);

//const dtype XRANGE0 = -1.21639;
//const dtype XRANGE1 = 23.62138;
//const dtype YRANGE0 = -30.2796;
//const dtype YRANGE1 = 12.1731;
//const dtype ZRANGE0 = 0;
//const dtype ZRANGE1 = 40.5358;
//const dtype STEPSIZE = 0.2;
const dtype XRANGE0 = -7.5;
const dtype XRANGE1 = 7.5;
const dtype YRANGE0 = -11.0;
const dtype YRANGE1 = 12.0;
const dtype ZRANGE0 = -3.0;
const dtype ZRANGE1 = 23.0;
const dtype STEPSIZE = 0.1;

void Carving(std::vector<std::string> &img_paths, std::vector<Mat> &rotation, std::vector<Mat> &translation, std::list<Point3> &pointcloud);
#endif //SPACE_CARVING_CARVING_H
