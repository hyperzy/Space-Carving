//
// Created by himalaya on 3/21/19.
//

#include "pc_constraint_carving.h"
#include "show3d.h"
#include "fileio.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <fstream>

using namespace std;

const dtype VAR_THRESH = 50;//221.0;
const unsigned int REMOVAL_THRESH = 50;

Border BORDER;

void pcConstraintsCarving(vector<vector<vector<Voxel>>> &bd_box, list<Point3> &pointcloud, const Border &border, const vector<Mat> &cam_pos, const vector<Mat> &input_img, const vector<Mat> &rotation)
{
	int iteration = 0;
	Init_Bounding(bd_box, border, pointcloud);
	cout << "iteration: " << iteration++ << endl;
	SweepPlane(cam_pos, input_img, bd_box, rotation);

	list<Vec3b> colors;
	FinalizePointcloud(pointcloud, colors, bd_box);
//	Show3D(rotation, cam_pos, pointcloud);
	SaveParam("beforepc.yml", rotation, cam_pos, pointcloud, colors);
	SavePCD("beforepc.pcd", pointcloud, colors);
	while (CheckConsistency(bd_box, VAR_THRESH))
	{
		cout << "iteration: " << iteration++ << endl;
		SweepPlane(cam_pos, input_img, bd_box, rotation);

	}
	FinalizePointcloud(pointcloud, colors, bd_box);
	SaveParam("afterpc.yml", rotation, cam_pos, pointcloud, colors);
	SavePCD("afterpc.pcd", pointcloud, colors);
}

int Init_Bounding(vector<vector<vector<Voxel>>> &bd_box, const Border &border, const list<Point3> &pointcloud)
{
	if (pointcloud.empty())
		cerr << "pointcloud list container is empty!" << endl;
	BORDER = border;
	int xrange = cvCeil((border.xmax - border.xmin) / STEPSIZE) + 1;
	int yrange = cvCeil((border.ymax - border.ymin) / STEPSIZE) + 1;
	int zrange = cvCeil((border.zmax - border.zmin) / STEPSIZE) + 1;

	// initial boudning box
	bd_box.resize(xrange, vector<vector<Voxel>>(yrange, vector<Voxel>(zrange)));

	cout << bd_box.size() << " " << bd_box[1].size() << " " << bd_box[1][1].size() << endl;

//	dtype minimum = FLT_MAX;
	for (auto it : pointcloud)
	{
//		if (it.x < minimum)
//			minimum = it.x;
		uint x, y, z;
		x = (uint)cvRound((it.x - border.xmin) / STEPSIZE);
		y = (uint)cvRound((it.y - border.ymin) / STEPSIZE);
		z = (uint)cvRound((it.z - border.zmin) / STEPSIZE);
		Voxel &p = bd_box[x][y][z];
		p.initial();
		p.coord3->x = it.x;
		p.coord3->y = it.y;
		p.coord3->z = it.z;
	}
//	cout << minimum << endl;
	return 0;
}


// /////////important point to be checked.  very easy to mess up the logics
bool isVisible(const dtype &x_cam, const dtype &y_cam, const dtype &z_cam, int i, int j, int k, const vector<vector<vector<Voxel>>> &bd_box, SweepDir dir)
{
	// if on the direction corresponding border
	if ((i == 0 && dir == xpos) || (j == 0 && dir == ypos) || (k == 0 && dir == zpos)
		|| (i == bd_box.size() - 1 && dir == xneg) || (j == bd_box[0].size() - 1 && dir == yneg) || (k == bd_box[0][0].size() - 1 && dir == zneg))
	{
		 return true;
	}


	// check occlusion relationship
	dtype x_point = bd_box[i][j][k].coord3->x, y_point = bd_box[i][j][k].coord3->y, z_point = bd_box[i][j][k].coord3->z;
	Vec3 p_to_c(x_cam - x_point, y_cam - y_point, z_cam - z_point);
	// unit vector of point to camera
	Vec3 unit_ptc = p_to_c / norm(p_to_c);
//	cout << unit_ptc << endl;
	// along the light line, find if the point is visible from the specific camera
	unsigned int count = 1;
	while (1)
	{
		j = cvRound(((y_point + count * STEPSIZE * unit_ptc[1]) - BORDER.ymin) / STEPSIZE);
		i = cvRound(((x_point + count * STEPSIZE * unit_ptc[0]) - BORDER.xmin) / STEPSIZE);
		k = cvRound(((z_point + count * STEPSIZE * unit_ptc[2]) - BORDER.zmin) / STEPSIZE);
		// if an occlusion point is not found finally, return true.
		if (i < 0 || j < 0 || k < 0 || i >= bd_box.size() || j >= bd_box[0].size() || k >= bd_box[0][0].size())
			return true;
		else
		{
			// if there is a point ahead, the original point will be occluded
			if (bd_box[i][j][k].coord3 != nullptr)
				return false;
			else
			{
//				x_point = i * STEPSIZE + BORDER.xmin;
//				y_point = j * STEPSIZE + BORDER.ymin;
//				z_point	= k * STEPSIZE + BORDER.zmin;
				count++;
				continue;
			}
		}
	}

}

bool isBraced(const vector<vector<vector<Voxel>>> &bd_box, int i, int j, int k, SweepDir dir)
{
	// if six direction are filled with facet
	// 'or' sequence is important otherwise index might exceed the limit
	if ((dir == xpos && i == 0) || (dir == xneg && i == bd_box.size() - 1)
		|| (dir == ypos && j == 0) || (dir == yneg && j == bd_box[0].size() - 1)
		|| (dir == zpos && k == 0) || (dir == yneg && k == bd_box[0][0].size() - 1))
		return true;
	else if ((dir == xneg || (i != 0 && bd_box[i - 1][j][k].coord3 != nullptr)) && (dir == xpos || (i != bd_box.size() - 1 && bd_box[i + 1][j][k].coord3 != nullptr))
		&& (dir == yneg || (j != 0 && bd_box[i][j - 1][k].coord3 != nullptr)) && (dir == ypos || (j != bd_box[0].size() - 1 && bd_box[i][j + 1][k].coord3 != nullptr))
		&& (dir == zneg || (k != 0 && bd_box[i][j][k - 1].coord3 != nullptr)) && (dir == zpos || (k != bd_box[0][0].size() - 1 && bd_box[i][j][k + 1].coord3 != nullptr)))
	{
		return true;
	} 
	else
		return false;
}

// Divide space into six pyramidal regions.
// Due to occlusion relationship, we only consider points at the border
void SweepPlane(const vector<Mat> &cam_pos, const vector<Mat> &input_img_rgb, vector<vector<vector<Voxel>>> &bd_box, const vector<Mat> &rotation)
{
	// ********** get translation vector and rotation vector ***********
	vector<int> vis_cam_index;
	vector<Vec3> tvec, rvec;
	for (auto i : rotation)
	{
		Vec3 temp;
		Rodrigues(i, temp);
		rvec.push_back(temp);
	}
	
	for (int i = 0; i < cam_pos.size(); i++)
	{
		Mat temp = -rotation[i] * cam_pos[i];
		tvec.push_back((Vec3)temp);
	}
	
	// *********** transform RGB color space to Lab color space **************
	vector<Mat> input_img;
	input_img.reserve(cam_pos.size());
	for (auto i : input_img_rgb)
	{
		Mat temp;
		cvtColor(i, temp, COLOR_RGB2Lab);
		input_img.push_back(temp);
	}
	
	// *********** sweep from x negative to x positive ***************
	for (int i = 0; i < bd_box.size(); i++)
	{
		for (int j = 0; j < bd_box[i].size(); j++)
		{
			for (int k = 0; k < bd_box[i][j].size(); k++)
			{
				if (bd_box[i][j][k].coord3 != nullptr)
				{
					if(isBraced(bd_box, i, j, k, xpos))
					{
//						bd_box[i][j][k].remove();
						continue;
					};
					dtype x_point = bd_box[i][j][k].coord3->x;
					dtype y_point = bd_box[i][j][k].coord3->y;
					dtype z_point = bd_box[i][j][k].coord3->z;


					// rows=3, cols = 1, 32F, dims = 2
					for (int index = 0; index < cam_pos.size(); index++)
					{
						dtype x_cam = cam_pos[index].at<dtype>(0, 0);
						dtype y_cam = cam_pos[index].at<dtype>(1, 0);
						dtype z_cam = cam_pos[index].at<dtype>(2, 0);

						// if cam is inside the pyramidal region
						if ((-x_cam - z_cam + x_point + z_point >= 0) && (-x_cam + y_cam + x_point - y_point >= 0)
							&& (-x_cam + z_cam + x_point - z_point > 0) && (-x_cam - y_cam + x_point + y_point > 0))
						{
							if (isVisible(x_cam, y_cam, z_cam, i, j, k, bd_box, xpos))
							vis_cam_index.push_back(index);
						}

						if (vis_cam_index.empty())
							continue;
						else // calculate photo consistency information.
						{
							CalculateColorInfo(bd_box[i][j][k], vis_cam_index, tvec, input_img, rvec, input_img_rgb);
						}
						vis_cam_index.clear();
					}
				}
			}
		}
	}

	// *********** sweep from x positive to x negtive ***************
	for (int i = bd_box.size() - 1; i >= 0; i--)
	{
		for (int j = 0; j < bd_box[i].size(); j++)
		{
			for (int k = 0; k < bd_box[i][j].size(); k++)
			{
				if (bd_box[i][j][k].coord3 != nullptr)
				{
					dtype x_point = bd_box[i][j][k].coord3->x;
					dtype y_point = bd_box[i][j][k].coord3->y;
					dtype z_point = bd_box[i][j][k].coord3->z;

					// rows=3, cols = 1, 32F, dims = 2
					for (int index = 0; index < cam_pos.size(); index++)
					{
						if(isBraced(bd_box, i, j, k, xneg))
						{
//							bd_box[i][j][k].remove();
							continue;
						};
						dtype x_cam = cam_pos[index].at<dtype>(0, 0);
						dtype y_cam = cam_pos[index].at<dtype>(1, 0);
						dtype z_cam = cam_pos[index].at<dtype>(2, 0);

						// if cam is inside the pyramidal region
						if ((x_cam - z_cam - x_point + z_point >= 0) && (x_cam - y_cam - x_point + y_point >= 0)
							&& (x_cam + z_cam - x_point - z_point > 0) && (x_cam + y_cam - x_point - y_point > 0))
						{
							if (isVisible(x_cam, y_cam, z_cam, i, j, k, bd_box, xneg))
								vis_cam_index.push_back(index);
						}

						if (vis_cam_index.empty())
							continue;
						else // calculate photo consistency information.
						{
							CalculateColorInfo(bd_box[i][j][k], vis_cam_index, tvec, input_img, rvec, input_img_rgb);
						}
						vis_cam_index.clear();
					}
				}
			}
		}
	}

	// *********** sweep from y negtive to y positive ***************
	for (int j = 0; j < bd_box[0].size(); j++)
	{
		for (int i = 0; i < bd_box.size(); i++)
		{
			for (int k = 0; k < bd_box[i][j].size(); k++)
			{
				if (bd_box[i][j][k].coord3 != nullptr)
				{
					dtype x_point = bd_box[i][j][k].coord3->x;
					dtype y_point = bd_box[i][j][k].coord3->y;
					dtype z_point = bd_box[i][j][k].coord3->z;

					// rows=3, cols = 1, 32F, dims = 2
					for (int index = 0; index < cam_pos.size(); index++)
					{
						if(isBraced(bd_box, i, j, k, ypos))
						{
//							bd_box[i][j][k].remove();
							continue;
						}
						dtype x_cam = cam_pos[index].at<dtype>(0, 0);
						dtype y_cam = cam_pos[index].at<dtype>(1, 0);
						dtype z_cam = cam_pos[index].at<dtype>(2, 0);

						// if cam is inside the pyramidal region
						if ((-y_cam - z_cam + y_point + z_point >= 0) && (-x_cam - y_cam + x_point + y_point >= 0)
							&& (-y_cam + z_cam + y_point - z_point > 0) && (x_cam - y_cam - x_point + y_point > 0))
						{
							if (isVisible(x_cam, y_cam, z_cam, i, j, k, bd_box, ypos))
								vis_cam_index.push_back(index);
						}

						if (vis_cam_index.empty())
							continue;
						else // calculate photo consistency information.
						{
							CalculateColorInfo(bd_box[i][j][k], vis_cam_index, tvec, input_img, rvec, input_img_rgb);
						}
						vis_cam_index.clear();
					}
				}
			}
		}
	}

	// *********** sweep from y positive to y negative ***************
	for (int j = bd_box[0].size() - 1; j >= 0; j--)
	{
		for (int i = 0; i < bd_box.size(); i++)
		{
			for (int k = 0; k < bd_box[i][j].size(); k++)
			{
				if (bd_box[i][j][k].coord3 != nullptr)
				{
					if(isBraced(bd_box, i, j, k, yneg))
					{
//						bd_box[i][j][k].remove();
						continue;
					};
					dtype x_point = bd_box[i][j][k].coord3->x;
					dtype y_point = bd_box[i][j][k].coord3->y;
					dtype z_point = bd_box[i][j][k].coord3->z;

					// rows=3, cols = 1, 32F, dims = 2
					for (int index = 0; index < cam_pos.size(); index++)
					{
						dtype x_cam = cam_pos[index].at<dtype>(0, 0);
						dtype y_cam = cam_pos[index].at<dtype>(1, 0);
						dtype z_cam = cam_pos[index].at<dtype>(2, 0);

						// if cam is inside the pyramidal region
						if ((y_cam - z_cam - y_point + z_point >= 0) && (x_cam + y_cam - x_point - y_point >= 0)
							&& (y_cam + z_cam - y_point - z_point > 0) && (-x_cam + y_cam + x_point - y_point > 0))
						{
							if (isVisible(x_cam, y_cam, z_cam, i, j, k, bd_box, yneg))
								vis_cam_index.push_back(index);
						}

						if (vis_cam_index.empty())
							continue;
						else // calculate photo consistency information.
						{
							CalculateColorInfo(bd_box[i][j][k], vis_cam_index, tvec, input_img, rvec, input_img_rgb);
						}
						vis_cam_index.clear();
					}
				}
			}
		}
	}

	// *********** sweep from z negative to z positive ***************
	for (int k = 0; k < bd_box[0][0].size(); k++)
	{
		for (int i = 0; i < bd_box.size(); i++)
		{
			for (int j = 0; j < bd_box[i].size(); j++)
			{
				if (bd_box[i][j][k].coord3 != nullptr)
				{
					dtype x_point = bd_box[i][j][k].coord3->x;
					dtype y_point = bd_box[i][j][k].coord3->y;
					dtype z_point = bd_box[i][j][k].coord3->z;

					// rows=3, cols = 1, 32F, dims = 2
					for (int index = 0; index < cam_pos.size(); index++)
					{
						if(isBraced(bd_box, i, j, k, zpos))
						{
//							bd_box[i][j][k].remove();
							continue;
						};
						dtype x_cam = cam_pos[index].at<dtype>(0, 0);
						dtype y_cam = cam_pos[index].at<dtype>(1, 0);
						dtype z_cam = cam_pos[index].at<dtype>(2, 0);

						// if cam is inside the pyramidal region
						if ((x_cam - z_cam - x_point + z_point >= 0) && (y_cam - z_cam - y_point + z_point >= 0)
							&& (-x_cam - z_cam + x_point + z_point >= 0) && (-y_cam - z_cam + x_point + y_point >= 0))
						{
							if (isVisible(x_cam, y_cam, z_cam, i, j, k, bd_box, zpos))
								vis_cam_index.push_back(index);
						}

						if (vis_cam_index.empty())
							continue;
						else // calculate photo consistency information.
						{
							CalculateColorInfo(bd_box[i][j][k], vis_cam_index, tvec, input_img, rvec, input_img_rgb);
						}
						vis_cam_index.clear();
					}
				}
			}
		}
	}

	// *********** sweep from z positive to z negative ***************
	for (int k = bd_box[0][0].size() - 1; k >= 0; k--)
	{
		for (int i = 0; i < bd_box.size(); i++)
		{
			for (int j = 0; j < bd_box[i].size(); j++)
			{
				if (bd_box[i][j][k].coord3 != nullptr)
				{
					if(isBraced(bd_box, i, j, k, zneg))
					{
//						bd_box[i][j][k].remove();
						continue;
					};
					dtype x_point = bd_box[i][j][k].coord3->x;
					dtype y_point = bd_box[i][j][k].coord3->y;
					dtype z_point = bd_box[i][j][k].coord3->z;

					// rows=3, cols = 1, 32F, dims = 2
					for (int index = 0; index < cam_pos.size(); index++)
					{
						dtype x_cam = cam_pos[index].at<dtype>(0, 0);
						dtype y_cam = cam_pos[index].at<dtype>(1, 0);
						dtype z_cam = cam_pos[index].at<dtype>(2, 0);

						// if cam is inside the pyramidal region
						if ((x_cam + z_cam - x_point - z_point >= 0) && (-y_cam + z_cam + y_point - z_point > 0)
							&& (-x_cam + z_cam + x_point - z_point >= 0) && (y_cam + z_cam - y_point - z_point > 0))
						{
							if (isVisible(x_cam, y_cam, z_cam, i, j, k, bd_box, zneg))
								vis_cam_index.push_back(index);
						}

						if (vis_cam_index.empty())
							continue;
						else // calculate photo consistency information.
						{
							CalculateColorInfo(bd_box[i][j][k], vis_cam_index, tvec, input_img, rvec, input_img_rgb);
						}
						vis_cam_index.clear();
					}
				}
			}
		}
	}
}

void CalculateColorInfo(Voxel &voxel, const vector<int> &vis_cam_index, const vector<Vec3> &tvec, const vector<Mat> &input_img, const vector<Vec3> &rvec, const vector<Mat> &input_img_rgb)
{
	vector<Point3> obj_point{Point3(voxel.coord3->x, voxel.coord3->y, voxel.coord3->z)};
	bool flag = false;
	for (auto index : vis_cam_index)
	{
		vector<Point2> img_point;
		projectPoints(obj_point, rvec[index], tvec[index], K, noArray(), img_point);
		int x_img = cvRound(img_point[0].x);
		int y_img = cvRound(img_point[0].y);
		voxel.info_consistency->cam_count++;
		voxel.info_consistency->sum_color_a += (uint)input_img[index].at<Vec3b>(y_img, x_img)[1];
		voxel.info_consistency->sum_color_b += (uint)input_img[index].at<Vec3b>(y_img, x_img)[2];
		voxel.info_consistency->sum_color2_a += (uint)pow(input_img[index].at<Vec3b>(y_img, x_img)[1], 2);
		voxel.info_consistency->sum_color2_b += (uint)pow(input_img[index].at<Vec3b>(y_img, x_img)[2], 2);
//		dtype temp = voxel.info_consistency->sum_color_a;
//		dtype val_a = input_img[index].at<Vec3f>(y_img, x_img)[1];
//		voxel.info_consistency->sum_color_a += (val_a - temp) / voxel.info_consistency->cam_count;
//		voxel.info_consistency->sum_color2_a += (val_a - temp) * (val_a - voxel.info_consistency->sum_color_a);
//		temp = input_img[index].at<Vec3f>(y_img, x_img)[2];
//		voxel.info_consistency->sum_color_b += (val_a - temp) / voxel.info_consistency->cam_count;
//		voxel.info_consistency->sum_color2_b += (val_a - temp) * (val_a - voxel.info_consistency->sum_color_b);

		if (!flag && voxel.color == nullptr)
		{
			voxel.color = new Color;
			voxel.color->b = input_img_rgb[index].at<Vec3b>(y_img, x_img)[0];
			voxel.color->g = input_img_rgb[index].at<Vec3b>(y_img, x_img)[1];
			voxel.color->r = input_img_rgb[index].at<Vec3b>(y_img, x_img)[2];
			flag = true;
		}
	}

}

// return true if there is a voxel removed and false otherwise.
bool CheckConsistency(vector<vector<vector<Voxel>>> &bd_box, dtype threshold)
{
	int removal_count = 0;
	bool flag = false;
//	ofstream fout1, fout2;
//	fout1.open("color_a.txt", ios::out | ios::app);
//	fout2.open("color_b.txt", ios::out | ios::app);
	for (int i = 0; i < bd_box.size(); i++)
	{
		for (int j = 0; j < bd_box[i].size(); j++)
		{
			for (int k = 0; k < bd_box[i][j].size(); k++)
			{
				Voxel &voxel = bd_box[i][j][k];
				if (bd_box[i][j][k].coord3 == nullptr || voxel.info_consistency->cam_count <= 1)
					continue;

				dtype sigma_a2 = (dtype)1.0 / (voxel.info_consistency->cam_count - 1) * ((dtype)voxel.info_consistency->sum_color2_a - (dtype)1.0 / voxel.info_consistency->cam_count * (dtype)pow(voxel.info_consistency->sum_color_a, 2));
				dtype sigma_b2 = (dtype)1.0 / (voxel.info_consistency->cam_count - 1) * ((dtype)voxel.info_consistency->sum_color2_b - (dtype)1.0 / voxel.info_consistency->cam_count * (dtype)pow(voxel.info_consistency->sum_color_b, 2));
//				dtype sigma_a2 = voxel.info_consistency->sum_color2_a;
//				dtype sigma_b2 = voxel.info_consistency->sum_color2_b;
//				fout1 << sigma_a2 << endl;
//				fout2 << sigma_b2 << endl;
				if (sigma_a2 > threshold || sigma_b2 > threshold)
				{
					voxel.remove();
					removal_count++;
					if (removal_count >= REMOVAL_THRESH)
					{
						flag = true;
					}
				}
			}
		}
	}
//	fout1.close();
//	fout2.close();
	cout << removal_count << endl;
	return flag;
}

void FinalizePointcloud(list<Point3> &pointcloud, list<Vec3b> &colors, vector<vector<vector<Voxel>>> &bd_box)
{
	pointcloud.clear();
	colors.clear();
	for (int i = 0; i < bd_box.size(); i++)
	{
		for (int j = 0; j < bd_box[i].size(); j++)
		{
			for (int k = 0; k < bd_box[i][j].size(); k++)
			{
				Voxel &voxel = bd_box[i][j][k];
//				if (voxel.coord3 != nullptr)
				if (voxel.color != nullptr)
					pointcloud.emplace_back(Point3(voxel.coord3->x, voxel.coord3->y, voxel.coord3->z));
				if (voxel.color != nullptr)
					colors.emplace_back(Vec3b(voxel.color->b, voxel.color->g, voxel.color->r));

			}
		}
	}
}