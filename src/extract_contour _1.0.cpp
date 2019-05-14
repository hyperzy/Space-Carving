//
// Created by himalaya on 2/9/19.
//

#include "extract_contour.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

#define MINIMUM_CNTOUR_POINT 20

//Mat PreProcess(Mat &img)
//{
//    Mat gray;
//    cvtColor(img, gray, COLOR_BGR2GRAY);
//    Mat out;
//    GaussianBlur(gray,out,Size(9, 9), 0);
//    return out;
//}
//
//Mat SobelGradient(Mat &blurred_img)
//{
//    Mat gradX, gradY, gradXY;
//    Sobel(blurred_img, gradX, gradX.depth(), 1, 0);
//    Sobel(blurred_img, gradY, gradY.depth(), 0, 1);
//    subtract(gradX, gradY, gradXY);
//    convertScaleAbs(gradXY, gradXY);
//    return gradXY;
//}
//
//Mat FindSilhouette(Mat &gradXY)
//{
//    Mat blurred, biImg;
//    GaussianBlur(gradXY, blurred, Size(9, 9), 0);
//    threshold(gradXY, biImg, 90, 255, THRESH_BINARY);
//    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(25, 25));
//    morphologyEx(biImg, biImg, MORPH_CLOSE, kernel);
////    erode(biImg, biImg, noArray(), Point(-1, -1), 4);
////    dilate(biImg, biImg, noArray(), Point(-1, -1), 4);
//    return biImg;
//}

Rect rect;
Mat src, dst;
string window_name;

bool PointSort::operator()(const Point &a, const Point &b) const
{
	if ((a.x < b.x) || ((a.x == b.x) && (a.y < b.y)))
		return true;
	else
		return false;
}
// mouse event handler. It can be improved with adding key interrupt
void onMouse(int event, int x, int y, int flags, void *param);
// save contour info
void SavingContour(const vector<set<Point, PointSort>> &bst_contours);
// using BST to store contour info for convenience of searching
vector<set<Point, PointSort>> ChangeDS(vector<vector<Point>> &contours);

vector<set<Point, PointSort>> ExtractContour(vector<string> &img_paths)
{
    src = imread(img_paths[0]);
//    window_name = "img" + to_string(0);
//    namedWindow(window_name, 0);
//    setMouseCallback(window_name, onMouse);
//    imshow(window_name, src);
//    while (1)
//    {
//        // press enter to jump
//        if (waitKey(0) == 13)
//            break;
//    }
//    destroyAllWindows();
//
//    // ************* Use Grabcut to extract contour ****************
//    Mat result, fg, bg;
//    grabCut(src, result, rect, bg, fg, 10, GC_INIT_WITH_RECT);
//    compare(result, GC_PR_FGD, result, CMP_EQ);
//    namedWindow("result", 0);
//    imshow("result", result);
//    Mat foreground(src.size(), CV_8UC3, Scalar(255, 255, 255));
//    src.copyTo(foreground, result);
//    namedWindow("foreground", 0);
//    imshow("foreground", foreground);
//
//    imwrite("bi.jpg", result);

    // ************* Extract and save contour coordinates **************
    Mat result = imread("bi.jpg", IMREAD_GRAYSCALE);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(result, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    Mat cntour = Mat::zeros(src.size(), CV_8UC3);
    for (int i = 0; i < hierarchy.size(); i++)
    {
    	if (contours[i].size() <= MINIMUM_CNTOUR_POINT)
    		continue;
        Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
        drawContours(cntour, contours, i, color, 1, 8, hierarchy);
    }
    namedWindow("contours", 0);
    imshow("contours", cntour);
    waitKey(0);
    destroyAllWindows();

    vector<set<Point, PointSort>> bst_contours = ChangeDS(contours);
    SavingContour(bst_contours);
    return bst_contours;
}
void onMouse(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        rect = Rect(x, y , 0 , 0);
        src.copyTo(dst);
        circle(dst, Point(x, y), 2, Scalar(255, 0, 0), FILLED);
        imshow(window_name, dst);
    }
//    // renew rectangular
//    else if (event == EVENT_MOUSEMOVE && !(flags & EVENT_FLAG_LBUTTON))
//    {
//        src.copyTo(dst);
//        imshow(window_name,dst);
//    }
    // draw rect when mouse is moving
    else if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))
    {
        src.copyTo(dst);
        rectangle(dst, Point(rect.x, rect.y), Point(x, y), Scalar(255, 0, 0), 2);
        imshow(window_name, dst);
    }
    else if (event == EVENT_LBUTTONUP)
    {
        src.copyTo(dst);
        rectangle(dst, Point(rect.x, rect.y), Point(x, y), Scalar(255, 0, 0), 2);
        rect.width = abs(x - rect.x);
        rect.height = abs(y - rect.y);
        rect.x = min(rect.x, x);
        rect.y = min(rect.y, y);
        imshow(window_name, dst);
    }
}

void SavingContour(const vector<set<Point, PointSort>> &bst_contours)
{
    FileStorage fs("contours.yml", FileStorage::WRITE);
    fs << "Contour count" << (int)bst_contours.size();
    fs << "Contour coordinate info" << "[";
    for (int i = 0; i < bst_contours.size(); i++)
    {
        fs << "Contour idx" << i;
        fs << "Sub-contour coordinate" << "[";
        for (auto &iter : bst_contours[i])
        {
            fs << iter;
        }
        fs << "]";
    }
    fs << "]";
    fs.release();
}

vector<set<Point, PointSort>> ChangeDS(vector<vector<Point>> &contours)
{
    vector<set<Point, PointSort>> bst_contours;
    for (auto i : contours)
    {
    	if (i.size() <= MINIMUM_CNTOUR_POINT)
    		continue;
        set<Point, PointSort> bst_1contour;
        for (auto j : i)
        {
            bst_1contour.insert(j);
        }
        if (bst_1contour.size() <= MINIMUM_CNTOUR_POINT)
        	continue;
        bst_contours.push_back(bst_1contour);
    }
    return bst_contours;
}


