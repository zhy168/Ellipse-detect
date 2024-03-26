#ifndef ELLIPSE_DETECTOR_H
#define ELLIPSE_DETECTOR_H
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <array>
#include <iostream>
#include <Eigen/Core>
using namespace cv;
using namespace std;
using namespace Eigen;

const double f = 82.41;
const double r = 9.1;
const uchar Minpoints=500;//椭圆点数下限
const double Minmatch=0.7;
const double fDistanceToEllipseContour=0.1f;
#define PI 3.14159

class Ellipse_Detector{

public:
    Ellipse_Detector();
    ~Ellipse_Detector();

    int ellipse_detect(Mat src,Mat& dst,vector<array<double, 5>>& parameters);

private:

    void RemoveSmallRegion(Mat& Src, Mat& Dst, int AreaLimit = 100, int CheckMode = 1, int NeihborMode = 0);
    void Cal_score(vector<vector<Point> > contours_final, vector<RotatedRect> box_final, int count, vector<vector<double>>& score);
};
#endif // ELLIPSE_DETECTOR_H
