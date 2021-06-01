#include <iostream>
#include <exception>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>

#include <stdio.h>
#include <iomanip>

using namespace std;
using namespace cv;


struct CalibData
{
    double u;
    double v;
    double x;
    double y;
    double z;
};

double lat0, lon0, height0;
cv::Mat candidate_matrix;
std::string RTK_file, cam_file, image_file, result_file, reproject_file;
std::vector<CalibData> points_calib;

bool ReadConfigfile(std::string config_fileName);

bool ReadCalibfile(std::string RTK_fileName, 
                   std::string cam_fileName, 
                   std::vector<CalibData>& points_calib);

std::vector<cv::Mat> GetUtm2Cam();

cv::Mat DrawUtm2Cam(std::vector<cv::Point2d> objectPoints,
                    std::vector<cv::Point2d> imagePoints);
