#ifndef _LONG_DISTANCE_AVOIDANCE_H_
#define _LONG_DISTANCE_AVOIDANCE_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <math.h>  
#include <opencv2/opencv.hpp>
#include <time.h>


using namespace std;
using namespace cv;

#define DOMAIN    32
#define WndWidth  21
#define CODEWIDTH 80

cv::Mat get_Rmatrix_from_Quaternion(vector<float> Qarray);
Vec3f GetEulerAngle(vector<float> Qarray);
cv::Mat Eular2Rot(Vec3f EulerAngle);
cv::Mat rot2euler(const cv::Mat & rotationMatrix);

cv::Mat calc_disparity_map(cv::Mat prev_mat, cv::Mat curr_mat);

#endif