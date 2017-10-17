#ifndef _OBSTACLE_DETECTION_H_
#define _OBSTACLE_DETECTION_H_

#include <opencv2\opencv.hpp>

#include "Matrix.h"

#define SECUREDIST 300
#define SCORE 4
using namespace std;
using namespace cv;

int obstacle_detection_cal(float distance_delta, cv::Mat disparitymap);
#endif