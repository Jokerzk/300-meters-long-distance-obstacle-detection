#ifndef _OBSTACLE_DETECTION_H_
#define _OBSTACLE_DETECTION_H_

#include <opencv2\opencv.hpp>

#include "Matrix.h"

#define SCORECOL 4
#define SCOREROW 4
using namespace std;
using namespace cv;

int obstacle_detection_cal_C(float distance_delta_ratio, float disparitymap[6][80]);
#endif