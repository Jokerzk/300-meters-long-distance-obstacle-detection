#ifndef _DISPARITY_H_
#define _DISPARITY_H_

#define max(x,y)  ( x>y?x:y )
#define min(x,y)  ( x<y?x:y )
#define SAD_TH 10

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

cv::Mat calc_disparity_map(cv::Mat prev_mat, cv::Mat curr_mat);
#endif
