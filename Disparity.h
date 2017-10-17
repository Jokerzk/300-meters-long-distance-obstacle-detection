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

void calc_disparity_map_C(float prev_mat[26][240], float curr_mat[26][240], float disparity_map[6][80]);
#endif
