#ifndef _PREPROCESS_H_
#define _PREPROCESS_H_

#define XRESIZE 240
#define YRESIZE 26

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include<io.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

struct ImageInfo
{
	float data[480][640];
	float quaternion[4];
	float translation[3];
	float mat[26][240];
};


int getimage_C(string str_prev, string str_curr, float img_prev[480][640], float img_curr[480][640]);

void Sobel_C(float src[80][240], float dest[80][240], int Xksize, int Yksize);

void Blur_C(float src[80][240], float dest[80][240], int width, int height, int border_width, int border_height);

void resize_C(float src[80][240], float dest[YRESIZE][XRESIZE]);

void normalize_normalize_C(float input[YRESIZE][XRESIZE], float output[YRESIZE][XRESIZE], const int width, const int height, const int normrange);

void preprocessimg_mat_C(float src_mat[80][240], float dest_mat[80][240]);

void ImagePreprocessing_C(int width, int height, float img[480][640], float Qarray[4], float intrinsic[4], float mat[26][240]);

#endif