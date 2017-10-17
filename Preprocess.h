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

int getimage_C(string str_prev, string str_curr, float img_prev[480][640], float img_curr[480][640]);

void Sobel_C(float src[80][240], float dest[80][240], int Xksize, int Yksize);

void Blur_C(float src[80][240], float dest[80][240], int width, int height, int border_width, int border_height);

void resize_C(float src[80][240], float dest[YRESIZE][XRESIZE]);

void ImagePreprocessing_C(float prev_img[480][640], float curr_img[480][640], int width, int height, bool distorted, float Qarray1[4], float Qarray2[4],  float intrinsic[4], float distortion[4]);

void preprocessimg_mat_C(cv::Mat src_mat, cv::Mat &dst);

void normalize(float input[YRESIZE][XRESIZE], float output[YRESIZE][XRESIZE], const int width, const int height, const int normrange);

int getimage(string str_prev, vector <string> str_curr, cv::Mat &img_prev, vector<Mat> &img_curr);

void preprocessimg_mat(cv::Mat src_mat, cv::Mat &dst);

void ImagePreprocessing(Mat img_prev, Mat img_curr, int width, int height, bool distorted, vector<float> Qarray_prev, vector<float> Qarray_curr, float intrinsic[4], float distortion[4], Mat &mat_prev, Mat &mat_curr);
#endif