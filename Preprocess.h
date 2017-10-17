#ifndef _PREPROCESS_H_
#define _PREPROCESS_H_

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

//int getimage(string str_prev, string str_curr, float img_prev[480][640], float img_curr[480][640]);

int getimage(string str_prev, vector <string> str_curr, cv::Mat &img_prev, vector<Mat> &img_curr);

void preprocessimg_mat(cv::Mat src_mat, cv::Mat &dst);

void ImagePreprocessing(Mat img_prev, Mat img_curr, int width, int height, bool distorted, vector<float> Qarray_prev, vector<float> Qarray_curr, float intrinsic[4], float distortion[4], Mat &mat_prev, Mat &mat_curr);
#endif