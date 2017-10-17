#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
#ifdef C
void GetEulerAngle(float Qarray[4], float EulerAngle[3]);

void Eular2Rot(float EulerAngle[3], float RotMat[3][3]);

float GetA(float arcs[3][3], int n);
void  GetAStart(float arcs[3][3], int n, float ans[3][3]);
bool GetMatrixInverse(float src[3][3], int n, float des[3][3]);

void GetMatrixMultiple_3(float src1[3][3], float src2[3][3], float des[3][3]);

void GetMatrixMultiple_1(float src1[3][3], float src2[3], float des[3]);
#endif

Vec3f GetEulerAngle(vector<float> Qarray);
cv::Mat Eular2Rot(Vec3f EulerAngle);



#endif