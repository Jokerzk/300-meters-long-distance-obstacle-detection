#include "Matrix.h"
#ifdef C
void GetEulerAngle(float Qarray[4], float EulerAngle[3])
{
	float q0 = Qarray[0];
	float q1 = Qarray[1];
	float q2 = Qarray[2];
	float q3 = Qarray[3];

	EulerAngle[0] = atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2));
	EulerAngle[1] = asin(2 * (q0*q2 - q3*q1));
	EulerAngle[2] = atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3));
}

void Eular2Rot(float EulerAngle[3], float RotMat[3][3])
{
	float froll = EulerAngle[0];
	float fpitch = EulerAngle[1];
	float fyaw = EulerAngle[2];

	RotMat[0][0] = cos(fpitch)*cos(fyaw);
	RotMat[0][1] = cos(fpitch)*sin(fyaw);
	RotMat[0][2] = -sin(fpitch);

	RotMat[1][0] = sin(froll)*sin(fpitch)*cos(fyaw) - cos(froll)*sin(fyaw);
	RotMat[1][1] = sin(froll)*sin(fpitch)*sin(fyaw) + cos(froll)*cos(fyaw);
	RotMat[1][2] = cos(fpitch)*sin(froll);

	RotMat[2][0] = cos(froll)*sin(fpitch)*cos(fyaw) + sin(froll)*sin(fyaw);
	RotMat[2][1] = cos(froll)*sin(fpitch)*sin(fyaw) - sin(froll)*cos(fyaw);
	RotMat[2][2] = cos(fpitch)*cos(froll);
}
float GetA(float arcs[3][3], int n)
{
	if (n == 1)
	{
		return arcs[0][0];
	}
	float ans = 0;
	float temp[3][3] = { 0.0 };
	int i, j, k;
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n - 1; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];

			}
		}
		float t = GetA(temp, n - 1);
		if (i % 2 == 0)
		{
			ans += arcs[0][i] * t;
		}
		else
		{
			ans -= arcs[0][i] * t;
		}
	}
	return ans;
}
void  GetAStart(float arcs[3][3], int n, float ans[3][3])
{
	if (n == 1)
	{
		ans[0][0] = 1;
		return;
	}
	int i, j, k, t;
	float temp[3][3];
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				for (t = 0; t<n - 1; t++)
				{
					temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
				}
			}
			ans[j][i] = GetA(temp, n - 1);
			if ((i + j) % 2 == 1)
			{
				ans[j][i] = -ans[j][i];
			}
		}
	}
}
bool GetMatrixInverse(float src[3][3], int n, float des[3][3])
{
	float flag = GetA(src, n);
	float t[3][3];
	if (flag == 0)
	{
		return false;
	}
	else
	{
		GetAStart(src, n, t);
		for (int i = 0; i<n; i++)
		{
			for (int j = 0; j<n; j++)
			{
				des[i][j] = t[i][j] / flag;
			}
		}
	}
	return true;
}
void GetMatrixMultiple_3(float src1[3][3], float src2[3][3], float des[3][3])
{
	for (int i = 0; i<3; i++)
		for (int k = 0; k<3; k++)
			for (int j = 0; j<3; j++)
				{
					des[i][k] += src1[k][j] * src2[j][k];
				}
}
void GetMatrixMultiple_1(float src1[3][3], float src2[3], float des[3])
{
	for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
				{
					des[i] += src1[i][j] * src2[j];
				}
}
#endif

Vec3f GetEulerAngle(vector<float> Qarray)
{
	Vec3f EulerAngle;

	float q0 = Qarray[0];
	float q1 = Qarray[1];
	float q2 = Qarray[2];
	float q3 = Qarray[3];

	EulerAngle[0] = atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2));
	EulerAngle[1] = asin(2 * (q0*q2 - q3*q1));
	EulerAngle[2] = atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3));

	return EulerAngle;
}

cv::Mat Eular2Rot(Vec3f EulerAngle)
{
	cv::Mat rotMat = cv::Mat(3, 3, CV_32FC1, Scalar(0));
	float froll = EulerAngle[0];
	float fpitch = EulerAngle[1];
	float fyaw = EulerAngle[2];
	rotMat.at<float>(0, 0) = cos(fpitch)*cos(fyaw);
	rotMat.at<float>(0, 1) = cos(fpitch)*sin(fyaw);
	rotMat.at<float>(0, 2) = -sin(fpitch);

	rotMat.at<float>(1, 0) = sin(froll)*sin(fpitch)*cos(fyaw) - cos(froll)*sin(fyaw);
	rotMat.at<float>(1, 1) = sin(froll)*sin(fpitch)*sin(fyaw) + cos(froll)*cos(fyaw);
	rotMat.at<float>(1, 2) = cos(fpitch)*sin(froll);

	rotMat.at<float>(2, 0) = cos(froll)*sin(fpitch)*cos(fyaw) + sin(froll)*sin(fyaw);
	rotMat.at<float>(2, 1) = cos(froll)*sin(fpitch)*sin(fyaw) - sin(froll)*cos(fyaw);
	rotMat.at<float>(2, 2) = cos(fpitch)*cos(froll);

	return rotMat;
}
