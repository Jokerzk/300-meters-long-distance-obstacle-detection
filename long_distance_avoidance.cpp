#include "long_distance_avoidance.h"

cv::Mat get_Rmatrix_from_Quaternion(vector<float> Qarray)
{
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar(0));

	rotation_matrix.at<float>(0, 0) = 1 - 2 * (pow(Qarray[2], 2) + pow(Qarray[3], 2));
	rotation_matrix.at<float>(0, 1) = 2 * (Qarray[1] * Qarray[2] - Qarray[0] * Qarray[3]);
	rotation_matrix.at<float>(0, 2) = 2 * (Qarray[1] * Qarray[3] + Qarray[0] * Qarray[2]);
	rotation_matrix.at<float>(1, 1) = 1 - 2 * (pow(Qarray[1], 2) + pow(Qarray[3], 2));
	rotation_matrix.at<float>(1, 0) = 2 * (Qarray[1] * Qarray[2] + Qarray[0] * Qarray[3]);
	rotation_matrix.at<float>(1, 2) = 2 * (Qarray[2] * Qarray[3] - Qarray[0] * Qarray[1]);
	rotation_matrix.at<float>(2, 2) = 1 - 2 * (pow(Qarray[1], 2) + pow(Qarray[2], 2));
	rotation_matrix.at<float>(2, 0) = 2 * (Qarray[1] * Qarray[3] - Qarray[0] * Qarray[2]);
	rotation_matrix.at<float>(2, 1) = 2 * (Qarray[3] * Qarray[2] + Qarray[0] * Qarray[1]);

	return rotation_matrix;
}

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

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
	cv::Mat euler(1, 3, CV_32F);
	float m00 = rotationMatrix.at<float>(0, 0);
	float m02 = rotationMatrix.at<float>(0, 2);
	float m10 = rotationMatrix.at<float>(1, 0);
	float m11 = rotationMatrix.at<float>(1, 1);
	float m12 = rotationMatrix.at<float>(1, 2);
	float m20 = rotationMatrix.at<float>(2, 0);
	float m22 = rotationMatrix.at<float>(2, 2);

	float x, y, z;

	// Assuming the angles are in radians.
	if (m10 > 0.998) { // singularity at north pole
		x = 0;
		y = CV_PI / 2;
		z = atan2(m02, m22);
	}
	else if (m10 < -0.998) { // singularity at south pole
		x = 0;
		y = -CV_PI / 2;
		z = atan2(m02, m22);
	}
	else
	{
		x = atan2(-m12, m11);
		y = asin(m10);
		z = atan2(-m20, m00);
	}

	euler.at<float>(0) = x;
	euler.at<float>(1) = y;
	euler.at<float>(2) = z;

	return euler;
}

cv::Mat calc_disparity_map(cv::Mat prev_mat, cv::Mat curr_mat)
{
	cv::Mat disparity_map = cv::Mat(40, 220, CV_16SC1, Scalar(0));
	//cv::Mat distance_map = cv::Mat(40, 220, CV_32FC1, Scalar(0));

	int height = prev_mat.rows;
	int width = prev_mat.cols;
	int half_w = width / 2;


	int winsize = 21;
	int half_win = 10;

	for (int i = 40; i < 80; ++i)
	{
		for (int j = 130; j < 350; ++j)
		{
			int min_sad = 9999999;
			int disp = 0;
			if (j<half_w)
			{
				int max_dis = min(j - half_win, 32);
				for (int k = 0; k < max_dis; ++k)
				{
					int sad = 0;
					for (int ii = -half_win; ii <= half_win; ++ii)
					{
						for (int jj = -half_win; jj <= half_win; ++jj)
						{
							sad += abs(prev_mat.at<uchar>(i + ii, j + jj) - curr_mat.at<uchar>(i + ii, j - k + jj));
						}
					}
					if (sad < min_sad)
					{
						min_sad = sad;
						disp = k * 16;
						//if (min_sad < 1500)
						//{
						//	disp = min(k, 1) * 16 /*/ max(abs(j - half_w), 1)*/;
						//}
					}
				}
			}
			else
			{

				int max_dis = min(width - half_win - j, 32);
				for (int k = 0; k < max_dis; ++k)
				{
					int sad = 0;
					for (int ii = -half_win; ii <= half_win; ++ii)
					{
						for (int jj = -half_win; jj <= half_win; ++jj)
						{
							sad += abs(prev_mat.at<uchar>(i + ii, j + jj) - curr_mat.at<uchar>(i + ii, j + k + jj));
						}
					}
					if (sad < min_sad)
					{
						min_sad = sad;
						disp = k * 16;
						//if (min_sad < 1500)
						//{
						//	disp = min(k, 1) * 16 /*/ max(abs(j - half_w), 1)*/;
						//}
					}
				}
			}
			disparity_map.at<short>(i-40, j-130) = disp/16;
			//distance_map.at<float>(i-40, j-130) = disp / (float)max(abs(j - half_w), 1) / 16.0;
			/*if (i == height / 2 && j >= width / 2 - 100 && j <= width / 2 + 100)
			{
				printf("%f\t", disp / 16.0);
				printf("%f\n", disp / (float)max(abs(j - half_w), 1) / 16.0);
			}*/
			
		}
	}
	disparity_map.convertTo(disparity_map, CV_8UC1);
	return disparity_map;
}
