#include "Obstacle_Detection.h"

float disp_10_300[10] = { 5, 8.33, 11.67, 15, 18.33, 21.67, 25, 28.33, 31.67, 35 };
float disp_20_300[10] = { 1, 1.67, 2.33, 3, 3.67, 4.33, 5, 5.67, 6.33, 7 };
float disp_30_300[10] = { 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5 };

int obstacle_detection_cal_C(float distance_delta_ratio, float disparitymap[6][80])
{
	float disp_left[6][4] = { 0 }, disp_right[6][4] = { 0 }, obsdist_left = 0, obsdist_right = 0;
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 6; j += 1)
		{
			for (int k = i * 10; k < (i + 1) * 10; k++)
			{
				if (i <= 3)
				{
					disp_left[j][3 - i] += disparitymap[j][k];
				}
				else
					disp_right[j][i - 4] += disparitymap[j][k];

			}
		}
	}
	int count_col = 0, count_row = 0;
	for (int i = 0; i < 6; i++)
	{
		count_col = 0;
		for (int j = 1; j < 4; j++)
		{
			if (disp_left[i][j] >= distance_delta_ratio*disp_10_300[j - 1])
				count_col++;
			if (disp_right[i][j] >= distance_delta_ratio*disp_10_300[j - 1])
				count_col++;
		}
		if (count_col >= SCORECOL)
			count_row++;
	}

	if (count_row >= SCOREROW)
		return 1;
	else
		return 0;

}

#ifdef OPENCV
int obstacle_detection_cal(float distance_delta_ratio, cv::Mat disparitymap)
{
	cv::Mat disparity = disparitymap(Rect(80,10,80,6));
	cv::Mat disp_mat;
	disparity.convertTo(disp_mat,CV_8UC1);
	cv::normalize(disp_mat, disp_mat, 0, 255, CV_MINMAX);
	imshow("disparity", disp_mat);
	waitKey(0);
	destroyWindow("disparity");
	float disp_left[6][4] = { 0 }, disp_right[6][4] = { 0 }, obsdist_left = 0, obsdist_right = 0;
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < disparity.rows; j += 1)
		{
			for (int k = i * 10; k < (i + 1) * 10; k++)
			{
				if (i <= 3)
				{
					disp_left[j][3 - i] += disparity.at<float>(j, k);
				}
				else
					disp_right[j][i - 4] += disparity.at<float>(j, k);

			}
		}
	}
	int count_col = 0, count_row = 0;
	for (int i = 0; i < 6; i++)
	{
		count_col = 0;
		for (int j = 1; j < 4; j++)
		{
			if (disp_left[i][j] >= distance_delta_ratio*disp_10_300[j - 1])
				count_col++;
			if (disp_right[i][j] >= distance_delta_ratio*disp_10_300[j - 1])
				count_col++;
		}
		if (count_col >= SCORECOL)
			count_row++;
	}

	if (count_row >= SCOREROW)
		return 1;
	else
		return 0;
}
#endif

