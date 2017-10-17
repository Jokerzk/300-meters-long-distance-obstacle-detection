#include "Obstacle_Detection.h"

float disp_10_300[10] = { 5, 8.33, 11.67, 15, 18.33, 21.67, 25, 28.33, 31.67, 35 };
float disp_20_300[10] = { 1, 1.67, 2.33, 3, 3.67, 4.33, 5, 5.67, 6.33, 7 };
float disp_30_300[10] = { 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5 };

#ifdef C
int obstacle_detection_cal(float distance_delta, float disparitymap[40][220])
{
	///*cv::imshow("disparity2", disparity2);
	//waitKey();*/
	/*float disp_left[11] = { 0 }, disp_right[11] = { 0 }, obsdist_left = 0, obsdist_right = 0;
	Mat sub_mat = Mat(1, disparity2.cols, CV_32FC1, Scalar(0));
	for (int i = 0; i < 22; i++)
	{
		for (int j = 0; j < disparity2.rows; j += 1)
		{
			for (int k = i * 10; k < (i + 1) * 10; k++)
			{
				if (i <= 10)
				{
					disp_left[10 - i] += disparity2.at<char>(j, k);
				}
				else
					disp_right[i - 11] += disparity2.at<char>(j, k);

			}
		}
	}*/
	/*for (int i = 0; i < 11; i++)
	{
	printf("left_%f\t", disp_left[i]);
	printf("right_%f\n", disp_right[i]);
	}*/
	/*int ini_left = 0, ini_right = 0, coutleft = 0, coutright = 0;
	for (int i = 0; i < 11; i++)
	{
		if (disp_left[i] - disp_left[i + 1] == 0)
			continue;
		else if (disp_left[i] - disp_left[i + 1] < 0)
		{
			obsdist_left += disp_left[i];
			coutleft++;
			if (coutleft == 1)
			{
				ini_left = i;
			}

		}
		else
			break;
	}
	for (int i = 0; i < 11; i++)
	{
		if (disp_right[i] - disp_right[i + 1] == 0)
			continue;
		else if (disp_right[i] - disp_right[i + 1] < 0)
		{
			obsdist_right += disp_right[i];
			coutright++;
			if (coutright == 1)
			{
				ini_right = i;
			}

		}
		else
			break;
	}
	obsdist_left = dis_delta*(1 - (disp_left[coutleft / 2 + ini_left] / 400 / (max((coutleft / 2 + ini_left), 1) * 10 + 5))) / (disp_left[coutleft / 2 + ini_left] / 400 / (max((coutleft / 2 + ini_left), 1) * 10 + 5));
	obsdist_right = dis_delta*(1 - (disp_right[coutright / 2 + ini_right] / 400 / (max((coutright / 2 + ini_right), 1) * 10 + 5))) / (disp_right[coutright / 2 + ini_right] / 400 / (max((coutright / 2 + ini_right), 1) * 10 + 5));

	if (obsdist_left < SECUREDIST || obsdist_right < SECUREDIST)
	{
		printf("Warning : Obstacles within %f meters! \n", obsdist_left);
		return 1;
	}
	else
	{
		printf("Security: There is no obstacle within 300 meters! \n");
		return 0;
	}*/
	return 0;
}

bool GetScore(float disp_left[10], float disp_right[10])
{
	int counts = 0;
	bool state = false;
	for (int i = 0; i < 10; i++)
	{
		if (disp_left[i] > disp_10_300[i])
			counts++;
		if (disp_right[i] > disp_10_300[i])
			counts++;
	}
	if (counts >= SCORE)
		state = true;
	return state;
}
#endif

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
				// << disparity.at<float>(j, k) << endl;
				if (i <= 3)
				{
					disp_left[j][3 - i] += disparity.at<float>(j, k);
				}
				else
					disp_right[j][i - 4] += disparity.at<float>(j, k);

			}
		}
	}
	/*for (int i = 0; i < 11; i++)
	{
	printf("left_%f\t", disp_left[i]);
	printf("right_%f\n", disp_right[i]);
	}*/
	int count_col = 0, count_row = 0;
	for (int i = 0; i < 6; i++)
	{
		count_col = 0;
		for (int j = 1; j < 4; j++)
		{
			if (disp_left[i][j] >= distance_delta_ratio*disp_10_300[j-1])
				count_col++;
			if (disp_right[i][j] >=distance_delta_ratio*disp_10_300[j-1])
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

