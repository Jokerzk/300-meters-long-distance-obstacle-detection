#include "Obstacle_Detection.h"

int disp_10_300[10] = { 0.5, 0.833, 1.167, 1.5, 1.833, 2.167, 2.5, 2.833, 3.167, 3.5 };
int disp_20_300[10] = { 1, 1.67, 2.33, 3, 3.67, 4.33, 5, 5.67, 6.33, 7 };
int disp_30_300[10] = { 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5 };

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

int obstacle_detection_cal(float distance_delta, cv::Mat disparitymap)
{
	float disp_left[11] = { 0 }, disp_right[11] = { 0 }, obsdist_left = 0, obsdist_right = 0;
	Mat sub_mat = Mat(1, disparitymap.cols, CV_32FC1, Scalar(0));
	for (int i = 0; i < 22; i++)
	{
		for (int j = 0; j < disparitymap.rows; j += 1)
		{
			for (int k = i * 10; k < (i + 1) * 10; k++)
			{
				if (i <= 10)
				{
					disp_left[10 - i] += disparitymap.at<char>(j, k);
				}
				else
					disp_right[i - 11] += disparitymap.at<char>(j, k);

			}
		}
	}
	/*for (int i = 0; i < 11; i++)
	{
	printf("left_%f\t", disp_left[i]);
	printf("right_%f\n", disp_right[i]);
	}*/
	int ini_left = 0, ini_right = 0, coutleft = 0, coutright = 0;
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
	obsdist_left = distance_delta*(1 - (disp_left[coutleft / 2 + ini_left] / 400 / (max((coutleft / 2 + ini_left), 1) * 10 + 5))) / (disp_left[coutleft / 2 + ini_left] / 400 / (max((coutleft / 2 + ini_left), 1) * 10 + 5));
	obsdist_right = distance_delta*(1 - (disp_right[coutright / 2 + ini_right] / 400 / (max((coutright / 2 + ini_right), 1) * 10 + 5))) / (disp_right[coutright / 2 + ini_right] / 400 / (max((coutright / 2 + ini_right), 1) * 10 + 5));

	if (obsdist_left < SECUREDIST || obsdist_right < SECUREDIST)
	{
		printf("Warning : Obstacles within 300 meters! \n");
		return 1;
	}
	else
	{
		printf("Security: There is no obstacle within 300 meters! \n");
		return 0;
	}
}

