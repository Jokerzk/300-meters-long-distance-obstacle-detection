#include "Disparity.h"
#ifdef C
void calc_disparity_map(float prev_mat[120][480], float curr_mat[120][480], float disparity_map[40][220])
{
	int height = 120;
	int width = 480;
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
							sad += abs(prev_mat[i + ii][j + jj] - curr_mat[i + ii][j - k + jj]);
						}
					}
					if (sad < min_sad)
					{
						min_sad = sad;
						//disp = k * 16;
						if (min_sad < 1500)
						{
							disp = min(k, 1) * 16 ;
						}
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
							sad += abs(prev_mat[i + ii][j + jj] - curr_mat[i + ii][j + k + jj]);
						}
					}
					if (sad < min_sad)
					{
						min_sad = sad;
						//disp = k * 16;
						if (min_sad < 1500)
						{
							disp = min(k, 1) * 16 ;
						}
					}
				}
			}
			disparity_map[i - 40][j - 130] = disp / 16;

		}
	}
}
#endif
cv::Mat calc_disparity_map(cv::Mat prev_mat, cv::Mat curr_mat)
{
	int height = prev_mat.rows;
	int width = prev_mat.cols;

	cv::Mat disparity_map = cv::Mat(height, width, CV_32FC1, Scalar(-1));

	int half_w = width / 2;
	int winsize = 21;
	int half_win = 10;

	for (int i = half_win; i < height - half_win; ++i)
	{
		for (int j = half_win; j < width - half_win; ++j)
		{
			int disp = -1;
			float min_sad = 9999;
			if (prev_mat.at<float>(i, j) >0.1)
			{
				float ratio = abs(j - half_w) / (float)half_w;
				int max_disparity = 100 * ratio;
				if (j<half_w)
				{
					int max_dis = min(j - half_win, max_disparity);
					for (int k = 0; k < max_dis; ++k)
					{
						float sad = 0;
						for (int ii = -half_win; ii <= half_win; ++ii)
						{
							for (int jj = -half_win; jj <= half_win; ++jj)
							{
								sad += (prev_mat.at<float>(i + ii, j + jj) - curr_mat.at<float>(i + ii, j - k + jj))*(prev_mat.at<float>(i + ii, j + jj) - curr_mat.at<float>(i + ii, j - k + jj));
							}
						}
						if (sad < min_sad)
						{
							min_sad = sad;
							if (min_sad < SAD_TH)
							{
								disp = max(k, 1);
							}
							else
							{
								disp = -1;
							}
						}
					}
				}
				else
				{
					int max_dis = min(width - half_win - j, max_disparity);
					for (int k = 0; k < max_dis; ++k)
					{
						float sad = 0;
						for (int ii = -half_win; ii <= half_win; ++ii)
						{
							for (int jj = -half_win; jj <= half_win; ++jj)
							{
								sad += (prev_mat.at<float>(i + ii, j + jj) - curr_mat.at<float>(i + ii, j + k + jj))*(prev_mat.at<float>(i + ii, j + jj) - curr_mat.at<float>(i + ii, j + k + jj));
							}
						}
						if (sad < min_sad)
						{
							min_sad = sad;
							if (min_sad < SAD_TH)
							{
								disp = max(k, 1);
							}
							else
							{
								disp = -1;
							}
						}
					}
				}
			}
			disparity_map.at<float>(i, j) = disp;
		}
	}
	cv::normalize(disparity_map, disparity_map, 255, 0, CV_MINMAX);
	disparity_map.convertTo(disparity_map, CV_8UC1);
	cv::medianBlur(disparity_map, disparity_map, 7);
	disparity_map.convertTo(disparity_map, CV_32FC1);
	cv::normalize(disparity_map, disparity_map, 1, 0, CV_MINMAX);
	cv::imshow("disparity", disparity_map);
	waitKey();
	return disparity_map;
}