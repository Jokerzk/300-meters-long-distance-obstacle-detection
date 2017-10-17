#include "Preprocess.h"
#include "Matrix.h"

int getimage_C(string str_prev, string str_curr, float img_prev[480][640], float img_curr[480][640])
{
	ifstream reader;

	reader.open(str_prev, ios_base::binary | ios_base::in);
	if (!reader.good())
	{
		return 0;
	}
	for (int i = 0; i < 480; i++)
	{
		for (int j = 0; j < 640; ++j)
		{
			char temp;
			reader.read((char*)&temp, sizeof(char));
			img_prev[i][j] = float(temp);
		}
	}
	reader.open(str_curr, ios_base::binary | ios_base::in);
	if (!reader.good())
	{
		return 0;
	}
	for (int i = 0; i< 480; i++)
	{
		for (int j = 0; j < 640; ++j)
		{
			char temp;
			reader.read((char*)&temp, sizeof(char));
			img_curr[i][j] = float(temp);
		}
	}
	return 0;
}
void Sobel_C(float src[80][240], float dest[80][240],int Xksize, int Yksize)
{
	float Xsumwin,Ysumwin;
	int Xkneral[9] = { -1, 0, 1, -2, 0, 2, -1, 0, 1 };
	int Ykneral[9] = { -1, -2, -1, 0, 0, 0, 1, 2, 1 };
	for (int i = Yksize / 2; i < 80 - Yksize / 2; i++)
	{
		for (int j = Yksize / 2; j < 240 - Yksize; j++)
		{
			int index = 0;
			Xsumwin = 0.0; Ysumwin = 0.0;
			for (int m = i - 1; m < i + 2; m++)
			{
				for (int n = j - 1; n < j + 2; n++)
				{
					Xsumwin += src[m][n] * Xkneral[index];
					Ysumwin += src[m][n] * Ykneral[index];
					index++;

				}
			}
			dest[i][j] = sqrt(Xsumwin*Xsumwin + Ysumwin*Ysumwin);
		}
	}
}
void Blur_C(float src[80][240], float dest[80][240], int width, int height, int border_width, int border_height)
{
	float temp_sum;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width ; j++)
		{
			temp_sum = 0;
			if (i<border_height / 2 || j < border_width / 2 || i > height - border_height / 2 || j > width - border_width / 2)
				{
					dest[i][j] = src[i][j];
					continue;
				}
			else
				{
					for(int k = i - border_height / 2;k <= i + border_height / 2; k++ )
						for(int w = j - border_width / 2; w <= j + border_width / 2; w++)
							{
								temp_sum += src[k][w];
							}
					
				}
			dest[i][j] = temp_sum / border_width/border_height;
		}
	}
}

void resize_C(float src[80][240], float dest[YRESIZE][XRESIZE])
{
	float fx = (float)XRESIZE/ (float)240;
	float fy = (float)YRESIZE / (double)80;
	
	for (int i = 0; i < YRESIZE; i++)
	{
		float srcy = i / fy;
		int y = floor(srcy);
		float v = srcy - y;
		if (v < 0)
		{
			y = 0;
			v = 0;
		}
		if (y >= 80 - 1)
		{
			y = 80 - 2;
			v = 1;
		}
		float* srcData1 = src[y];
		float* srcData2 = src[y + 1];

		for (int j = 0; j < XRESIZE; j += 1)
		{
			float srcx = j / fx;
			int x = floor(srcx);
			float u = srcx - x;
			if (x < 0)
			{
				x = 0;
				u = 0;
			}
			if (x >= 240 - 1)
			{
				x = 240 - 2;
				u = 1;
			}
			dest[i][j] = (1 - u)*(1 - v)*srcData1[x] + (1 - u)*v*srcData2[x] + u*(1 - v)*srcData1[x + 1] + u*v*srcData2[x + 1];
		}
	}
}

void normalize_C(float input[YRESIZE][XRESIZE], float output[YRESIZE][XRESIZE], const int width, const int height, const int normrange)
{
	float maxval(0);
	float minval(1000);
	{
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				if (maxval < input[y][x]) maxval = input[y][x];
				if (minval > input[y][x]) minval = input[y][x];
			}
		}}
	double range = maxval - minval;
	if (0 == range) range = 1;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			output[y][x] = ((normrange*(input[y][x] - minval)) / range);
		}
	}
}

void preprocessimg_mat_C(float src_mat[80][240], float dest_mat[26][240])
{
	float temp_blur[80][240] = { 0 }, temp_sobel[80][240] = { 0 }, temp_resize[26][240];
	Blur_C(src_mat, temp_blur, 240, 80, 3, 11);
	Sobel_C(temp_blur, temp_sobel, 3, 3);
	resize_C(temp_sobel, dest_mat);
	normalize_C(dest_mat, dest_mat, 240, 26, 1);
}

void ImagePreprocessing_C(int width, int height, float img[480][640], float Qarray[4], float intrinsic[4], float mat[26][240])
{
	float euAngle[3];
	GetEulerAngle_C(Qarray, euAngle);

	float M[3][3] = { 0 }, M_inv[3][3] = { 0 };
	M[0][0] = intrinsic[0];
	M[0][2] = intrinsic[2];
	M[1][1] = intrinsic[1];
	M[1][2] = intrinsic[3];
	M[2][2] = 1;
	GetMatrixInverse(M, 3, M_inv);

	float cam_angle[3];
	float rot_mat[3][3], rot_mat_inv[3][3];

	cam_angle[0] = euAngle[1];
	cam_angle[1] = 0;
	cam_angle[2] = euAngle[0];
	Eular2Rot_C(cam_angle, rot_mat);
	GetMatrixInverse(rot_mat, 3, rot_mat_inv);

	//********** image rotation according to its Euler angles **********//
	float newimg[80][240] = { 0 };
	float temp_[3][3] = { 0 }, temp[3][3] = { 0 };
	GetMatrixMultiple_3(M, rot_mat, temp_);
	GetMatrixMultiple_3(temp_, M_inv, temp);

	int border_width = 200, border_height = 200;

	for (int i = border_height; i < height - border_height; ++i)
	{
		for (int j = border_width; j < width - border_width; ++j)
		{
			float tempk[3];
			tempk[0] = j;
			tempk[1] = i;
			tempk[2] = 1.0;

			float new_temp[3] = { 0 };
			GetMatrixMultiple_1(temp, tempk, new_temp);
			int r0 = new_temp[1] / new_temp[2];
			int c0 = new_temp[0] / new_temp[2];
			if (c0 >= 0 && c0 < width && r0 >= 0 && r0 < height)
			{
				newimg[i - border_height][j - border_width] = img[r0][c0];
			}
		}
	}
	preprocessimg_mat_C(newimg, mat);
}

