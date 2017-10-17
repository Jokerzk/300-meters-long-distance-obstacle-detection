#include "Preprocess.h"
#include "Matrix.h"
#ifdef Movidius
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
	//free(tmpdata);
}

//void resize_C(float src[80][240], float dest[YRESIZE][XRESIZE])
//{
//	float scale_x = float(XRESIZE) / 240;
//	float scale_y = float(YRESIZE) / 80;
//	for(int i = 0;i < XRESIZE;i++)
//	{
//		int sx = i*scale_x;
//		sx = min(sx,240 - 1);
//		for(int j = 0;j < YRESIZE;j++)
//		{
//			int sy = j*scale_y;
//			sy = min(sy,80 - 1);
//			dest[j][i] = src[sy][sx];
//		}
//	}
//}

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
void ImagePreprocessing_C(float prev_img[480][640], float curr_img[480][640], int width, int height, bool distorted, float Qarray1[4], float Qarray2[4], float intrinsic[4], float distortion[4], float prev_mat[26][240], float curr_mat[26][240])
{
	float euAngle0[3], euAngle2[3];
	GetEulerAngle_C(Qarray1, euAngle0);
	GetEulerAngle_C(Qarray2, euAngle2);

	float M[3][3] = { 0 }, M_inv[3][3] = { 0 };
	M[0][0] = intrinsic[0];
	M[0][2] = intrinsic[2];
	M[1][1] = intrinsic[1];
	M[1][2] = intrinsic[3];
	M[2][2] = 1;
	GetMatrixInverse(M, 3, M_inv);

	float cam_angle0[3], cam_angle2[3];
	float rot_mat0[3][3], rot_mat0_inv[3][3], rot_mat2[3][3], rot_mat2_inv[3][3];

	cam_angle0[0] = euAngle0[1];
	cam_angle0[1] = 0;
	cam_angle0[2] = euAngle0[0];
	Eular2Rot_C(cam_angle0, rot_mat0);
	GetMatrixInverse(rot_mat0, 3, rot_mat0_inv);

	cam_angle2[0] = euAngle2[1];
	cam_angle2[1] = 0;
	cam_angle2[2] = euAngle2[0];
	Eular2Rot_C(cam_angle2, rot_mat2);
	GetMatrixInverse(rot_mat2, 3, rot_mat2_inv);

	//********** images undistortion **********//

	/*if (distorted == false)
	{
	cv::Mat distortion_mat = cv::Mat(1, 4, CV_32FC1, distortion);
	cv::undistort(prev_mat, imgpre, M, distortion_mat);
	cv::undistort(curr_mat, imgcur, M, distortion_mat);
	}
	else
	{
	imgpre = prev_mat;
	imgcur = curr_mat;
	}*/
	//********** image rotation according to its Euler angles **********//
	float newimg0[80][240] = { 0 }, newimg2[80][240] = { 0 };
	float temp_0[3][3] = { 0 }, temp0[3][3] = { 0 }, temp_2[3][3] = { 0 }, temp2[3][3] = { 0 };
	GetMatrixMultiple_3(M, rot_mat0, temp_0);
	GetMatrixMultiple_3(temp_0, M_inv, temp0);

	GetMatrixMultiple_3(M, rot_mat2, temp_2);
	GetMatrixMultiple_3(temp_2, M_inv, temp2);
	int border_width = 200, border_height = 200;

	for (int i = border_height; i < height - border_height; ++i)
	{
		for (int j = border_width; j < width - border_width; ++j)
		{
			float temp[3];
			temp[0] = j;
			temp[1] = i;
			temp[2] = 1.0;

			float new_temp0[3] = { 0 };
			GetMatrixMultiple_1(temp0, temp, new_temp0);
			int r0 = new_temp0[1] / new_temp0[2];
			int c0 = new_temp0[0] / new_temp0[2];
			if (c0 >= 0 && c0 < width && r0 >= 0 && r0 < height)
			{
				newimg0[i - border_height][j - border_width] = prev_img[r0][c0];
			}

			float new_temp2[3] = { 0 };
			GetMatrixMultiple_1(temp2, temp, new_temp2);
			int r2 = new_temp2[1] / new_temp2[2];
			int c2 = new_temp2[0] / new_temp2[2];
			if (c2 >= 0 && c2 < 640 && r2 >= 0 && r2 < 480)
			{
				newimg2[i - border_height][j - border_width] = curr_img[r2][c2];
			}
		}
	}
	preprocessimg_mat_C(newimg0, prev_mat);
	preprocessimg_mat_C(newimg2, curr_mat);
}

void preprocessimg_mat_C(float src_mat[80][240], float dest_mat[26][240])
{
	float temp_blur[80][240] = { 0 }, temp_sobel[80][240] = { 0 }, temp_resize[26][240];
	Blur_C(src_mat, temp_blur, 240, 80, 3, 11);
	Sobel_C(temp_blur, temp_sobel, 3, 3);
	resize_C(temp_sobel, temp_resize);
	normalize_C(temp_resize, dest_mat, 240, 26, 1);
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

#endif

int getimage(string str_prev, vector <string> str_curr, cv::Mat &img_prev, vector<Mat> &img_curr)
{
	ifstream reader;

	reader.open(str_prev, ios_base::binary | ios_base::in);
	if (!reader.good())
	{
		return 0;
	}
	Mat frame = Mat(480, 640, CV_8UC1);
	for (int i = 0; i< 480; i++)
	{
		for (int j = 0; j < 640; ++j)
		{
			char temp;
			reader.read((char*)&temp, sizeof(char));
			frame.at<unsigned char>(i, j) = temp;
		}
	}
	frame.copyTo(img_prev);
	reader.close();
	for (int i = 0; i < str_curr.size(); i++)
	{
		reader.open(str_curr[i], ios_base::binary | ios_base::in);
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
				frame.at<unsigned char>(i, j) = temp;
			}
		}
		frame.copyTo(img_curr[i]);
		reader.close();
	}
	return 0;
}

void ImagePreprocessing(Mat img_prev, Mat img_curr, int width, int height, bool distorted, vector<float> Qarray_prev,  vector<float> Qarray_curr, float intrinsic[4], float distortion[4], Mat &mat_prev, Mat &mat_curr)
{
	Vec3f euAngle_prev = GetEulerAngle(Qarray_prev);
	Vec3f euAngle_curr = GetEulerAngle(Qarray_curr);
	cv::Mat M = cv::Mat(3, 3, CV_32FC1, float(0));
	M.at<float>(0, 0) = intrinsic[0];
	M.at<float>(0, 2) = intrinsic[2];
	M.at<float>(1, 1) = intrinsic[1];
	M.at<float>(1, 2) = intrinsic[3];
	M.at<float>(2, 2) = 1;
	cv::Mat inv_M = M.inv();
	Vec3f cam_angle;
	cam_angle[0] = euAngle_prev[1];
	cam_angle[1] = 0;
	cam_angle[2] = euAngle_prev[0];
	cv::Mat rot_mat0 = Eular2Rot(cam_angle);
	cv::Mat inv_rot0 = rot_mat0.inv();

	cam_angle[0] = euAngle_curr[1];// +0.4 / 57.3;
	cam_angle[1] = 0;// -1.5 / 57.3;
	cam_angle[2] = euAngle_curr[0];
	cv::Mat rot_mat1 = Eular2Rot(cam_angle);
	cv::Mat inv_rot1 = rot_mat1.inv();
	//********** images undistortion **********//
	cv::Mat imgprev, imgcurr;
	if (distorted == false)
	{
		cv::Mat distortion_mat = cv::Mat(1, 4, CV_32FC1, distortion);
		cv::undistort(img_prev, imgprev, M, distortion_mat);
		cv::undistort(img_curr, imgcurr, M, distortion_mat);
	}
	else
	{
		imgprev = img_prev;
		imgcurr = img_curr;
	}
	//********** image rotation according to its Euler angles **********//
	cv::Mat newimg0 = cv::Mat(80, 240, CV_8UC1, Scalar(0));
	cv::Mat newimg2 = cv::Mat(80, 240, CV_8UC1, Scalar(0));

	cv::Mat temp0 = M * rot_mat0 * inv_M;
	cv::Mat temp1 = M * rot_mat1 * inv_M;
	cv::Mat temptest = M * rot_mat1;
	int border_width = 200, border_height = 200;

	for (int i = border_height; i < height - border_height; ++i)
	{
		for (int j = border_width; j < width - border_width; ++j)
		{
			cv::Mat temp(3, 1, CV_32FC1);
			temp.at<float>(0, 0) = j;
			temp.at<float>(1, 0) = i;
			temp.at<float>(2, 0) = 1.0;

			cv::Mat new_temp0 = temp0 * temp;
			int r0 = new_temp0.at<float>(1, 0) / new_temp0.at<float>(2, 0);
			int c0 = new_temp0.at<float>(0, 0) / new_temp0.at<float>(2, 0);
			if (c0 >= 0 && c0 < width && r0 >= 0 && r0 < height)
			{
				newimg0.at<char>(i - border_height, j - border_width) = imgprev.at<char>(r0, c0);
			}

			cv::Mat new_temp1 = temp1 * temp;
			int r1 = new_temp1.at<float>(1, 0) / new_temp1.at<float>(2, 0);
			int c1 = new_temp1.at<float>(0, 0) / new_temp1.at<float>(2, 0);
			if (c1 >= 0 && c1 < width && r1 >= 0 && r1 < height)
			{
				newimg2.at<char>(i - border_height, j - border_width) = imgcurr.at<char>(r1, c1);
			}
		}
	}

	preprocessimg_mat(newimg0, mat_prev);
	preprocessimg_mat(newimg2, mat_curr);
	
}

void preprocessimg_mat(cv::Mat src_mat, cv::Mat &dst)
{
	/*cv::blur(src_mat, src_mat, cv::Size(3, 11));
	cv::Size small_sz;
	small_sz.height = src_mat.rows / 3;
	small_sz.width = src_mat.cols;
	cv::resize(src_mat, dst, small_sz);
	dst.convertTo(dst, CV_32FC1);
	cv::normalize(dst, dst, 1, 0, CV_MINMAX);*/

	Mat dst_x, dst_y; 
	blur(src_mat, src_mat, cv::Size(3, 11));
	/*cv::Mat src_temp;
	src_mat.convertTo(src_temp, CV_32FC1);
	float src_tempf[80][240], dest_tempf[80][240];
	for (int i = 0; i < 80; i++)
	{
		for (int j = 0; j < 240; j++)
		{
			src_tempf[i][j] = src_temp.at<float>(i, j);
		}
	}
	Sobel_C(src_tempf, dest_tempf, 3, 3);
	cout << "dest_tempf:" << "\n" << dest_tempf[20][120] << "\t" << dest_tempf[40][120] << endl;*/

	Sobel(src_mat, dst_x, CV_32FC1, 1, 0, 3);
	Sobel(src_mat, dst_y, CV_32FC1, 0, 1, 3);

	dst = Mat(src_mat.rows, src_mat.cols,CV_32FC1);
	for (int i = 0; i < src_mat.rows; ++i)
	{
		for (int j = 0; j < src_mat.cols; ++j)
		{
			dst.at<float>(i, j) = sqrt(dst_x.at<float>(i, j)*dst_x.at<float>(i, j) + dst_y.at<float>(i, j)*dst_y.at<float>(i, j));
		}
	}
	
	/*float dest_tempf[80][240], temp_resize[26][240];
	for (int i = 0; i < 80; i++)
	{
		for (int j = 0; j < 240; j++)
		{
			dest_tempf[i][j] = dst.at<float>(i, j);
		}
	}
	resize_C(dest_tempf, temp_resize);
	Mat dst_resize = Mat(26, 240, CV_32FC1, temp_resize);
	cout << "dst_resize:" << "\n" << dst_resize.at<float>(13, 140) << "\t" << dst_resize.at<float>(20, 140) << endl;*/

	cv::Size small_sz;
	small_sz.height = dst.rows / 3;
	small_sz.width = dst.cols;
	cv::resize(dst, dst, small_sz);
	//cout << "dst:" << "\n" << dst.at<float>(13, 140) << "\t" << dst.at<float>(20, 140) << endl;
	cv::normalize(dst, dst, 1, 0, NORM_MINMAX);

}
