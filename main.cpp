#include "assert.h"
#include "long_distance_avoidance.h"

//#define PROFILING
//#define DISTORT
#define SECUREDIST 300

using namespace std;
using namespace cv;

//ned: timestamp:513929, lat_est:22.5977631, lon_est:113.9951248, ned_x_est:2098.0, ned_y_est:-1004.2, height_est:73.66, vel_ned_est[0]:-9.038,[1]:4.621,[2]:-0.034, valid_flag:0
//atti: timestamp:513949, UAV q0:0.2083 q1:0.0740 q2:0.0124 q3:0.9751, Gimbal roll:0.00 pitch:0.00 yaw:0.00 
//ned: timestamp:515409, lat_est : 22.5976391, lon_est : 113.9951859, ned_x_est : 1961.2, ned_y_est : -935.5, height_est : 73.65, vel_ned_est[0] : -9.201, [1] : 4.625, [2] : 0.334, valid_flag : 0
//atti: timestamp:515439, UAV q0 : 0.2036 q1 : 0.0874 q2 : 0.0214 q3 : 0.9748, Gimbal roll : 0.00 pitch : 0.00 yaw : 0.00
//ne: timestamp:515789, lat_est : 22.5976048, lon_est : 113.9952164, ned_x_est : 1925.4, ned_y_est : -917.8, height_est : 73.63, vel_ned_est[0] : -9.269, [1] : 4.489, [2] : 0.320, valid_flag : 0
//atti: timestamp:515809, UAV q0 : 0.2060 q1 : 0.0610 q2 : 0.0145 q3 : 0.9765, Gimbal roll : 0.00 pitch : 0.00 yaw : 0.00

//fx=439.82920402256644, fy = 440.36608099367356, cx = 314.01121535161167, cy = 244.7449018100555;

int getimage(string str_prev,string str_curr,cv::Mat &img_prev, cv::Mat &img_curr)
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
			frame.at<unsigned char>(i, j) = temp;
		}
	}
	frame.copyTo(img_curr);
	reader.close();
	return 0;
}

void obstacle_detection_cal(cv::Mat prev_mat, cv::Mat curr_mat, int width, int height, bool distorted, float Qarray1[4], float Tarray1[3], float Qarray2[4], float Tarray2[3], float intrinsic[4], float distortion[4])
{
	//int timestart = clock();

	Vec3f euAngle0 = GetEulerAngle(Qarray1);
	Vec3f euAngle2 = GetEulerAngle(Qarray2);

	float dis_delta = sqrt(pow((Tarray1[0] - Tarray2[0]), 2) + pow((Tarray1[1] - Tarray2[1]), 2));

	cv::Mat M = cv::Mat(3, 3, CV_32FC1, float(0));
	M.at<float>(0, 0) = intrinsic[0];
	M.at<float>(0, 2) = intrinsic[2];
	M.at<float>(1, 1) = intrinsic[1];
	M.at<float>(1, 2) = intrinsic[3];
	M.at<float>(2, 2) = 1;
	cv::Mat inv_M = M.inv();

	Vec3f cam_angle;
	cam_angle[0] = euAngle0[1];
	cam_angle[1] = 0 ;
	cam_angle[2] = euAngle0[0];
	cv::Mat rot_mat0 = Eular2Rot(cam_angle);
	cv::Mat inv_rot0 = rot_mat0.inv();

	cam_angle[0] = euAngle2[1];// +0.4 / 57.3;
	cam_angle[1] = 0;// -1.5 / 57.3;
	cam_angle[2] = euAngle2[0];
	cv::Mat rot_mat1 = Eular2Rot(cam_angle);
	cv::Mat inv_rot1 = rot_mat1.inv();

	//********** images undistortion **********//
	cv::Mat imgpre, imgcur;
	if (distorted == false)
	{
		cv::Mat distortion_mat = cv::Mat(1, 4, CV_32FC1, distortion);
		cv::undistort(prev_mat, imgpre, M, distortion_mat);
		cv::undistort(curr_mat, imgcur, M, distortion_mat);
	}
	else
	{
		imgpre = prev_mat;
		imgcur = curr_mat;
	}
	//********** image rotation according to its Euler angles **********//
	cv::Mat newimg0 = cv::Mat(120, 480, CV_8UC1, Scalar(0));
	cv::Mat newimg2 = cv::Mat(120, 480, CV_8UC1, Scalar(0));

	cv::Mat temp0 = M * rot_mat0 * inv_M;
	cv::Mat temp1 = M * rot_mat1 * inv_M;

	for (int i = 180; i < height - 180; ++i)
	{
		for (int j = 80; j < width - 80; ++j)
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
				newimg0.at<char>(i - 180, j - 80) = imgpre.at<char>(r0, c0);
			}

			cv::Mat new_temp1 = temp1 * temp;
			int r1 = new_temp1.at<float>(1, 0) / new_temp1.at<float>(2, 0);
			int c1 = new_temp1.at<float>(0, 0) / new_temp1.at<float>(2, 0);
			if (c1 >= 0 && c1 < width && r1 >= 0 && r1 < height)
			{
				newimg2.at<char>(i - 180, j - 80) = imgcur.at<char>(r1, c1);
			}
		}
	}
	/*cv::Mat process = cv::Mat(120, 480 * 2, CV_8UC1, Scalar(0));
	newimg0.copyTo(process(cv::Rect(0, 0, 480, 120)));
	newimg2.copyTo(process(cv::Rect(480, 0, 480, 120)));
	cv::Mat color_process;
	cv::cvtColor(process, color_process, CV_GRAY2BGR);
	cv::line(color_process, cv::Point2f(0, 60), cv::Point2f(960, 60), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(240, 0), cv::Point2f(240, 120), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(720, 0), cv::Point2f(720, 120), CV_RGB(255, 0, 0), 1);
	imshow("process", color_process);
	waitKey();*/

	//**********blur**********//
	cv::Mat blur_mat0, blur_mat2;
	cv::Size ksize(7,7);
	blur(newimg0, blur_mat0, ksize);
	blur(newimg2, blur_mat2, ksize);
	cv::Mat disparity2 = calc_disparity_map(blur_mat0, blur_mat2);
	float disp_left[11] = { 0 }, disp_right[11] = { 0 }, obsdist_left = 0, obsdist_right = 0;
	Mat sub_mat = Mat(1, disparity2.cols, CV_32FC1, Scalar(0));
	for (int i = 0; i < 22; i++)
	{
		for (int j = 0; j < disparity2.rows; j += 1)
		{
			for (int k = i * 10; k < (i+1) * 10; k++)
			{
				if (i <= 10)
				{
					disp_left[10 - i] += disparity2.at<char>(j, k);
				}
				else
					disp_right[i - 11] += disparity2.at<char>(j, k);
				
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
	obsdist_left = dis_delta*(1 - (disp_left[coutleft / 2 + ini_left] / 400 / (max((coutleft / 2 + ini_left), 1) * 10 + 5))) / (disp_left[coutleft / 2 + ini_left] / 400 / (max((coutleft / 2 + ini_left), 1) * 10 + 5));
	obsdist_right = dis_delta*(1 - (disp_right[coutright / 2 + ini_right] / 400 / (max((coutright / 2 + ini_right), 1) * 10 + 5))) / (disp_right[coutright / 2 + ini_right] / 400 / (max((coutright / 2 + ini_right), 1) * 10 + 5));
	
	if (obsdist_left < SECUREDIST || obsdist_right < SECUREDIST)
	{
		printf("Warning : Obstacles within 300 meters! \n");
	}
	else
	{
		printf("Security: There is no obstacle within 300 meters! \n");
	}
}
int main(int argc,char *argv[])
{
	//int times = clock();
	string str_prev, str_curr, str_data;
	int mode, imgwidth, imgheight;
	bool distorted;
	float Q_prev[4], T_prev[3], Q_curr[4], T_curr[3], intrinsic[4], distortion[4];

	cv::Mat img_prev, img_curr;
	if (argc <= 1)
	{
		printf("Please input a couple of images and corresponding data\n");
		cin >> str_prev;
		cin >> str_curr;
		cin >> str_data;
	}
	else if (argc == 4)
	{
		str_prev  = argv[1];
		str_curr  = argv[2];
		str_data  = argv[3];
	}
	else 
	{
		printf("Input incorrect!\n");
		return 0;
	}
	ifstream fid;
	fid.open(str_data);
	if (!fid.good())
	{
		printf("File read failed!\n");
		return 0;
	}
	else
	{
		char temp[1024];
		fid >> temp >> temp;
		fid >> mode;
		fid >> temp >> temp;
		fid >> distorted;
		fid >> temp >> temp;
		fid >> imgwidth >> imgheight;
		fid >> temp >> temp;
		fid >> Q_prev[0] >> Q_prev[1] >> Q_prev[2] >> Q_prev[3];
		fid >> temp >> temp;
		fid >> T_prev[0] >> T_prev[1] >> T_prev[2];
		fid >> temp >> temp;
		fid >> Q_curr[0] >> Q_curr[1] >> Q_curr[2] >> Q_curr[3];
		fid >> temp >> temp;
		fid >> T_curr[0] >> T_curr[1] >> T_curr[2];
		fid >> temp >> temp >> temp;
		fid >> intrinsic[0] >> intrinsic[1] >> intrinsic[2] >> intrinsic[3];
		fid >> temp >> temp;
		fid >> distortion[0] >> distortion[1] >> distortion[2] >> distortion[3];
	}
	switch (mode)
	{
	case 0:{
			   img_prev = imread(str_prev);
			   img_curr = imread(str_curr);}
	case 1:{
			   getimage(str_prev, str_curr, img_prev, img_curr);}
	default:
		break;
	}
	
	obstacle_detection_cal(img_prev, img_curr, imgwidth, imgheight, distorted, Q_prev, T_prev, Q_curr, T_curr, intrinsic, distortion);
	//int timee = clock();
	//printf("%d", timee - times);
	system("pause");
}

