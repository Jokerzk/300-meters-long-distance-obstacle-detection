#include "assert.h"
#include "long_distance_avoidance.h"

//#define PROFILING
//#define DISTORT
#define SECUREDIST 300
#define WINDOW_NAME "【图像对齐示例】"


using namespace std;
using namespace cv;

vector<string> res;
char* sequence_name;
Vec3f cam_angle;
const int g_nMaxAlphaValue = 100; //Alpha最大值
int PitchValueSlider = 50, RollValueSlider = 50, YawValueSlider = 50;  //滑动条对应的变量
double PitchValue, RollValue, YawValue;
Mat g_srcImage1;
Mat g_srcImage2;
Mat g_dstImage;

cv::Mat M, inv_M;
cv::Mat newimg0, newimg2, process, color_process;
cv::Mat temp0, temp1;
cv::Mat rot_mat1, inv_rot1;
cv::Mat imgpre, imgcur;
Vec3f euAngle0, euAngle2;
char euler_FilePath[256];
ofstream euler_file;

int disp_10_300[10] = { 0.5, 0.833, 1.167, 1.5, 1.833, 2.167, 2.5, 2.833, 3.167, 3.5 };
int disp_20_300[10] = { 1, 1.67, 2.33, 3, 3.67, 4.33, 5, 5.67, 6.33, 7 };
int disp_30_300[10] = { 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5 };

vector<string> listdir(const string &path)
{

	string dir = path;
	vector<string> s;
	_finddata_t fileDir;
	long lfDir;

	if ((lfDir = _findfirst(dir.c_str(), &fileDir)) == -1l)
		printf("No file is found\n");
	else{
		do{
			string str(fileDir.name);
			if (str.find('.') == -1)
				s.push_back(str);


		} while (_findnext(lfDir, &fileDir) == 0);
	}
	_findclose(lfDir);
	return s;

}

void findfile(const string &str)
{
	string s = str;
	vector<string> tmp = listdir(s + "\\*");
	for (int i = 0; i<tmp.size(); i++)
	{


		string temp = s + "\\" + tmp[i];
		res.push_back(temp);
		findfile(temp);
	}
}

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
void on_Tracker(int, void*)
{
	PitchValue = ((double)(PitchValueSlider - 50) / g_nMaxAlphaValue) / 5.73;
	RollValue = ((double)(RollValueSlider - 50) / g_nMaxAlphaValue) / 5.73;
	YawValue = ((double)(YawValueSlider - 50) / g_nMaxAlphaValue) / 5.73;

	cam_angle[0] = euAngle2[1] + RollValue;// +0.4 / 57.3;
	cam_angle[1] = 0 + PitchValue;// -1.5 / 57.3;
	cam_angle[2] = euAngle2[0] + YawValue;
	rot_mat1 = Eular2Rot(cam_angle);
	inv_rot1 = rot_mat1.inv();
	temp1 = M * rot_mat1 * inv_M;

	for (int i = 180; i < 480 - 180; ++i)
	{
		for (int j = 80; j < 640 - 80; ++j)
		{			cv::Mat temp(3, 1, CV_32FC1);
			temp.at<float>(0, 0) = j;
			temp.at<float>(1, 0) = i;
			temp.at<float>(2, 0) = 1.0;

			cv::Mat new_temp1 = temp1 * temp;
			int r1 = new_temp1.at<float>(1, 0) / new_temp1.at<float>(2, 0);
			int c1 = new_temp1.at<float>(0, 0) / new_temp1.at<float>(2, 0);
			if (c1 >= 0 && c1 < 640 && r1 >= 0 && r1 < 480)
			{
				newimg2.at<char>(i - 180, j - 80) = imgcur.at<char>(r1, c1);
			}
		}
	}
	process = cv::Mat(120, 480 * 2, CV_8UC1, Scalar(0));
	newimg0.copyTo(process(cv::Rect(0, 0, 480, 120)));
	newimg2.copyTo(process(cv::Rect(480, 0, 480, 120)));
	cv::cvtColor(process, color_process, CV_GRAY2BGR);
	cv::line(color_process, cv::Point2f(0, 60), cv::Point2f(960, 60), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(240, 0), cv::Point2f(240, 120), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(720, 0), cv::Point2f(720, 120), CV_RGB(255, 0, 0), 1);
	imshow(WINDOW_NAME, color_process);


}

//int obstacle_detection_cal(cv::Mat prev_mat, cv::Mat curr_mat, int width, int height, bool distorted, vector<float> Qarray1, vector<float> Tarray1, vector<float> Qarray2, vector<float> Tarray2, float intrinsic[4], float distortion[4])
//{
//	//int timestart = clock();
//
//	Vec3f euAngle0 = GetEulerAngle(Qarray1);
//	Vec3f euAngle2 = GetEulerAngle(Qarray2);
//
//	float dis_delta = sqrt(pow((Tarray1[0] - Tarray2[0]), 2) + pow((Tarray1[1] - Tarray2[1]), 2));
//
//	cv::Mat M = cv::Mat(3, 3, CV_32FC1, float(0));
//	M.at<float>(0, 0) = intrinsic[0];
//	M.at<float>(0, 2) = intrinsic[2];
//	M.at<float>(1, 1) = intrinsic[1];
//	M.at<float>(1, 2) = intrinsic[3];
//	M.at<float>(2, 2) = 1;
//	cv::Mat inv_M = M.inv();
//
//	Vec3f cam_angle;
//	cam_angle[0] = euAngle0[1];
//	cam_angle[1] = 0 ;
//	cam_angle[2] = euAngle0[0];
//	cv::Mat rot_mat0 = Eular2Rot(cam_angle);
//	cv::Mat inv_rot0 = rot_mat0.inv();
//
//	cam_angle[0] = euAngle2[1];// +0.4 / 57.3;
//	cam_angle[1] = 0;// -1.5 / 57.3;
//	cam_angle[2] = euAngle2[0];
//	cv::Mat rot_mat1 = Eular2Rot(cam_angle);
//	cv::Mat inv_rot1 = rot_mat1.inv();
//
//	//********** images undistortion **********//
//	cv::Mat imgpre, imgcur;
//	if (distorted == false)
//	{
//		cv::Mat distortion_mat = cv::Mat(1, 4, CV_32FC1, distortion);
//		cv::undistort(prev_mat, imgpre, M, distortion_mat);
//		cv::undistort(curr_mat, imgcur, M, distortion_mat);
//	}
//	else
//	{
//		imgpre = prev_mat;
//		imgcur = curr_mat;
//	}
//	//********** image rotation according to its Euler angles **********//
//	cv::Mat newimg0 = cv::Mat(120, 480, CV_8UC1, Scalar(0));
//	cv::Mat newimg2 = cv::Mat(120, 480, CV_8UC1, Scalar(0));
//
//	cv::Mat temp0 = M * rot_mat0 * inv_M;
//	cv::Mat temp1 = M * rot_mat1 * inv_M;
//
//	for (int i = 180; i < height - 180; ++i)
//	{
//		for (int j = 80; j < width - 80; ++j)
//		{
//			cv::Mat temp(3, 1, CV_32FC1);
//			temp.at<float>(0, 0) = j;
//			temp.at<float>(1, 0) = i;
//			temp.at<float>(2, 0) = 1.0;
//
//			cv::Mat new_temp0 = temp0 * temp;
//			int r0 = new_temp0.at<float>(1, 0) / new_temp0.at<float>(2, 0);
//			int c0 = new_temp0.at<float>(0, 0) / new_temp0.at<float>(2, 0);
//			if (c0 >= 0 && c0 < width && r0 >= 0 && r0 < height)
//			{
//				newimg0.at<char>(i - 180, j - 80) = imgpre.at<char>(r0, c0);
//			}
//
//			cv::Mat new_temp1 = temp1 * temp;
//			int r1 = new_temp1.at<float>(1, 0) / new_temp1.at<float>(2, 0);
//			int c1 = new_temp1.at<float>(0, 0) / new_temp1.at<float>(2, 0);
//			if (c1 >= 0 && c1 < width && r1 >= 0 && r1 < height)
//			{
//				newimg2.at<char>(i - 180, j - 80) = imgcur.at<char>(r1, c1);
//			}
//		}
//	}
//
//	cv::Mat process = cv::Mat(120, 480 * 2, CV_8UC1, Scalar(0));
//	newimg0.copyTo(process(cv::Rect(0, 0, 480, 120)));
//	newimg2.copyTo(process(cv::Rect(480, 0, 480, 120)));
//	cv::Mat color_process;
//	cv::cvtColor(process, color_process, CV_GRAY2BGR);
//	cv::line(color_process, cv::Point2f(0, 60), cv::Point2f(960, 60), CV_RGB(255, 0, 0), 1);
//	cv::line(color_process, cv::Point2f(240, 0), cv::Point2f(240, 120), CV_RGB(255, 0, 0), 1);
//	cv::line(color_process, cv::Point2f(720, 0), cv::Point2f(720, 120), CV_RGB(255, 0, 0), 1);
//	imshow("process", color_process);
//	waitKey();
//
//	//**********blur**********//
//	cv::Mat blur_mat0, blur_mat2;
//	cv::Size ksize(7,7);
//	blur(newimg0, blur_mat0, ksize);
//	blur(newimg2, blur_mat2, ksize);
//	cv::Mat disparity2 = calc_disparity_map(blur_mat0, blur_mat2);
//	float disp_left[11] = { 0 }, disp_right[11] = { 0 }, obsdist_left = 0, obsdist_right = 0;
//	Mat sub_mat = Mat(1, disparity2.cols, CV_32FC1, Scalar(0));
//	for (int i = 0; i < 22; i++)
//	{
//		for (int j = 0; j < disparity2.rows; j += 1)
//		{
//			for (int k = i * 10; k < (i+1) * 10; k++)
//			{
//				if (i <= 10)
//				{
//					disp_left[10 - i] += disparity2.at<char>(j, k);
//				}
//				else
//					disp_right[i - 11] += disparity2.at<char>(j, k);
//				
//			}
//		}
//	}
//	/*for (int i = 0; i < 11; i++)
//	{
//		printf("left_%f\t", disp_left[i]);
//		printf("right_%f\n", disp_right[i]);
//	}*/
//	int ini_left = 0, ini_right = 0, coutleft = 0, coutright = 0;
//	for (int i = 0; i < 11; i++)
//	{
//		if (disp_left[i] - disp_left[i + 1] == 0)
//			continue;
//		else if (disp_left[i] - disp_left[i + 1] < 0)
//		{
//			obsdist_left += disp_left[i];
//			coutleft++;
//			if (coutleft == 1)
//			{
//				ini_left = i;
//			}
//				
//		}
//		else
//			break;
//	}
//	for (int i = 0; i < 11; i++)
//	{
//		if (disp_right[i] - disp_right[i + 1] == 0)
//			continue;
//		else if (disp_right[i] - disp_right[i + 1] < 0)
//		{
//			obsdist_right += disp_right[i];
//			coutright++;
//			if (coutright == 1)
//			{
//				ini_right = i;
//			}
//				
//		}
//		else
//			break;
//	}
//	obsdist_left = dis_delta*(1 - (disp_left[coutleft / 2 + ini_left] / 400 / (max((coutleft / 2 + ini_left), 1) * 10 + 5))) / (disp_left[coutleft / 2 + ini_left] / 400 / (max((coutleft / 2 + ini_left), 1) * 10 + 5));
//	obsdist_right = dis_delta*(1 - (disp_right[coutright / 2 + ini_right] / 400 / (max((coutright / 2 + ini_right), 1) * 10 + 5))) / (disp_right[coutright / 2 + ini_right] / 400 / (max((coutright / 2 + ini_right), 1) * 10 + 5));
//	
//	if (obsdist_left < SECUREDIST || obsdist_right < SECUREDIST)
//	{
//		printf("Warning : Obstacles within 300 meters! \n");
//		return 1;
//	}
//	else
//	{
//		printf("Security: There is no obstacle within 300 meters! \n");
//		return 0;
//	}
//}
int obstacle_detection_cal(cv::Mat prev_mat, cv::Mat curr_mat, int width, int height, bool distorted, vector<float> Qarray1, vector<float> Tarray1, vector<float> Qarray2, vector<float> Tarray2, float intrinsic[4], float distortion[4])
{
	//int timestart = clock();

	euAngle0 = GetEulerAngle(Qarray1);
	euAngle2 = GetEulerAngle(Qarray2);
	euler_file << euAngle0[0] * 57.3 << "\t" << euAngle0[1] * 57.3 << "\t" << euAngle0[2] * 57.3 << "\t" << endl;
	cout << euAngle0[0] * 57.3 << "\t" << euAngle0[1] * 57.3 << "\t" << euAngle0[2] * 57.3 << "\t" << endl;
	cout << euAngle2[0] * 57.3 << "\t" << euAngle2[1] * 57.3 << "\t" << euAngle2[2] * 57.3 << "\t" << endl;

	float dis_delta = sqrt(pow((Tarray1[0] - Tarray2[0]), 2) + pow((Tarray1[1] - Tarray2[1]), 2));
	float dis_cur = sqrt(pow(Tarray2[0], 2) + pow(Tarray2[1], 2));
	cout << dis_delta << "\t" << dis_cur << endl;

	M = cv::Mat(3, 3, CV_32FC1, float(0));
	M.at<float>(0, 0) = intrinsic[0];
	M.at<float>(0, 2) = intrinsic[2];
	M.at<float>(1, 1) = intrinsic[1];
	M.at<float>(1, 2) = intrinsic[3];
	M.at<float>(2, 2) = 1;
	inv_M = M.inv();

	//Vec3f cam_angle;
	cam_angle[0] = euAngle0[1];
	cam_angle[1] = 0;
	cam_angle[2] = euAngle0[0];
	cv::Mat rot_mat0 = Eular2Rot(cam_angle);
	cv::Mat inv_rot0 = rot_mat0.inv();

	cam_angle[0] = euAngle2[1];// +0.4 / 57.3;
	cam_angle[1] = 0;// -1.5 / 57.3;
	cam_angle[2] = euAngle2[0];
	rot_mat1 = Eular2Rot(cam_angle);
	inv_rot1 = rot_mat1.inv();

	//********** images undistortion **********//

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
	newimg0 = cv::Mat(120, 480, CV_8UC1, Scalar(0));
	newimg2 = cv::Mat(120, 480, CV_8UC1, Scalar(0));

	temp0 = M * rot_mat0 * inv_M;
	temp1 = M * rot_mat1 * inv_M;

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
		}
	}
	//namedWindow(WINDOW_NAME);  //此处一定要先创建窗体，否则Trackbar无法显示
	//String TrackbarPitch("deltaPitch_nornalized");
	//String TrackbarRoll("deltaRoll_nornalized");
	//String TrackbarYaw("deltaYaw_nornalized");

	//createTrackbar(TrackbarPitch, WINDOW_NAME, &PitchValueSlider, g_nMaxAlphaValue, on_Tracker);
	//createTrackbar(TrackbarRoll, WINDOW_NAME, &RollValueSlider, g_nMaxAlphaValue, on_Tracker);
	//createTrackbar(TrackbarYaw, WINDOW_NAME, &YawValueSlider, g_nMaxAlphaValue, on_Tracker);
	//on_Tracker(0, 0);
	//waitKey();
	//destroyWindow(WINDOW_NAME);



	//**********blur**********//
	/*cv::imshow("newimg0", newimg0);
	cv::imshow("newimg2", newimg2);
	cv::Mat process = cv::Mat(120, 480 * 2, CV_8UC1, Scalar(0));
	newimg0.copyTo(process(cv::Rect(0, 0, 480, 120)));
	newimg2.copyTo(process(cv::Rect(480, 0, 480, 120)));
	cv::Mat color_process;
	cv::cvtColor(process, color_process, CV_GRAY2BGR);
	cv::line(color_process, cv::Point2f(0, 60), cv::Point2f(960, 60), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(240, 0), cv::Point2f(240, 120), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(720, 0), cv::Point2f(720, 120), CV_RGB(255, 0, 0), 1);
	imshow("process", color_process);
	waitKey();*/
	//return 0;

	cv::Mat blur_mat0, blur_mat2;
	cv::Size ksize(7, 7);
	blur(newimg0, blur_mat0, ksize);
	blur(newimg2, blur_mat2, ksize);
	cv::Mat disparity2 = calc_disparity_map(blur_mat0, blur_mat2);

	///*cv::imshow("disparity2", disparity2);
	//waitKey();*/
	float disp_left[11] = { 0 }, disp_right[11] = { 0 }, obsdist_left = 0, obsdist_right = 0;
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
		printf("Warning : Obstacles within %f meters! \n", obsdist_left);
		return 1;
	}
	else
	{
		printf("Security: There is no obstacle within 300 meters! \n");
		return 0;
	}
}
int main(int argc,char *argv[])
{
	//int times = clock();

	string str;
	
	vector < float> Q(4), T(3);
	int mode, imgwidth, imgheight;
	bool distorted;
	float intrinsic[4], distortion[4];
	
	if (argc <= 1)
	{
		printf("plz input sequences path\n");
		getline(cin,str);
		printf("%s\n", str);
	}
	else if (argc == 2)
	{
		str = argv[1];
	}
	else
	{
		printf("Too many arguments supplied.\n ");
		return 0;
	}
	findfile(str);
for (int i = 0; i < res.size(); i++)
	{
		string str_copy = res[i];
		char cfg_FilePath[256], img_FilePath[256], detect_FilePath[256], temp[1024];
		ifstream image_file, config_file;
		ofstream detect_file;
		vector <vector<float>> Quaternion, Translation;
		vector <string> str_image;
		sprintf(euler_FilePath, "%s/Euler.xls", str_copy.c_str());
		sprintf(cfg_FilePath, "%s/config.txt", str_copy.c_str());
		sprintf(img_FilePath, "%s/image_data.txt", str_copy.c_str());
		sprintf(detect_FilePath, "%s/detect_results.txt", str_copy.c_str());
		config_file.open(cfg_FilePath);
		image_file.open(img_FilePath);
		detect_file.open(detect_FilePath);
		euler_file.open(euler_FilePath);
		if (!config_file.good() || !image_file.good())
		{
			printf("File read failed!\n");
			return 0;
		}
		else
		{
			config_file >> temp >> temp;
			config_file >> mode;
			config_file >> temp >> temp;
			config_file >> distorted;
			config_file >> temp >> temp;
			config_file >> imgwidth >> imgheight;
			config_file >> temp >> temp >> temp;
			config_file >> intrinsic[0] >> intrinsic[1] >> intrinsic[2] >> intrinsic[3];
			config_file >> temp >> temp;
			config_file >> distortion[0] >> distortion[1] >> distortion[2] >> distortion[3];
			euler_file << "Roll\t" << "Pitch\t" << "Yaw\t" << endl;

			while (!image_file.eof())
			{
				/*image_file >> temp;
				str_image.push_back(temp);*/
				image_file >> Q[0] >> Q[1] >> Q[2] >> Q[3];
				Quaternion.push_back(Q);
				image_file >> T[0] >> T[1] >> T[2];
				Translation.push_back(T);
				image_file >> temp;
				str_image.push_back(temp);
			}
		}
		string str_prev, str_curr;
		cv::Mat img_prev, img_curr;
		vector<float> Q_prev(4), T_prev(3), Q_curr(4), T_curr(3);
		for (int i = 0; i < str_image.size() - 5; i +=5)
		{
			char pre_name[1024], cur_name[1024];
			str_prev = str_image[i];
			str_curr = str_image[i + 5];
			sprintf(pre_name, "%s/%s.raw8", str_copy.c_str(), str_prev.c_str());
			sprintf(cur_name, "%s/%s.raw8", str_copy.c_str(), str_curr.c_str());

			Q_prev = Quaternion[i];
			Q_curr = Quaternion[i+5];
			T_prev = Translation[i];
			T_curr = Translation[i + 5];
			
			switch (mode)
			{
			case 0:{
					   img_prev = imread(str_prev);
					   img_curr = imread(str_curr); }
			case 1:{
					   getimage(pre_name, cur_name, img_prev, img_curr); }
			default:
				break;
			}
			/*imshow(str_prev, img_prev);
			imshow(str_curr, img_curr);
			waitKey();*/
			int state = obstacle_detection_cal(img_prev, img_curr, imgwidth, imgheight, distorted, Q_prev, T_prev, Q_curr, T_curr, intrinsic, distortion);
			detect_file << str_curr << state << endl;
			destroyWindow(str_prev);
			destroyWindow(str_curr);

		}


		/*for (int i = 0; i < str_image.size(); i += 1)
		{

			char pre_name[1024], cur_name[1024];
			str_prev = str_image[i];
			str_curr = str_image[i];
			sprintf(pre_name, "%s/%s.raw8", str_copy.c_str(), str_prev.c_str());
			sprintf(cur_name, "%s/%s.raw8", str_copy.c_str(), str_curr.c_str());
			
			switch (mode)
			{
			case 0:{
					   img_prev = imread(pre_name);
					   img_curr = imread(cur_name); }
			case 1:{
					   getimage(pre_name, cur_name, img_prev, img_curr); }
			default:
				break;
			}
			imshow("img_prev", img_prev);
			imshow("img_curr", img_curr);
			waitKey(10);
			if (i == str_image.size()-1)
			{
				system("pause");
			}
		}*/
		
		
		

	}
	//int timee = clock();
	//printf("%d", timee - times);
	system("pause");
}

