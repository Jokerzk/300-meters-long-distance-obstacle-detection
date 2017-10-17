#include "Matrix.h"
#include "Preprocess.h"
#include "Disparity.h"
#include "Obstacle_Detection.h"


#define WINDOW_NAME "imagealine"
//#define CALIB
using namespace std;
using namespace cv;

vector<string> res;
char* sequence_name;

Vec3f cam_angle;
const int g_nMaxAlphaValue = 100; //Alpha最大值
int PitchValueSlider, RollValueSlider, YawValueSlider;  //滑动条对应的变量
double PitchValue, RollValue, YawValue;
Mat g_srcImage1;
Mat g_srcImage2;
Mat g_dstImage;
//cv::Mat process;
cv::Mat M, inv_M;
cv::Mat newimg0, newimg2, processmat,color_process;
cv::Mat temp0, temp1;
cv::Mat rot_mat1, inv_rot1;
cv::Mat imgpre, imgcur;
Vec3f euAngle0, euAngle2;

char calib_FilePath[1024];
ofstream calib_file;

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
		{
			cv::Mat temp(3, 1, CV_32FC1);
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
	processmat = cv::Mat(120, 480 * 2, CV_8UC1, Scalar(0));
	newimg0.copyTo(processmat(cv::Rect(0, 0, 480, 120)));
	newimg2.copyTo(processmat(cv::Rect(480, 0, 480, 120)));
	cv::cvtColor(processmat, color_process, CV_GRAY2BGR);
	cv::line(color_process, cv::Point2f(0, 60), cv::Point2f(960, 60), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(240, 0), cv::Point2f(240, 120), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(720, 0), cv::Point2f(720, 120), CV_RGB(255, 0, 0), 1);
	imshow(WINDOW_NAME, color_process);


}

int obstacle_detection_cal(Mat img_prev, Mat img_curr, int width, int height, bool distorted, vector<float> Qarray_prev, vector<float> Qarray_curr, float intrinsic[4], float distortion[4], cv::Mat &mat_prev, cv::Mat &mat_curr)
{
	euAngle0 = GetEulerAngle(Qarray_prev);
	euAngle2 = GetEulerAngle(Qarray_curr);

	//float dis_delta = sqrt(pow((Tarray1[0] - Tarray2[0]), 2) + pow((Tarray1[1] - Tarray2[1]), 2));

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
		cv::undistort(img_prev, imgpre, M, distortion_mat);
		cv::undistort(img_curr, imgcur, M, distortion_mat);
	}
	else
	{
		imgpre = img_prev;
		imgcur = img_curr;
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
	namedWindow(WINDOW_NAME);  //此处一定要先创建窗体，否则Trackbar无法显示
	String TrackbarPitch("deltaPitch_nornalized");
	String TrackbarRoll("deltaRoll_nornalized");
	String TrackbarYaw("deltaYaw_nornalized");

	PitchValueSlider = 50, RollValueSlider = 50, YawValueSlider = 50;
	createTrackbar(TrackbarPitch, WINDOW_NAME, &PitchValueSlider, g_nMaxAlphaValue, on_Tracker);
	createTrackbar(TrackbarRoll, WINDOW_NAME, &RollValueSlider, g_nMaxAlphaValue, on_Tracker);
	createTrackbar(TrackbarYaw, WINDOW_NAME, &YawValueSlider, g_nMaxAlphaValue, on_Tracker);
	on_Tracker(0, 0);
	waitKey();
	destroyWindow(WINDOW_NAME);
	calib_file << PitchValue*57.3 << "\t" << RollValue*57.3 << "\t" << YawValue*57.3 << "\t" << endl;

	cv::Mat mat_calib_prev, mat_calib_curr;

	mat_calib_prev = newimg0(Rect(120, 20, 240, 80));
	mat_calib_curr = newimg2(Rect(120, 20, 240, 80));
	preprocessimg_mat(mat_calib_prev, mat_prev);
	preprocessimg_mat(mat_calib_curr, mat_curr);
	//imshow("mat_prev", mat_prev);
	//imshow("mat_curr", mat_curr);
	//waitKey();
	//cv::Mat disparitymap;
	//disparitymap = calc_disparity_map(mat_prev, mat_curr);

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
	return 0;
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

int main(int argc, char *argv[])
{
	string str;

	vector < float> Q(4), T(3);
	int mode, imgwidth, imgheight;
	bool distorted;
	float intrinsic[4], distortion[4];

	if (argc <= 1)
	{
		printf("plz input sequences path\n");
		getline(cin, str);
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
		sprintf(cfg_FilePath, "%s/config.txt", str_copy.c_str());
		sprintf(img_FilePath, "%s/image_data.txt", str_copy.c_str());
		sprintf(detect_FilePath, "%s/detect_results.xls", str_copy.c_str());
		//sprintf(calib_FilePath, "%s/modify_metric.xls", str_copy.c_str());
		config_file.open(cfg_FilePath);
		image_file.open(img_FilePath);
		detect_file.open(detect_FilePath);
		//calib_file.open(calib_FilePath);
		detect_file << "detect_result" << "\t" << "image_number" << endl;
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

			while (!image_file.eof())
			{
				image_file >> Q[0] >> Q[1] >> Q[2] >> Q[3];
				Quaternion.push_back(Q);
				image_file >> T[0] >> T[1] >> T[2];
				//cout << T[0] << "\t" << T[1] << "\t" << T[2] << "\t" << endl;
				Translation.push_back(T);
				image_file >> temp;
				str_image.push_back(temp);
			}
		}
		string str_prev, str_curr_2, str_curr_1, str_curr;
		cv::Mat img_prev, img_curr_2, img_curr_1, img_curr;
		vector<float> Q_prev(4), T_prev(3), Q_curr_2(4), T_curr_2(3), Q_curr_1(4), T_curr_1(3), Q_curr(4), T_curr(3);
		vector <float> distance;
		vector <string> curr_filename;
		vector <Mat> curr_mat(3);
		bool isobstacles = false;
		for (int i = 0; i < str_image.size() - 8; i += 8)
		{
			char pre_name[1024], cur_2_name[1024], cur_1_name[1024], cur_name[1024], pre_out[1024], cur_out[1024];
			str_prev = str_image[i];
			str_curr_2 = str_image[i + 6];
			str_curr_1 = str_image[i + 7];
			str_curr = str_image[i + 8];
			sprintf(pre_name, "%s/%s.raw8", str_copy.c_str(), str_prev.c_str());
			sprintf(cur_2_name, "%s/%s.raw8", str_copy.c_str(), str_curr_2.c_str());
			sprintf(cur_1_name, "%s/%s.raw8", str_copy.c_str(), str_curr_1.c_str());
			sprintf(cur_name, "%s/%s.raw8", str_copy.c_str(), str_curr.c_str());
			curr_filename.push_back(cur_2_name);
			curr_filename.push_back(cur_1_name);
			curr_filename.push_back(cur_name);
			//sprintf(pre_out, "%s/jpg/%s.jpg", str_copy.c_str(), str_prev.c_str());
			//sprintf(cur_out, "%s/jpg/%s.jpg", str_copy.c_str(), str_curr.c_str());

			Q_prev = Quaternion[i];
			Q_curr_2 = Quaternion[i + 6];
			Q_curr_1 = Quaternion[i + 7];
			Q_curr = Quaternion[i + 8];

			float dis_delta_ratio_2 = sqrt(pow((Translation[i][0] - Translation[i + 6][0]), 2) + pow((Translation[i][1] - Translation[i + 6][1]), 2)) / 10;
			float dis_delta_ratio_1 = sqrt(pow((Translation[i][0] - Translation[i + 7][0]), 2) + pow((Translation[i][1] - Translation[i + 7][1]), 2)) / 10;
			float dis_delta_ratio = sqrt(pow((Translation[i][0] - Translation[i + 8][0]), 2) + pow((Translation[i][1] - Translation[i + 8][1]), 2)) / 10;
			//distance.push_back(dis_delta);

			switch (mode)
			{
			case 0:{
					   img_prev = imread(str_prev);
					   img_curr_2 = imread(str_curr_2);
					   img_curr_1 = imread(str_curr_1); 
					   img_curr = imread(str_curr); }
			case 1:{
					   getimage(pre_name, curr_filename, img_prev, curr_mat);
						img_curr_2 = curr_mat[0];
						img_curr_1 = curr_mat[1]; 
						img_curr = curr_mat[2]; }
			default:
				break;
			}
			curr_filename.resize(NULL);
			//curr_mat.resize(NULL);
			//imshow(str_prev, img_prev);
			//imshow(str_curr_2, img_curr);
			//imshow(str_curr_1, img_curr);
			//imshow(str_curr, img_curr);
			//waitKey();
			//destroyWindow(str_prev);
			//destroyWindow(str_curr_2);
			//destroyWindow(str_curr_1);
			//destroyWindow(str_curr);
			/*ORB_Algorithm(img_prev, img_curr);*/
			cv::Mat mat_prev, mat_curr;
			cv::Mat disparitymap_2, disparitymap_1, disparitymap;
			//********image preprocessing with calibration********//
#ifdef CALIB
			
			obstacle_detection_cal(img_prev, img_curr, imgwidth, imgheight, distorted, Q_prev, Q_curr, intrinsic, distortion, mat_prev, mat_curr);
			
#else
			ImagePreprocessing(img_prev, img_curr, imgwidth, imgheight, distorted, Q_prev, Q_curr, intrinsic, distortion, mat_prev, mat_curr);

#endif
			/*imshow("mat_prev", mat_prev);
			imshow("mat_curr", mat_curr);
			waitKey();
			destroyWindow("mat_prev");
			destroyWindow("mat_curr");*/
			disparitymap = calc_disparity_map(mat_prev,mat_curr);
			int state = obstacle_detection_cal(dis_delta_ratio, disparitymap);
#ifdef CALIB
			obstacle_detection_cal(img_prev, img_curr_2, imgwidth, imgheight, distorted, Q_prev, Q_curr_2, intrinsic, distortion, mat_prev, mat_curr);

#else
			ImagePreprocessing(img_prev, img_curr, imgwidth, imgheight, distorted, Q_prev, Q_curr, intrinsic, distortion, mat_prev, mat_curr);

#endif
			/*imshow("mat_prev", mat_prev);
			imshow("mat_curr", mat_curr);
			waitKey();
			destroyWindow("mat_prev");
			destroyWindow("mat_curr");*/
			disparitymap_2 = calc_disparity_map(mat_prev, mat_curr);
			int state_2 = obstacle_detection_cal(dis_delta_ratio_2, disparitymap);
			if (state + state_2 == 0)
			{
				detect_file << 0 << "\t" << str_curr << endl;
				continue;
			}
			else if (state + state_2 == 2)
			{
				isobstacles = true;
				detect_file << 1 << "\t" << str_curr << endl;
				continue;
			}
			else
			{
#ifdef CALIB
				obstacle_detection_cal(img_prev, img_curr_1, imgwidth, imgheight, distorted, Q_prev, Q_curr_1, intrinsic, distortion, mat_prev, mat_curr);

#else
				ImagePreprocessing(img_prev, img_curr, imgwidth, imgheight, distorted, Q_prev, Q_curr, intrinsic, distortion, mat_prev, mat_curr);

#endif
				/*imshow("mat_prev", mat_prev);
				imshow("mat_curr", mat_curr);
				waitKey();
				destroyWindow("mat_prev");
				destroyWindow("mat_curr");*/
				disparitymap_1 = calc_disparity_map(mat_prev, mat_curr);
				int state_1 = obstacle_detection_cal(dis_delta_ratio_1, disparitymap);
				if (state_1 == 1)
				{
					isobstacles = true;
					detect_file << 1 << "\t" << str_curr << endl;
				}
				else
				{
					detect_file << 0 << "\t" << str_curr << endl;
				}
			}
			
		}
		//system("pause");
	}
}

