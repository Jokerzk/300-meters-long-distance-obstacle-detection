#include "Matrix.h"
#include "Preprocess.h"
#include "Disparity.h"
#include "Obstacle_Detection.h"

vector<string> res;
char* sequence_name;

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
		sprintf(detect_FilePath, "%s/detect_results.txt", str_copy.c_str());
		config_file.open(cfg_FilePath);
		image_file.open(img_FilePath);
		detect_file.open(detect_FilePath);
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
		string str_prev, str_curr;
		cv::Mat img_prev, img_curr;
		vector<float> Q_prev(4), T_prev(3), Q_curr(4), T_curr(3);
		vector <float> distance;
		for (int i = 0; i < str_image.size() - 1; i += 1)
		{
			char pre_name[1024], cur_name[1024], pre_out[1024], cur_out[1024];
			str_prev = str_image[i];
			str_curr = str_image[i + 1];
			sprintf(pre_name, "%s/%s.raw8", str_copy.c_str(), str_prev.c_str());
			sprintf(cur_name, "%s/%s.raw8", str_copy.c_str(), str_curr.c_str());
			sprintf(pre_out, "%s/jpg/%s.jpg", str_copy.c_str(), str_prev.c_str());
			sprintf(cur_out, "%s/jpg/%s.jpg", str_copy.c_str(), str_curr.c_str());

			Q_prev = Quaternion[i];
			Q_curr = Quaternion[i + 1];


			float dis_delta = sqrt(pow((Translation[i][0] - Translation[i + 1][0]), 2) + pow((Translation[i][1] - Translation[i + 1][1]), 2));
			distance.push_back(dis_delta);

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
			imwrite(pre_out, img_prev);
			imwrite(cur_out, img_curr);


			/*imshow(str_prev, img_prev);
			imshow(str_curr, img_curr);
			waitKey();
			destroyWindow(str_prev);
			destroyWindow(str_curr);
			ORB_Algorithm(img_prev, img_curr);*/

			//cv::Mat mat_prev, mat_curr;
			//ImagePreprocessing(img_prev, img_curr, imgwidth, imgheight, distorted, Q_prev, Q_curr, intrinsic, distortion, mat_prev, mat_curr);

			//imshow("mat_prev", mat_prev);
			//imshow("mat_curr", mat_curr);
			//waitKey();
			//cv::Mat disparitymap;
			//disparitymap = calc_disparity_map(mat_prev, mat_curr);
			//int state = obstacle_detection_cal(dis_delta, disparitymap);

		}
		for (int i = 0; i < distance.size(); i++)
		{
			cout << distance[i] << endl;
		}

	}
}

