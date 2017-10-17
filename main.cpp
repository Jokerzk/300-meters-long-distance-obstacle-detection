#include "Matrix.h"
#include "Preprocess.h"
#include "Disparity.h"
#include "Obstacle_Detection.h"

using namespace std;
using namespace cv;

ImageInfo img_prev,img_curr[3];
bool state_prev = false,status = false;
int state[3] = { 0 };
float disparity_map[6][80];

void get_cameraInfo(int *img_size, float *intrinsic, float *distortion)
{
}

void getimage(unsigned char* img_buffer, float *img_Quaternion, float *img_Translation, ImageInfo image)
{
}
void update_image(ImageInfo image_prev, ImageInfo image_curr)
{
	image_prev = image_curr;
}

bool detector()
{
	//*****************************//
	//get current image;
	//judge that whether the previous image repository empty,if yes,store the latest one;
	//if no,work as the standard procedure;

	int img_size[2] = { 480, 640 };
	float intrinsic[4] = { 439.82920402256644, 440.36608099367356, 314.01121535161167, 244.7449018100555 };
	float distortion[4] = { 0.0027952110681112047, -0.03242641428960264, 0.0007953171465510378, 0.0007002791454423 };
	get_cameraInfo(img_size, intrinsic, distortion);

	//getimage(img1_buffer, img1_Quaternion, img1_Translation,);
	//getimage(img2_buffer, img2_Quaternion, img2_Translation);
	//getimage(img3_buffer, img3_Quaternion, img3_Translation);

	if (state_prev == false )
	{
		ImagePreprocessing_C(640, 480, img_curr[2].data, img_curr[2].quaternion, intrinsic, img_curr[2].mat);
		update_image(img_prev, img_curr[2]);
		state_prev = true;
	}
	else
	{
		for (int index = 2; index >=0; )
		{ 
			if (abs(img_prev.translation[2] - img_curr[index].translation[2]) > 0.2) //if the height difference is over 0.2m between two images,it is considered not under the certain assumption;
			{
				state[index] = 0;
				continue;
			}
			else
			{
				float dis_delta_ratio = sqrt(pow((img_prev.translation[0] - img_curr[index].translation[0]), 2) + pow((img_prev.translation[1] - img_curr[index].translation[1]), 2)) / 10;
				ImagePreprocessing_C(640, 480, img_curr[index].data, img_curr[index].quaternion, intrinsic, img_curr[index].mat);
				calc_disparity_map_C(img_prev.mat, img_curr[index].mat, disparity_map);
				state[index] = obstacle_detection_cal_C(dis_delta_ratio, disparity_map);

				if (index == 1 && state[0] + state[1] + state[2] == 0)
				{
					status = false;
					break;
				}
				else if (index == 1 || index == 0 && state[0] + state[1] + state[2] == 2)
				{
					status = true;
					break;
				}
				else
					continue;
			}
			index--;
		}
		update_image(img_prev, img_curr[2]);
	}
	return status;
}

int main()
{
	bool  State = true;
	if (State = true)
	{
		detector();
	}
	
	return 0;
}

