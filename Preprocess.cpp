#include "Preprocess.h"
#include "Matrix.h"

#ifdef C
int getimage(string str_prev, string str_curr, float img_prev[480][640], float img_curr[480][640])
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
void GaussianBlur(float* data, int width, int height)
{
	float sumwin;
	int kneral[9] = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };
	int sum = height * width * sizeof(float);
	float *tmpdata = (float*)malloc(sum);
	memcpy((float*)tmpdata, (float*)data, sum);
	for (int i = 1; i < height - 1; i++)
	{
		for (int j = 1; j < width - 1; j++)
		{
			int index = 0;
			float sumwin = 0.0;
			for (int m = i - 1; m < i + 2; m++)
			{
				for (int n = j - 1; n < j + 2; n++)
				{
					sumwin += tmpdata[m * width + n] * kneral[index++];
				}
			}
			data[i * width + j] = sumwin / 16;
		}
	}
	free(tmpdata);
}
void ImagePreprocessing(float prev_img[480][640], float curr_img[480][640], bool distorted, float Qarray1[4], float Tarray1[3], float Qarray2[4], float Tarray2[3], float intrinsic[4], float distortion[4])
{
	float euAngle0[3], euAngle2[3];
	GetEulerAngle(Qarray1, euAngle0);
	GetEulerAngle(Qarray2, euAngle2);

	float dis_delta = sqrt(pow((Tarray1[0] - Tarray2[0]), 2) + pow((Tarray1[1] - Tarray2[1]), 2));
	float dis_cur = sqrt(pow(Tarray2[0], 2) + pow(Tarray2[1], 2));

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
	Eular2Rot(cam_angle0, rot_mat0);
	GetMatrixInverse(rot_mat0, 3, rot_mat0_inv);

	cam_angle2[0] = euAngle2[1];
	cam_angle2[1] = 0;
	cam_angle2[2] = euAngle2[0];
	Eular2Rot(cam_angle2, rot_mat2);
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
	float newimg0[120][480] = { 0 }, newimg2[120][480] = { 0 };
	float temp[3][3] = { 0 }, temp0[3][3] = { 0 }, temp2[3][3] = { 0 };
	GetMatrixMultiple_3(M, rot_mat0, temp);
	GetMatrixMultiple_3(temp, M_inv, temp0);
	GetMatrixMultiple_3(M, rot_mat2, temp);
	GetMatrixMultiple_3(temp, M_inv, temp2);

	for (int i = 180; i < 640 - 180; ++i)
	{
		for (int j = 80; j < 480 - 80; ++j)
		{
			float temp[3];
			temp[0] = j;
			temp[1] = i;
			temp[2] = 1.0;

			float new_temp0[3] = { 0 };
			GetMatrixMultiple_1(temp0, temp, new_temp0);
			int r0 = new_temp0[1] / new_temp0[2];
			int c0 = new_temp0[0] / new_temp0[2];
			if (c0 >= 0 && c0 < 640 && r0 >= 0 && r0 < 480)
			{
				newimg0[i - 180][j - 80] = prev_img[r0][c0];
			}

			float new_temp2[3] = { 0 };
			GetMatrixMultiple_1(temp2, temp, new_temp2);
			int r0 = new_temp2[1] / new_temp2[2];
			int c0 = new_temp2[0] / new_temp2[2];
			if (c0 >= 0 && c0 < 640 && r0 >= 0 && r0 < 480)
			{
				newimg2[i - 180][j - 80] = curr_img[r0][c0];
			}
		}
	}

	GaussianBlur(newimg0, 480, 120);
	GaussianBlur(newimg2, 480, 120);

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

	/*cv::Mat process = cv::Mat(80, 480, CV_8UC1, Scalar(0));
	newimg0.copyTo(process(cv::Rect(0, 0, 240, 80)));
	newimg2.copyTo(process(cv::Rect(480, 0, 240, 80)));
	cv::Mat color_process;
	cv::cvtColor(process, color_process, CV_GRAY2BGR);
	cv::line(color_process, cv::Point2f(0, 40), cv::Point2f(480, 40), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(120, 0), cv::Point2f(120, 80), CV_RGB(255, 0, 0), 1);
	cv::line(color_process, cv::Point2f(360, 0), cv::Point2f(360, 80), CV_RGB(255, 0, 0), 1);
	imshow("process", color_process);
	waitKey();*/
	
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
	Sobel(src_mat, dst_x, CV_32FC1, 1, 0, 5);
	Sobel(src_mat, dst_y, CV_32FC1, 0, 1, 5);
	dst = Mat(src_mat.rows, src_mat.cols,CV_32FC1);
	for (int i = 0; i < src_mat.rows; ++i)
	{
		for (int j = 0; j < src_mat.cols; ++j)
		{
			dst.at<float>(i, j) = sqrt(dst_x.at<float>(i, j)*dst_x.at<float>(i, j) + dst_y.at<float>(i, j)*dst_y.at<float>(i, j));
		}
	}
	cv::Size small_sz;
	small_sz.height = dst.rows / 3;
	small_sz.width = dst.cols;
	cv::resize(dst, dst, small_sz);
	cv::normalize(dst, dst, 1, 0, CV_MINMAX);

}

//int ORB_Algorithm(cv::Mat imgA, cv::Mat imgB)
//{
//	imshow("imgA", imgA);
//	imshow("imgB", imgB);
//	waitKey();
//	ORB orb;
//	vector<KeyPoint> keyPointsA, keyPointsB;
//	Mat descriptorsA, descriptorsB;
//	orb(imgA, Mat(), keyPointsA, descriptorsA);
//	orb(imgB, Mat(), keyPointsB, descriptorsB);
//	drawKeypoints(imgA, keyPointsA, imgA, Scalar::all(-1));
//	drawKeypoints(imgB, keyPointsB, imgB, Scalar::all(-1));
//	cv::imshow("keypointsA", imgA);
//	cv::imshow("keypointsB", imgB);
//	waitKey();
//	//--Get discriptors of imgs
//	BruteForceMatcher<Hamming> matcher;
//	vector<DMatch> matches;
//
//	matcher.match(descriptorsA, descriptorsB, matches);
//	//--Return the minimum distances of pairs
//	
//	double max_dist = 0;
//	double min_dist = 100;
//	//-- Quick calculation of max and min distances between keypoints
//	for (int i = 0; i<descriptorsA.rows; i++)
//	{
//		double dist = matches[i].distance;
//		if (dist < min_dist) min_dist = dist;
//		if (dist > max_dist) max_dist = dist;
//	}
//	printf("-- Max dist : %f \n", max_dist);
//	printf("-- Min dist : %f \n", min_dist);
//	//-- Draw only "good" matches (i.e. whose distance is less than 0.6*max_dist )
//
//	vector<int> queryIdxs(matches.size()), trainIdxs(matches.size());
//	for (size_t i = 0; i < matches.size(); i++)
//	{
//		queryIdxs[i] = matches[i].queryIdx;
//		trainIdxs[i] = matches[i].trainIdx;
//	}
//
//	Mat H12, H13;   //单应矩阵  
//	vector<Point2f> points1; KeyPoint::convert(keyPointsA, points1, queryIdxs);
//	vector<Point2f> points2; KeyPoint::convert(keyPointsB, points2, trainIdxs);
//	int ransacReprojThreshold = 10;  //拒绝阈值  
//	H12 = findHomography(Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold);//根据匹配点获得单应矩阵
//	H13 = findFundamentalMat(Mat(points1), Mat(points2), FM_RANSAC);
//	cout << H13 << endl;
//	vector<DMatch> matchesMask;
//	Mat points1t;
//	perspectiveTransform(Mat(points1), points1t, H12);//利用单应矩阵透视变换得到图1中各点的MASK
//	for (size_t i = 0; i < points1.size(); i++)  //保存‘内点’  
//	{
//		if (norm(points2[i] - points1t.at<Point2f>((int)i, 0)) <= ransacReprojThreshold) //当图2中之前找的匹配点与MASK的灰度值小于给定阈值时，标记为内点
//		{
//			matchesMask.push_back(matches[i]);
//		}
//	}
//	Mat img_matches_ransac;
//	drawMatches(imgA, keyPointsA, imgB, keyPointsB,
//		matchesMask, img_matches_ransac, Scalar::all(-1), Scalar::all(-1),
//		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//	cv::imshow("ORB_match_ransac", img_matches_ransac);
//
//	vector<DMatch> good_matches;
//	for (int i = 0; i<descriptorsA.rows; i++)
//	{
//		if (matches[i].distance < 0.6*max_dist)
//		{
//			good_matches.push_back(matches[i]);
//		}
//	}
//
//	Mat img_matches;
//	drawMatches(imgA, keyPointsA, imgB, keyPointsB,
//		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//	cv::imshow("ORB_match", img_matches);
//	cv::waitKey();
//	return 0;
//}
