/*///////////////////////////////////////////////////////////////////////////////////////
    Это консольное приложение, использующее VisionCore.                                //
    Оно используется для разработки и отладки системы компьютерного зрения.            //
*////////////////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"

#include <opencv2/core/mat.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

#include "StereoVision.h"
#include "StereoCalibData.h"
#include "StaticHelpers.h"

using namespace cv;
using namespace std;

//Выводит видео-поток с камеры
bool aiming(VideoCapture & cap, VideoCapture & cap2);

void displayMap(StereoVision& sv, Mat& leftGrey, Mat& rightGrey);
void callback(int wtf, void* data);
void getImagesCapture();
void getImagesFile();

struct CallbackData
{
	StereoVision* sv;
	Mat* left;
	Mat* right;
};

//Ptr<StereoSGBM> sgbm;
int minDisparity = 30;
int numDisparities = 80;
int SADWindowSize = 3;
int p1 = 1600;
int p2 = 2200;
int disp12MaxDiff = 10;
int preFilterCap = 10;
int uniquenessRatio = 10;
int speckleWindowSize = 100;
int speckleRange = 32;
int gaussRange = 1;


int main(int argc, _TCHAR* argv[])
{
	//getImagesFile();
	getImagesCapture();
	
	return 0;
}

void getImagesCapture()
{
	Mat leftColor, rightColor;
	Mat leftResized, rightResized;
	Mat leftGrey, rightGrey;

	StereoVision sv;
	vector<Mat> left, right;

	sv.out = &cout;

	//Получаем изображение с веб-камер
	VideoCapture cap0(1);
	VideoCapture cap1(2);

	while (true)
	{
		bool res = aiming(cap0, cap1);

		cap0 >> leftColor;
		cap1 >> rightColor;

		resize(leftColor, leftResized, Size(0, 0), 0.5, 0.5);
		resize(rightColor, rightResized, Size(0, 0), 0.5, 0.5);

		//Переводим в черно-белые
		cvtColor(leftResized, leftGrey, CV_RGB2GRAY);
		cvtColor(rightResized, rightGrey, CV_RGB2GRAY);

		left.push_back(leftGrey.clone());
		right.push_back(rightGrey.clone());

		waitKey(500);

		if (res)
		{
			//Калибровка
			StereoCalibData data = sv.Calibrate(left, right, Size(9, 6));

			displayMap(sv, leftGrey, rightGrey);

			data.Save("calib.yml");
			left.clear();
			right.clear();
		}
	}

}

void getImagesFile()
{
	Mat leftColor, rightColor;
	Mat leftGrey, rightGrey;

	StereoVision sv;
	vector<Mat> left, right;

	string f1, f2;
	cout << "Enter file names\n";
	cin >> f1;
	cin >> f2;

	leftColor = imread(f1.c_str());
	rightColor = imread(f2.c_str());

	//Переводим в черно-белые
	cvtColor(leftColor, leftGrey, CV_RGB2GRAY);
	cvtColor(rightColor, rightGrey, CV_RGB2GRAY);

	left.push_back(leftGrey);
	right.push_back(rightGrey);

	//Калибровка
	StereoCalibData data = sv.Calibrate(left, right, Size(7, 7));

	displayMap(sv, leftGrey, rightGrey);
}


//Выводимт видео-поток с камеры
bool aiming(VideoCapture & cap1, VideoCapture & cap2)
{
	Mat img1, img2, img;
	int code;
	while (true)
	{
		cap1 >> img1;
		cap2 >> img2;

		img = Mat(img1.rows, img1.cols + img2.cols, CV_8UC3);
		Mat left(img, Rect(0, 0, img1.cols, img1.rows));
		Mat right(img, Rect(img1.cols, 0, img2.cols, img1.rows));
		img1.copyTo(left);
		img2.copyTo(right);

		imshow("Aiming", img);
		code = waitKey(1);
		if (code != -1)
			break;
		
	}

	return code == 13;
}

void displayMap(StereoVision& sv, Mat& leftGrey, Mat& rightGrey)
{
	namedWindow("depth");
	namedWindow("trackbars", WINDOW_FREERATIO);
	

	CallbackData data;
	data.left = &leftGrey;
	data.right = &rightGrey;
	data.sv = &sv;

	createTrackbar("Min Disparity", "trackbars", &minDisparity, 100, callback, (void*)&data);
	createTrackbar("Num Disparties", "trackbars", &numDisparities, 100, callback, (void*)&data);
	//createTrackbar("SAD Window Size", "trackbars", &SADWindowSize, 20, callback, (void*)&data);
	createTrackbar("P1", "trackbars", &p1, 3000, callback, (void*)&data);
	createTrackbar("P2", "trackbars", &p2, 3000, callback, (void*)&data);
	createTrackbar("Disp12MaxDiff", "trackbars", &disp12MaxDiff, 100, callback, (void*)&data);
	createTrackbar("preFilterCap", "trackbars", &preFilterCap, 100, callback, (void*)&data);
	createTrackbar("Uniqueness Ratio", "trackbars", &uniquenessRatio, 100, callback, (void*)&data);
	createTrackbar("Speckle Win Size", "trackbars", &speckleWindowSize, 200, callback, (void*)&data);
	createTrackbar("Speckle Range", "trackbars", &speckleRange, 10, callback, (void*)&data);
	createTrackbar("Blur", "trackbars", &gaussRange, 100, callback, (void*)&data);
	

	callback(0, (void*)&data);

	waitKey(0);

	destroyWindow("depth");
	destroyWindow("normal_depth");
	destroyWindow("blur_depth");
	destroyWindow("w1");
	destroyWindow("w2");
	destroyWindow("trackbars");
}

void callback(int wtf, void* data)
{
	CallbackData* cData = (CallbackData*)data;
	Ptr<StereoSGBM> sgbm = cv::StereoSGBM::create(0, 80, 9);

	if (numDisparities % 16 != 0)
		numDisparities -= numDisparities % 16;

	if (gaussRange < 1)gaussRange = 1;
	if (gaussRange % 2 == 0)gaussRange += 1;

	sgbm->setMinDisparity(minDisparity - 50);
	sgbm->setNumDisparities(numDisparities);
	sgbm->setPreFilterCap(preFilterCap);
	sgbm->setP1(p1);
	sgbm->setP2(p2);
	sgbm->setDisp12MaxDiff(disp12MaxDiff-1);
	sgbm->setUniquenessRatio(uniquenessRatio);
	sgbm->setSpeckleWindowSize(speckleWindowSize);
	sgbm->setSpeckleRange(speckleRange);
	sgbm->setMode(cv::StereoSGBM::MODE_SGBM); // : StereoSGBM::MODE_HH

	cData->sv->CalculatePointCloud(*(cData->left), *(cData->right), sgbm, gaussRange);

}