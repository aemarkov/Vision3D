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

//Масштабирует изображение и переводит его в ч\б
void convertImage(Mat& image, float scale);

//Выполняет калибровку
void calibrate(VideoCapture cap1, VideoCapture cap2, StereoVision& sv, Size patternSize);

//Паараметры для SGBM
int minDisparity = 0;
int numDisparities = 80;
int SADWindowSize = 3;
int p1 = 1600;
int p2 = 2200;
int disp12MaxDiff = 10;
int preFilterCap = 10;
int uniquenessRatio = 10;
int speckleWindowSize = 100;
int speckleRange = 32;


//Структура для передачи данных в обработчики событий
struct CallbackData
{
	StereoVision& sv;
	Mat& left;
	Mat& right;
	
	CallbackData(StereoVision& sv, Mat& left, Mat& right) :sv(sv), left(left), right(right){}
};

int main(int argc, _TCHAR* argv[])
{
	StereoVision sv("calib_small_good.yml");
	VideoCapture cap1(1), cap2(2);
	Mat leftIm, rightIm;

	while (true)
	{
		//Калибровка
		//calibrate(cap1, cap2, sv, Size(9, 6));
		//sv.GetCalibData().Save("calib.yml");

		//Захват изображений
		aiming(cap1, cap2);
		cap1 >> leftIm;
		cap2 >> rightIm;

		//Уменьшение и перевод в ч\б
		convertImage(leftIm, 0.5);
		convertImage(rightIm, 0.5);

		displayMap(sv, leftIm, rightIm);
	}

	return 0;
}

//Выводимт видео-поток с камеры
bool aiming(VideoCapture & cap1, VideoCapture & cap2)
{
	Mat img1, img2, img;
	Mat img1Flip, img2Flip;
	int code;
	while (true)
	{
		cap1 >> img1;
		cap2 >> img2;

		flip(img1, img1Flip, 1);
		flip(img2, img2Flip, 1);

		img = Mat(img1Flip.rows, img1Flip.cols + img2Flip.cols, CV_8UC3);
		Mat left(img, Rect(0, 0, img1Flip.cols, img1Flip.rows));
		Mat right(img, Rect(img1.cols, 0, img2.cols, img1.rows));
		img1Flip.copyTo(left);
		img2Flip.copyTo(right);

		imshow("Aiming", img);
		code = waitKey(1);
		if (code != -1)
			break;
		
	}

	return code == 13;
}

//Масштабирует изображение и переводит его в ч\б
void convertImage(Mat& image, float scale)
{
	Mat resized, grey;
	resize(image, resized, Size(0, 0), scale, scale);
	cvtColor(resized, grey, CV_RGB2GRAY);
	image = grey.clone();
}


//Выполняет калибровку
void calibrate(VideoCapture cap1, VideoCapture cap2, StereoVision& sv, Size patternSize)
{
	Mat leftIm, rightIm;			//Изображения с веб-камер
	vector<Mat> left, right;		//Списки изображений
	bool res;

	do
	{
		//Показ видео
		res = aiming(cap1, cap2);

		//Захват изображений
		cap1 >> leftIm;
		cap2 >> rightIm;

		//Уменьшение и перевод в ч\б
		convertImage(leftIm, 0.5);
		convertImage(rightIm, 0.5);

		left.push_back(leftIm.clone());
		right.push_back(rightIm.clone());

		waitKey(500);
	} while (!res);

	sv.Calibrate(left, right, patternSize);
}

void displayMap(StereoVision& sv, Mat& leftGrey, Mat& rightGrey)
{
	namedWindow("depth");
	namedWindow("trackbars", WINDOW_FREERATIO);
	
	CallbackData data(sv, leftGrey, rightGrey);

	createTrackbar("Min Disparity", "trackbars", &minDisparity, 100, callback, (void*)&data);
	createTrackbar("Num Disparties", "trackbars", &numDisparities, 100, callback, (void*)&data);
	createTrackbar("SAD Window Size", "trackbars", &SADWindowSize, 20, callback, (void*)&data);
	createTrackbar("P1", "trackbars", &p1, 3000, callback, (void*)&data);
	createTrackbar("P2", "trackbars", &p2, 3000, callback, (void*)&data);
	createTrackbar("Disp12MaxDiff", "trackbars", &disp12MaxDiff, 100, callback, (void*)&data);
	createTrackbar("preFilterCap", "trackbars", &preFilterCap, 100, callback, (void*)&data);
	createTrackbar("Uniqueness Ratio", "trackbars", &uniquenessRatio, 100, callback, (void*)&data);
	createTrackbar("Speckle Win Size", "trackbars", &speckleWindowSize, 200, callback, (void*)&data);
	createTrackbar("Speckle Range", "trackbars", &speckleRange, 10, callback, (void*)&data);	

	callback(0, (void*)&data);

	waitKey(0);

	destroyWindow("normal_depth");
	destroyWindow("w1");
	destroyWindow("w2");
	destroyWindow("trackbars");
}

void callback(int wtf, void* data)
{
	CallbackData cData = *(CallbackData*)data;

	Ptr<StereoSGBM> sgbm = cv::StereoSGBM::create(0, 80, SADWindowSize);

	if (numDisparities % 16 != 0)
		numDisparities -= numDisparities % 16;
	if (numDisparities < 16)numDisparities = 16;

	sgbm->setMinDisparity(minDisparity);
	sgbm->setNumDisparities(numDisparities);
	sgbm->setPreFilterCap(preFilterCap);
	sgbm->setP1(p1);
	sgbm->setP2(p2);
	sgbm->setDisp12MaxDiff(disp12MaxDiff-1);
	sgbm->setUniquenessRatio(uniquenessRatio);
	sgbm->setSpeckleWindowSize(speckleWindowSize);
	sgbm->setSpeckleRange(speckleRange);
	sgbm->setMode(cv::StereoSGBM::MODE_SGBM); // : StereoSGBM::MODE_HH

	cData.sv.CalculatePointCloud(cData.left, cData.right, sgbm, 0);

}