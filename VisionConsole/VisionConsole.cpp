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

int main(int argc, _TCHAR* argv[])
{

	StereoVision sv;
	Mat leftColor, rightColor;
	Mat leftGrey, rightGrey;
	
	vector<Mat> left, right;

	VideoCapture cap0(0);
	VideoCapture cap1(1);

	while (true)
	{
		bool res = aiming(cap0, cap1);

		cap0 >> leftColor;
		cap1 >> rightColor;

		//Переводим в черно-белые
		cvtColor(leftColor, leftGrey, CV_RGB2GRAY);
		cvtColor(rightColor, rightGrey, CV_RGB2GRAY);

		left.push_back(leftGrey);
		right.push_back(rightGrey);

		waitKey(500);

		if (res)
		{
			//Калибровка
			StereoCalibData data = sv.Calibrate(left, right, Size(9, 9));
			sv.CalculatePointCloud(leftGrey, rightGrey);
			data.Save("calib.yml");

			left.clear();
			right.clear();
		}
	}

	return 0;
}


//Выводимт видео-поток с камеры
bool aiming(VideoCapture & cap1, VideoCapture & cap2)
{
	Mat img1, img2, img;
	int code;
	while (true)
	{
		cap1 >> img2;
		cap2 >> img1;

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