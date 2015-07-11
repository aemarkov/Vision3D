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


int main(int argc, _TCHAR* argv[])
{

	StereoVision sv(&cout);
	Mat leftColor, rightColor;
	Mat leftGrey, rightGrey;

	//Загружаем картинки
	leftColor = imread("images\\calib_left.JPG");
	rightColor = imread("images\\calib_right.JPG");

	//Переводим в черно-белые
	cvtColor(leftColor, leftGrey, CV_RGB2GRAY);
	cvtColor(rightColor, rightGrey, CV_RGB2GRAY);

	StereoCalibData data =  sv.Calibrate(leftGrey, rightGrey, Size(9,9));


	sv.CalculatePointCloud(leftColor, rightColor);

	return 0;
}

