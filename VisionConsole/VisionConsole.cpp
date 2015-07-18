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


//Паараметры для StereoSGBM
struct StereoSGBMParams
{
	int minDisparity = 0;
	int numDisparities = 0;
	int SADWindowSize = 0;
	int p1 = 0;
	int p2 = 0;
	int disp12MaxDiff = 0;
	int preFilterCap = 0;
	int uniquenessRatio = 0;
	int speckleWindowSize = 0;
	int speckleRange = 0;
} stereoSGBMParams;

//Параметры для SteroBM
struct StereoBMParams
{
	int blockSize =  0; 
	int numDisparities =  0;
	int preFilterSize =  0;
	int preFilterCap =  0;
	int minDisparity =  0;
	int textureThreshold =  0;
	int uniquenessRatio =  0;
	int speckleWindowSize =  0;
	int speckleRange =  0;
	int disp12MaxDiff =  0;
} stereoBMParams;

//Структура для передачи данных в обработчики событий
struct CallbackData
{
	StereoVision& sv;
	Mat& left;
	Mat& right;
	CallbackData(StereoVision& sv, Mat& left, Mat& right) :sv(sv), left(left), right(right){}
};


///////////////////////////////////////////////////////////////////////////////////////////////////

//Выводит видео-поток с камеры
bool aiming(VideoCapture & cap, VideoCapture & cap2);

void displayTrackbarsSGBM(CallbackData& data);
void displayTrackbarsBM(CallbackData& data);

//Событие изменения положения ползунка
void callbackSGBM(int wtf, void* data);
void callbackBM(int wtf, void* data);

//Масштабирует изображение и переводит его в ч\б
void convertImage(Mat& image, float scale);

//Выполняет калибровку
void calibrate(VideoCapture cap1, VideoCapture cap2, StereoVision& sv, Size patternSize);

void disparityRealtime(VideoCapture cap1, VideoCapture cap2, StereoVision& sv);



int main(int argc, _TCHAR* argv[])
{
	StereoVision sv("calib.yml");
	VideoCapture cap1(1), cap2(0);
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

		CallbackData data = CallbackData(sv, leftIm, rightIm);
		displayTrackbarsBM(data);
		callbackBM(0, &data);

		disparityRealtime(cap1, cap2, sv);
	}

	return 0;
}

//Выводимт видео-поток с камер
bool aiming(VideoCapture & cap1, VideoCapture & cap2)
{
	Mat img1, img2, img;
	Mat img1Flip, img2Flip;
	int code;
	while (true)
	{
		//Получаем изображения
		cap1 >> img1;
		cap2 >> img2;

		//Переворачиваем, потому что мои камеры дают изображения
		//горизонтально отраженное
		flip(img1, img1Flip, 1);
		flip(img2, img2Flip, 1);
		
		//Совмещаем два изображения рядом
		img = Mat(img1Flip.rows, img1Flip.cols + img2Flip.cols, CV_8UC3);
		Mat left(img, Rect(0, 0, img1Flip.cols, img1Flip.rows));
		Mat right(img, Rect(img1.cols, 0, img2.cols, img1.rows));
		img1Flip.copyTo(left);
		img2Flip.copyTo(right);

		imshow("Aiming", img);

		//Выход по нажатию кнопки
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
		
		//Добавляем в векторы
		left.push_back(leftIm.clone());
		right.push_back(rightIm.clone());

		waitKey(500);
	} while (!res);

	sv.Calibrate(left, right, patternSize);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////                    ОТОБРАЖЕНИЕ ПОЛЗУНКОВ                     ///////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void displayTrackbarsSGBM(CallbackData& data)
{
	namedWindow("trackbars");
	createTrackbar("Min Disparity", "trackbars", &stereoSGBMParams.minDisparity, 100, callbackSGBM, (void*)&data);
	createTrackbar("Num Disparties", "trackbars", &stereoSGBMParams.numDisparities, 100, callbackSGBM, (void*)&data);
	createTrackbar("SAD Window Size", "trackbars", &stereoSGBMParams.SADWindowSize, 20, callbackSGBM, (void*)&data);
	createTrackbar("P1", "trackbars", &stereoSGBMParams.p1, 3000, callbackSGBM, (void*)&data);
	createTrackbar("P2", "trackbars", &stereoSGBMParams.p2, 3000, callbackSGBM, (void*)&data);
	createTrackbar("Disp12MaxDiff", "trackbars", &stereoSGBMParams.disp12MaxDiff, 100, callbackSGBM, (void*)&data);
	createTrackbar("preFilterCap", "trackbars", &stereoSGBMParams.preFilterCap, 100, callbackSGBM, (void*)&data);
	createTrackbar("Uniqueness Ratio", "trackbars", &stereoSGBMParams.uniquenessRatio, 100, callbackSGBM, (void*)&data);
	createTrackbar("Speckle Win Size", "trackbars", &stereoSGBMParams.speckleWindowSize, 200, callbackSGBM, (void*)&data);
	createTrackbar("Speckle Range", "trackbars", &stereoSGBMParams.speckleRange, 10, callbackSGBM, (void*)&data);
}
void displayTrackbarsBM(CallbackData& data)
{
	namedWindow("trackbars");
	createTrackbar("Block size", "trackbars", &stereoBMParams.blockSize, 100, callbackBM, (void*)&data);
	createTrackbar("Num Disparties", "trackbars", &stereoBMParams.numDisparities, 100, callbackBM, (void*)&data);
	createTrackbar("Pre filter size", "trackbars", &stereoBMParams.preFilterSize, 100, callbackBM, (void*)&data);
	createTrackbar("Pre filter cap", "trackbars", &stereoBMParams.preFilterCap, 63, callbackBM, (void*)&data);
	createTrackbar("Min disparity", "trackbars", &stereoBMParams.minDisparity, 100, callbackBM, (void*)&data);
	createTrackbar("Texture threshold", "trackbars", &stereoBMParams.textureThreshold, 5000, callbackBM, (void*)&data);
	createTrackbar("Uniquess ratio", "trackbars", &stereoBMParams.uniquenessRatio, 100, callbackBM, (void*)&data);
	createTrackbar("Speckle win size", "trackbars", &stereoBMParams.speckleWindowSize, 200, callbackBM, (void*)&data);
	createTrackbar("Speckle range", "trackbars", &stereoBMParams.speckleRange, 100, callbackBM, (void*)&data);
	createTrackbar("Disp12MaxDiff", "trackbars", &stereoBMParams.disp12MaxDiff, 100, callbackBM, (void*)&data);
}

void callbackSGBM(int wtf, void* data)
{
	CallbackData cData = *(CallbackData*)data;

	//Ограничения на параметры
	if (stereoSGBMParams.numDisparities % 16 != 0)
		stereoSGBMParams.numDisparities -= stereoSGBMParams.numDisparities % 16;
	if (stereoSGBMParams.numDisparities < 16)stereoSGBMParams.numDisparities = 16;

	if (stereoSGBMParams.preFilterCap % 2 == 0)stereoSGBMParams.preFilterCap++;

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(stereoSGBMParams.minDisparity, stereoSGBMParams.numDisparities, stereoSGBMParams.SADWindowSize);
	sgbm->setPreFilterCap(stereoSGBMParams.preFilterCap);
	sgbm->setP1(stereoSGBMParams.p1);
	sgbm->setP2(stereoSGBMParams.p2);
	sgbm->setDisp12MaxDiff(stereoSGBMParams.disp12MaxDiff - 1);
	sgbm->setUniquenessRatio(stereoSGBMParams.uniquenessRatio);
	sgbm->setSpeckleWindowSize(stereoSGBMParams.speckleWindowSize);
	sgbm->setSpeckleRange(stereoSGBMParams.speckleRange);
	//sgbm->setMode(cv::StereoSGBM::MODE_SGBM); // : StereoSGBM::MODE_HH*/
	
	cData.sv.SetStereoMatcher(sgbm);
	cData.sv.CalculatePointCloud(cData.left, cData.right);
}

void callbackBM(int wtf, void* data)
{
	CallbackData cData = *(CallbackData*)data;

	//Ограничения на параметры
	if (stereoBMParams.numDisparities % 16 != 0)
		stereoBMParams.numDisparities -= stereoBMParams.numDisparities % 16;
	if (stereoBMParams.numDisparities < 16)stereoBMParams.numDisparities = 16;

	if (stereoBMParams.preFilterCap % 2 == 0)stereoBMParams.preFilterCap++;

	if (stereoBMParams.blockSize % 2 == 0)stereoBMParams.blockSize++;
	if (stereoBMParams.blockSize < 5)stereoBMParams.blockSize = 5;

	if (stereoBMParams.preFilterSize % 2 == 0)stereoBMParams.preFilterSize++;
	if (stereoBMParams.preFilterSize < 5)stereoBMParams.preFilterSize = 5;

	Ptr<StereoBM> sbm = cv::StereoBM::create(stereoBMParams.numDisparities, stereoBMParams.blockSize);
	sbm->setPreFilterSize(stereoBMParams.preFilterSize);				//Влиянеие не обнаружено
	sbm->setPreFilterCap(stereoBMParams.preFilterCap);					//Что - то нечетное, уменьшает шумы
	sbm->setMinDisparity(stereoBMParams.minDisparity);					//Меньше - разбитое фото, больше - темное (изменения сглаживаются)
	sbm->setTextureThreshold(stereoBMParams.textureThreshold);			//Порог текстуры (> = появляются черные пятна, < пятен нет, но детализация уменьшается)
	sbm->setUniquenessRatio(stereoBMParams.uniquenessRatio);			//Влияет на контуры (> четче)
	sbm->setSpeckleWindowSize(stereoBMParams.speckleWindowSize);		//Жесть какая - то
	sbm->setSpeckleRange(stereoBMParams.speckleRange);
	sbm->setDisp12MaxDiff(stereoBMParams.disp12MaxDiff);				//Сглаживает шумы

	cData.sv.SetStereoMatcher(sbm);
	cData.sv.CalculatePointCloud(cData.left, cData.right);
}

//Отображение карты в режиме реального времени
void disparityRealtime(VideoCapture cap1, VideoCapture cap2, StereoVision& sv)
{
	Mat left, right;

	while (true)
	{
		cap1 >> left;
		cap2 >> right;

		convertImage(left, 0.5);
		convertImage(right, 0.5);

		sv.CalculatePointCloud(left, right);

		if (waitKey(1) != -1)break;
	}
}