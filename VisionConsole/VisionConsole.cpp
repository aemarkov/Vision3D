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

#include "GlutViewer.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace pcl;
using namespace std;

#define IMAGE_SCALE 0.5

// Settings for AdaptiveCostSOStereoMatching algorithm
struct ACSOSM_args
{
	// Default arguments
	ACSOSM_args()
		:radius(5),
		gamma_s(10),
		gamma_c(25),
		smoothness_weak(20),
		smoothness_strong(100)
	{}
	// radius (half length) of the column used for cost aggregation; 
	// the total column length is equal to 2*radius + 1
	int radius = 5;

	// spatial bandwith used for cost aggregation based on adaptive weights

	int gamma_s = 10;

	//  color bandwith used for cost aggregation based on adaptive weights
	int gamma_c = 25;

	// "weak" smoothness penalty used within 2-pass Scanline Optimization
	int smoothness_weak = 20;

	// "strong" smoothness penalty used within 2-pass Scanline Optimization
	int smoothness_strong = 100;
};

// Settings for BlockBasedStereoMatching algorithm
struct BBSM_args
{
	// Default arguments
	BBSM_args()
		:radius(5)
	{}
	// setter for the radius of the squared window
	// radius of the squared window used to compute the block-based stereo algorithm 
	// the window side is equal to 2*radius + 1
	int radius;
};

struct Common_args
{
	// Default arguments
	Common_args()
		:max_disparity(60),
		x_offset(0),
		ratio_filter(20),
		peak_filter(0),
		is_pre_processing(true),
		is_left_right_check(true),
		left_to_right_check_treshold(1),
		median_filter_radius(4)
	{}

	// number of disparity candidates (disparity range); 
	// || > 0
	int max_disparity = 60;

	// number of pixels to shift the disparity range over the target image
	// || >= 0
	int x_offset = 0; // horizontal offset

	// value of the ratio filter;
	// it is a number in the range[0, 100]
	// (0: no filtering action; 100: all disparities are filtered)
	// || [0; 100]
	int ratio_filter = 20;

	// value of the peak filter; it is a number in the range 
	// || [0, inf] (0: no filtering action)
	int peak_filter = 0;

	// setting the boolean to true activates the pre-processing step for both stereo images
	bool is_pre_processing = true;

	// setter for the left-right consistency check stage,
	// that eliminates inconsistent/wrong disparity values from the disparity map at approx.
	// setting the boolean to true activates the left-right consistency check
	bool is_left_right_check = true;

	// sets the value of the left-right consistency check threshold 
	// only has some influence if the left-right check is active typically has either the value 
	// 0 ("strong" consistency check, more points being filtered) 
	// 1 ("weak" consistency check, less points being filtered)
	int left_to_right_check_treshold = 1;


	// median filter applied on the previously computed disparity map
	// this radius of the squared window used to compute the median filter; 
	// the window side is equal to 2*radius + 1
	int median_filter_radius = 4;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

//Выводит видео-поток с камеры
bool aiming(VideoCapture & cap, VideoCapture & cap2);

//Масштабирует изображение и переводит его в ч\б
void convertImage(Mat& image, float scale);

//Выполняет калибровку
bool calibrate(VideoCapture cap1, VideoCapture cap2, StereoVision& sv, Size patternSize);

//Показать подсказки
void displayHelp();


void stereo_cycle(StereoVision& sv, cv::Mat left, cv::Mat right);
void ACSOSM_preset(pcl::AdaptiveCostSOStereoMatching & acsosm, Common_args & comm_a, ACSOSM_args & acsosm_a);
void BBSM_preset(pcl::BlockBasedStereoMatching & bbsm, Common_args & comm_a, BBSM_args & bbsm_a);
void print_ACSOSM_properties_list();
void print_BBSM_properties_list();
void print_Common_properties_list();

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, _TCHAR* argv[])
{
	setlocale(LC_ALL, "Russian");

	if (argc < 5)
	{
		//Показать хелп
		displayHelp();
		return 0;
	}

	//Параметры, переданные пользователем
	int leftCameraIndex = -1, rightCameraIndex = -1;		//Индексы левой и правой веб-камер
	bool useStereoSGBM = false;							//Использовать StereoBM или StereoSGBM
	string calibFilename, stereoFilename;				//Имена файлов настроек
	bool isCalib = false, isStereoConfig = false;		//Используются ли файлы конфигурации
	bool isRealtime = true;							//Показывать ли карту глубины в реальном времени

	//Обработка параметров
	int currentParam = 1;
	while (currentParam < argc)
	{
		if (strcmp("-left", argv[currentParam]) == 0)
		{
			leftCameraIndex = atoi(argv[currentParam + 1]);
			currentParam += 2;
		}
		else if (strcmp("-right", argv[currentParam]) == 0)
		{
			rightCameraIndex = atoi(argv[currentParam + 1]);
			currentParam += 2;
		}
		else if (strcmp("-sbm", argv[currentParam]) == 0)
		{
			useStereoSGBM = false;
			currentParam++;
		}
		else if (strcmp("-sgbm", argv[currentParam]) == 0)
		{
			useStereoSGBM = true;
			currentParam++;
		}
		else if (strcmp("-calib", argv[currentParam]) == 0)
		{
			calibFilename = string(argv[currentParam + 1]);
			currentParam += 2;
			isCalib = true;
		}
		else if (strcmp("-stereoparam", argv[currentParam]) == 0)
		{
			stereoFilename = string(argv[currentParam + 1]);
			currentParam += 2;
			isStereoConfig = true;
		}
		else
		{
			cout << argv[currentParam] << " - invalid argument";
			return 0;
		}

	}

	//Проверяем, что камеры указаны
	if ((leftCameraIndex == -1) || (rightCameraIndex == -1))
	{
		cout << "ERROR: camera not set\n";
		return 0;
	}

	StereoVision sv;												//Отвечат за калибровку и построение облака точек
	VideoCapture cap1(leftCameraIndex), cap2(rightCameraIndex);		//Веб-камеры
	Mat leftIm, rightIm;											//Изображения с камеры

	//Загружаем конфиг калибровки
	//!!
	if (isCalib)
	{
		sv = StereoVision(calibFilename.c_str());
	}
	else
	{
		//Калибровка
		int w, h;
		cout << "Enter pattern size width, height: ";
		cin >> w >> h;

		if (calibrate(cap1, cap2, sv, Size(w, h)))
		{
			cout << "Enter calibration config filename *.yml or *.xml: ";
			cin >> calibFilename;
			sv.GetCalibData().Save(calibFilename.c_str());
		}
		else
			return 0;

	}

	//Захват изображений
	cap1 >> leftIm;
	cap2 >> rightIm;
	convertImage(leftIm, 0.5);
	convertImage(rightIm, 0.5);

	stereo_cycle(sv, leftIm, rightIm);

	PointCloud<PointXYZRGB>::Ptr cloud = sv.CalculatePointCloud(leftIm, rightIm);


	

	return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////                   ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ                    ///////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void displayHelp()
{
	cout << "Vision3D console application\n";
	cout << "Usage:\n";
	cout << "VisionConsole -left <left camera index> -right <right camera index> [-sbm|-sgbm] [-calib <calibration file name>] [-stereoparam <stereo matching settings file>]\n";
	cout << " -left        - index of left webcamera\n";
	cout << " -right       - index of right webcamera\n";
	cout << " -sbm         - use StereoSB algorythm (default)\n";
	cout << " -sgbm        - use StereoSGBM algorythm\n";
	cout << " -calib       - filename with stereo calibration data\n";
	cout << " -stereoparam - filename with stereo matching algorytm params";
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

		cv::imshow("Aiming", img);

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
bool calibrate(VideoCapture cap1, VideoCapture cap2, StereoVision& sv, Size patternSize)
{
	bool isSuccess;					//Калибровка удачная
	bool isGood;					//Результаты калибровки хорошие
	string answer;					//Отвтет пользователя

	Mat leftIm, rightIm;			//Изображения с веб-камер
	vector<Mat> left, right;		//Списки изображений
	bool res;

	do
	{
		left.clear();
		right.clear();
		do
		{
			//Показ видео
			res = aiming(cap1, cap2);

			//Захват изображений
			cap1 >> leftIm;
			cap2 >> rightIm;

			//Уменьшение и перевод в ч\б
			convertImage(leftIm, IMAGE_SCALE);
			convertImage(rightIm, IMAGE_SCALE);

			//Добавляем в векторы
			left.push_back(leftIm.clone());
			right.push_back(rightIm.clone());

			waitKey(500);
		} while (!res);

		isSuccess = sv.Calibrate(left, right, patternSize);

		if (isSuccess)
		{
			//Показываем результат
			sv.Rectify(leftIm, rightIm);
			cv::imshow("left rectified", leftIm);
			cv::imshow("right rectified", rightIm);
			waitKey(0);
			destroyWindow("left rectified");
			destroyWindow("right rectified");
		}
		else
			cout << "Calibration failed\n";
		
		cout << "Save this result? ";
		cin >> answer;
		isGood = answer == "yes";

	} while (!isGood);

	return isSuccess;
}

//Отображение карты в режиме реального времени
bool disparityRealtime(VideoCapture cap1, VideoCapture cap2, StereoVision& sv)
{
	Mat left, right, disparity;
	int keyCode;

	while (true)
	{
		
		//Захват изображений
		cap1 >> left;
		cap2 >> right;

		//Перевод в черно-белое и масштабирование
		convertImage(left, IMAGE_SCALE);
		convertImage(right, IMAGE_SCALE);

		//Построение и показ карты различий
		sv.CalculatePointCloud(left, right, disparity, true);
		cv::imshow("disparity", disparity);

		keyCode = waitKey(1);
		if (keyCode != -1)break;
	}
	return keyCode == 27;
}


//////////////////////////////////////////////////////////////////////////

// VV==MAIN==VV
// Содержит цикл, в котором
// 1 - Изменяются параметры алгоритма
// 2 - Пересчитывается облако точек в соответствии с выбранным алгоритмом
void stereo_cycle(StereoVision& sv, cv::Mat left, cv::Mat right)
{
	// Algorithm objects
	pcl::AdaptiveCostSOStereoMatching acsosm;
	pcl::BlockBasedStereoMatching bbsm;

	// Algorithm settings
	Common_args comm_a;
	ACSOSM_args acsosm_a;
	BBSM_args bbsm_a;


	// Output point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Staff variables
	enum { END_CMD = 'z' };
	char command; // Continue? 
	int alg; // 1 - ACSOSM, 2 - BBSM

	// INPUT
	cout << "Выберите алгоритм (1 - для ACSOSM, 2 - для BBSM): ";
	cin >> alg;

	// CALCULATIONS
	do
	{
		switch (alg)
		{
		case 1:
			ACSOSM_preset(acsosm, comm_a, acsosm_a);

			//acsosm.compute(*leftCloud, *rightCloud);
			//acsosm.medianFilter(comm_a.median_filter_radius);
			//acsosm.getPointCloud(u0, v0, f, dist, outCloud, leftCloud);

			//Настраиваем объект StereoMatcher в sv
			sv.SetStereoMatcher(& acsosm);
			break;

		case 2:
			BBSM_preset(bbsm, comm_a, bbsm_a);

			//bbsm.compute(*leftCloud, *rightCloud);
			//bbsm.medianFilter(comm_a.median_filter_radius);
			//bbsm.getPointCloud(u0, v0, f, dist, outCloud, rightCloud);

			//Настраиваем объект StereoMatcher в sv
			sv.SetStereoMatcher(&bbsm);
			break;
		}

		//Получаем облако точек
		sv.medianFilterRadius = comm_a.median_filter_radius;
		outCloud = sv.CalculatePointCloud(left, right);

		//Визуализация
		pcl::visualization::CloudViewer viewer("Filtered");
		viewer.showCloud(outCloud);

		while (!viewer.wasStopped());

		printf("'z' = END || any key = CONTINUE:  ");
		scanf("%c", &command);
	} while (command != END_CMD);
}

// Функции настройки алгоритмов
// SETTINGS
// Обновить настройки алгоритма ACSOSM по введенным данным
// Позволяет выбрать редактируемое свойство алгоритма
// * acsosm[in/out]   : объект алгоритма
// * comm_a[in/out]   : общие настройки алгоритмов BBSM и ACSOSM
// * acsosm_a[in/out] : настройки ACSOSM алгоритма
void ACSOSM_preset(pcl::AdaptiveCostSOStereoMatching & acsosm, Common_args & comm_a, ACSOSM_args & acsosm_a)
{
	print_Common_properties_list();
	print_ACSOSM_properties_list();
	cout << "Введите номер свойства, которые вы хотите редактировать (-1 для завершения):";
	int cmd; // ^

	// exit from cycle when break in switch
	do
	{
		scanf("%d", &cmd);
		switch (cmd)
		{
		case 1:
			printf("1)MaxDisparity[int] (%d): ", comm_a.max_disparity);
			scanf("%d", &comm_a.max_disparity);
			//acsosm.setMaxDisparity(comm_a.max_disparity);
			break;

		case 2:
			printf("2)XOffset[int] (%d): ", comm_a.x_offset);
			scanf("%d", &comm_a.x_offset);
			//acsosm.setXOffset(comm_a.x_offset);
			break;

		case 3:
			printf("3)RatioFilter[int] (%d): ", comm_a.ratio_filter);
			scanf("%d", &comm_a.ratio_filter);
			//acsosm.setRatioFilter(comm_a.ratio_filter);
			break;

		case 4:
			printf("4)PeakFilter[int] (%d): ", comm_a.peak_filter);
			scanf("%d", &comm_a.peak_filter);
			//acsosm.setPeakFilter(comm_a.peak_filter);
			break;

		case 5:
			printf("5)PreProcessing[bool](0 = false, 1 = true) (%d): ", comm_a.is_pre_processing);
			scanf("%d", &comm_a.is_pre_processing);
			//acsosm.setPreProcessing(comm_a.is_pre_processing);
			break;

		case 6:
			printf("6)LeftRightCheck[bool](0 = false, 1 = true) (%d): ", comm_a.is_left_right_check);
			scanf("%d", &comm_a.is_left_right_check);
			//acsosm.setLeftRightCheck(comm_a.is_left_right_check);
			break;

		case 7:
			printf("7)LeftRightCheckTreshold[int] (%d): ", comm_a.left_to_right_check_treshold);
			scanf("%d", &comm_a.left_to_right_check_treshold);
			//acsosm.setLeftRightCheckThreshold(comm_a.left_to_right_check_treshold);
			break;

		case 8:
			printf("8)MedianFilterRadius[int] (%d): ", comm_a.median_filter_radius);
			scanf("%d", &comm_a.median_filter_radius);
			// Not YET!
			break;

		case 9:
			printf("9)Radius[int] (%d): ", acsosm_a.radius);
			scanf("%d", &acsosm_a.radius);
			//acsosm.setRadius(acsosm_a.radius);
			break;

		case 10:
			printf("10)GammaS[int] (%d): ", acsosm_a.gamma_s);
			scanf("%d", &acsosm_a.gamma_s);
			//acsosm.setGammaS(acsosm_a.gamma_s);
			break;

		case 11:
			printf("11)GammaC[int] (%d): ", acsosm_a.gamma_c);
			scanf("%d", &acsosm_a.gamma_c);
			//acsosm.setGammaC(acsosm_a.gamma_c);
			break;

		case 12:
			printf("12)SmoothnessWeak[int] (%d): ", acsosm_a.smoothness_weak);
			scanf("%d", &acsosm_a.smoothness_weak);
			//acsosm.setSmoothWeak(acsosm_a.smoothness_weak);
			break;

		case 13:
			printf("13)SmoothnessStrong[int] (%d): ", acsosm_a.smoothness_strong);
			scanf("%d", &acsosm_a.smoothness_strong);
			//acsosm.setSmoothStrong(acsosm_a.smoothness_strong);
			break;
		}
	} while (cmd != -1);

	acsosm.setMaxDisparity(comm_a.max_disparity);
	acsosm.setXOffset(comm_a.x_offset);
	acsosm.setRadius(acsosm_a.radius);
	acsosm.setSmoothWeak(acsosm_a.smoothness_weak);
	acsosm.setSmoothStrong(acsosm_a.smoothness_strong);
	acsosm.setGammaC(acsosm_a.gamma_c);
	acsosm.setGammaS(acsosm_a.gamma_s);
	acsosm.setRatioFilter(comm_a.ratio_filter);
	acsosm.setPeakFilter(comm_a.peak_filter);
	acsosm.setLeftRightCheck(comm_a.is_left_right_check);
	acsosm.setLeftRightCheckThreshold(comm_a.left_to_right_check_treshold);
	acsosm.setPreProcessing(comm_a.is_pre_processing);
}


// Обновить настройки алгоритма BBSM по введенным данным
// Позволяет выбрать редактируемое свойство алгоритма BBSM
// * bbsm[in/out]   : объект алгоритма
// * comm_a[in/out] : общие настройки алгоритмов BBSM и ACSOSM
// * bbsm_a[in/out] : настройки BBSM алгоритма
void BBSM_preset(pcl::BlockBasedStereoMatching & bbsm, Common_args & comm_a, BBSM_args & bbsm_a)
{
	print_Common_properties_list();
	print_BBSM_properties_list();

	cout << "Введите номер свойства, которые вы хотите редактировать (-1 для завершения):";
	int cmd; // ^

	// exit from cycle when break in switch
	while (true)
	{
		scanf("%d", &cmd);
		switch (cmd)
		{
		case 1:
			printf("1)MaxDisparity[int] (%d): ", comm_a.max_disparity);
			scanf("%d", &comm_a.max_disparity);
			break;

		case 2:
			printf("2)XOffset[int] (%d): ", comm_a.x_offset);
			scanf("%d", &comm_a.x_offset);
			
			break;

		case 3:
			printf("3)RatioFilter[int] (%d): ", comm_a.ratio_filter);
			scanf("%d", &comm_a.ratio_filter);
			
			break;

		case 4:
			printf("4)PeakFilter[int] (%d): ", comm_a.peak_filter);
			scanf("%d", &comm_a.peak_filter);
			
			break;

		case 5:
			printf("5)PreProcessing[bool](0 = false, 1 = true) (%d): ", comm_a.is_pre_processing);
			scanf("%d", &comm_a.is_pre_processing);
			
			break;

		case 6:
			printf("6)LeftRightCheck[bool](0 = false, 1 = true) (%d): ", comm_a.is_left_right_check);
			scanf("%d", &comm_a.is_left_right_check);
			
			break;

		case 7:
			printf("7)LeftRightCheckTreshold[int] (%d): ", comm_a.left_to_right_check_treshold);
			scanf("%d", &comm_a.left_to_right_check_treshold);
			
			break;

		case 8:
			printf("8)MedianFilterRadius[int] (%d): ", comm_a.median_filter_radius);
			scanf("%d", &comm_a.median_filter_radius);
			// Not YET!
			break;

		case 9:
			printf("9)Radius[int] (%d): ", bbsm_a.radius);
			scanf("%d", &bbsm_a.radius);
		
			break;

		default:
			break; // exit_cycle
		}
	}

	bbsm.setMaxDisparity(comm_a.max_disparity);
	bbsm.setXOffset(comm_a.x_offset);
	bbsm.setRatioFilter(comm_a.ratio_filter);
	bbsm.setPeakFilter(comm_a.peak_filter);
	bbsm.setPreProcessing(comm_a.is_pre_processing);
	bbsm.setLeftRightCheck(comm_a.is_left_right_check);
	bbsm.setLeftRightCheckThreshold(comm_a.left_to_right_check_treshold);
	bbsm.setRadius(bbsm_a.radius);
}


// Функции печати, выводящие список свойств конкретного алгоритма на экран
// PRINT
void print_Common_properties_list()
{
	printf("%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n",
		"1)MaxDisparity[int]: ",
		"2)XOffset[int]: ",
		"3)RatioFilter[int]: ",
		"4)PeakFilter[int]: ",
		"5)PreProcessing[bool](0 = false, 1 = true): ",
		"6)LeftRightCheck[bool](0 = false, 1 = true): ",
		"7)LeftRightCheckTreshold[int]: ",
		"8)MedianFilterRadius[int]: ");
}

void print_ACSOSM_properties_list()
{
	printf("%s\n%s\n%s\n%s\n%s\n",
		"9)Radius[int]: ",
		"10)GammaS[int]",
		"11)GammaC[int]",
		"12)SmoothnessWeak[int]",
		"13)SmoothnessStrong[int]");
}

void print_BBSM_properties_list()
{
	puts("9)Radius[int]: ");
}