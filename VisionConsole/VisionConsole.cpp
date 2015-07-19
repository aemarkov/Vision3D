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


	Ptr<StereoSGBM> sgbm;
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

	Ptr<StereoBM> sbm;
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

//Вывод карты различий в реальном времени
void disparityRealtime(VideoCapture cap1, VideoCapture cap2, StereoVision& sv);


void loadBMSettings(StereoBMParams& params, const char* filename);
void loadSGBMSettings(StereoSGBMParams& params, const char* filename);

//Показать подсказки
void displayHelp();

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, _TCHAR* argv[])
{
	if (argc < 5)
	{
		//Показать хелп
		displayHelp();
		return 0;
	}

	//Параметры, переданные пользователем
	int leftCameraIndex=-1, rightCameraIndex=-1;		//Индексы левой и правой веб-камер
	bool useStereoSGBM = false;							//Использовать StereoBM или StereoSGBM
	string calibFilename, stereoFilename;				//Имена файлов настроек
	bool isCalib = false, isStereoConfig = false;		//Используются ли файлы конфигурации

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
		else if (strcmp("-calib", argv[currentParam])==0)
		{
			//calibFilename = new char[strlen(argv[currentParam + 1])+1];
			//strcpy(calibFilename, argv[currentParam + 1]);
			calibFilename = string(argv[currentParam + 1]);
			currentParam += 2;
			isCalib = true;
		}
		else if (strcmp("-stereoparam", argv[currentParam]) == 0)
		{
			//stereoFilename = new char[strlen(argv[currentParam + 1]) + 1];
			//strcpy(stereoFilename, argv[currentParam + 1]);
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

	if ((leftCameraIndex == -1) || (rightCameraIndex == -1))
	{
		cout << "ERROR: camera not set\n";
		return 0;
	}

	StereoVision sv;
	VideoCapture cap1(leftCameraIndex), cap2(rightCameraIndex);
	Mat leftIm, rightIm;
	Mat leftRes, rightRes;

	//Загружаем конфиг калибровки
	//!!
	if (isCalib)
	{
		sv = StereoVision(calibFilename.c_str());
	}

	//Загружаем конфиг Stereo Matching'а
	if (isStereoConfig)
	{
		loadBMSettings(stereoBMParams, stereoFilename.c_str());
	}

	while (true)
	{
		//Калибровка
		if (!isCalib)
		{
			int w, h;
			cout << "Enter pattern size width, height: ";
			cin >> w >> h;

			calibrate(cap1, cap2, sv, Size(w, h));
			cout << "Enter calibration config filename *.yml or *.xml: ";
			cin >> calibFilename;
			sv.GetCalibData().Save(calibFilename.c_str());
		}

		//Захват изображений
		cap1 >> leftIm;
		cap2 >> rightIm;

		//Уменьшение и перевод в ч\б
		convertImage(leftIm, 0.5);
		convertImage(rightIm, 0.5);
		
		auto data = CallbackData(sv, leftIm, rightIm);

		if (!useStereoSGBM)
		{
			//StereoBM
			displayTrackbarsBM(data);
			callbackBM(0, &data);
		}
		else
		{
			//StereoSGBM
			displayTrackbarsSGBM(data);
			callbackSGBM(0, &data);
		}

		//Показ карты различий в вреальном времени
		disparityRealtime(cap1, cap2, sv);

		//Созранение облака точек
		cap1 >> leftIm;
		cap2 >> rightIm;
		resize(leftIm, leftRes, Size(0, 0), 0.5, 0.5);
		resize(rightIm, rightRes, Size(0, 0), 0.5, 0.5);

		auto cloud = sv.CalculatePointCloud(leftRes, rightRes);
		cloud.SaveToObj("cloud.obj");
		
		//Сохранение файла конфигурации
		if (!isStereoConfig)
		{
			char answer;
			cout << "Do you want to save config? (y\\n): ";
			cin >> answer;
			if (answer == 'y')
			{
				isStereoConfig = true;
				cout << "Enter filename *.yml or *.xml: ";
				cin >> stereoFilename;
			}
		}

		if (isStereoConfig)
		{
			if (useStereoSGBM)
				stereoSGBMParams.sgbm->save(stereoFilename);
			else
				stereoBMParams.sbm->save(stereoFilename);
		}
	}

	return 0;
}

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

///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////                        СОБЫТИЯ ПОЛЗУНКОВ                     ///////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void callbackSGBM(int wtf, void* data)
{
	CallbackData cData = *(CallbackData*)data;

	//Ограничения на параметры
	if (stereoSGBMParams.numDisparities % 16 != 0)
		stereoSGBMParams.numDisparities -= stereoSGBMParams.numDisparities % 16;
	if (stereoSGBMParams.numDisparities < 16)stereoSGBMParams.numDisparities = 16;

	if (stereoSGBMParams.preFilterCap % 2 == 0)stereoSGBMParams.preFilterCap++;

	//Настройка
	stereoSGBMParams.sgbm = StereoSGBM::create(stereoSGBMParams.minDisparity, stereoSGBMParams.numDisparities, stereoSGBMParams.SADWindowSize);
	stereoSGBMParams.sgbm->setPreFilterCap(stereoSGBMParams.preFilterCap);
	stereoSGBMParams.sgbm->setP1(stereoSGBMParams.p1);
	stereoSGBMParams.sgbm->setP2(stereoSGBMParams.p2);
	stereoSGBMParams.sgbm->setDisp12MaxDiff(stereoSGBMParams.disp12MaxDiff - 1);
	stereoSGBMParams.sgbm->setUniquenessRatio(stereoSGBMParams.uniquenessRatio);
	stereoSGBMParams.sgbm->setSpeckleWindowSize(stereoSGBMParams.speckleWindowSize);
	stereoSGBMParams.sgbm->setSpeckleRange(stereoSGBMParams.speckleRange);
	//sgbm->setMode(cv::StereoSGBM::MODE_SGBM); // : StereoSGBM::MODE_HH*/
	
	cData.sv.SetStereoMatcher(stereoSGBMParams.sgbm);
	//cData.sv.CalculatePointCloud(cData.left, cData.right);
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

	//Настройка
	stereoBMParams.sbm = cv::StereoBM::create(stereoBMParams.numDisparities, stereoBMParams.blockSize);
	stereoBMParams.sbm->setPreFilterSize(stereoBMParams.preFilterSize);				//Влиянеие не обнаружено
	stereoBMParams.sbm->setPreFilterCap(stereoBMParams.preFilterCap);				//Что - то нечетное, уменьшает шумы
	stereoBMParams.sbm->setMinDisparity(stereoBMParams.minDisparity);				//Меньше - разбитое фото, больше - темное (изменения сглаживаются)
	stereoBMParams.sbm->setTextureThreshold(stereoBMParams.textureThreshold);		//Порог текстуры (> = появляются черные пятна, < пятен нет, но детализация уменьшается)
	stereoBMParams.sbm->setUniquenessRatio(stereoBMParams.uniquenessRatio);			//Влияет на контуры (> четче)
	stereoBMParams.sbm->setSpeckleWindowSize(stereoBMParams.speckleWindowSize);		//Жесть какая - то
	stereoBMParams.sbm->setSpeckleRange(stereoBMParams.speckleRange);
	stereoBMParams.sbm->setDisp12MaxDiff(stereoBMParams.disp12MaxDiff);				//Сглаживает шумы

	cData.sv.SetStereoMatcher(stereoBMParams.sbm);
	//cData.sv.CalculatePointCloud(cData.left, cData.right, true);
}

//Отображение карты в режиме реального времени
void disparityRealtime(VideoCapture cap1, VideoCapture cap2, StereoVision& sv)
{
	Mat left, right, disparity;

	while (true)
	{
		cap1 >> left;
		cap2 >> right;

		convertImage(left, 0.5);
		convertImage(right, 0.5);

		sv.CalculatePointCloud(left, right, disparity, true);
		imshow("disparity", disparity);

		if (waitKey(1) != -1)break;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////                      ЗАГРУЗКА ПАРАМЕТРОВ StereoMatcher                     ///////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void loadBMSettings(StereoBMParams& params, const char* filename)
{
	auto bm = params.sbm;
	bm = StereoBM::load<StereoBM>(filename);
	params.blockSize = bm->getBlockSize();
	params.disp12MaxDiff = bm->getDisp12MaxDiff();
	params.minDisparity = bm->getMinDisparity();
	params.numDisparities = bm->getNumDisparities();
	params.preFilterCap = bm->getPreFilterCap();
	params.preFilterSize = bm->getPreFilterSize();
	params.speckleRange = bm->getSpeckleRange();
	params.speckleWindowSize = bm->getSpeckleWindowSize();
	params.textureThreshold = bm->getTextureThreshold();
	params.uniquenessRatio = bm->getUniquenessRatio();
}

void loadSGBMSettings(StereoSGBMParams& params, const char* filename)
{
	/*auto sgbm = params.sgbm;
	sgbm = Algorithm::load<StereoSGBM>(filename);
	params.disp12MaxDiff = sgbm->getDisp12MaxDiff();
	params.minDisparity = sgbm->getMinDisparity();
	params.numDisparities = sgbm->getNumDisparities();
	params.p1 = sgbm->getP1();
	params.p2 = sgbm->getP2();
	params.preFilterCap = sgbm->getPreFilterCap();
	params.SADWindowSize = sgbm->getBlockSize();	//??
	params.speckleRange = sgbm->getSpeckleRange();
	params.speckleWindowSize = sgbm->getSpeckleWindowSize();
	params.uniquenessRatio = sgbm->getUniquenessRatio();*/
}