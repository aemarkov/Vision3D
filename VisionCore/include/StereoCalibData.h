#ifndef __STEREOCALIBDATA_H__
#define __STEREOCALIBDATA_H__

#include <opencv2/opencv.hpp>

/*///////////////////////////////////////////////////////////////////////////////////////
//                                StereoCalibData                                      //
//-------------------------------------------------------------------------------------//
// Этот класс инкапсулирует параметры, необходимые для построение облака точек по      //
// двум изображениям:                                                                  //
//  - Матрицы двух камер                                                               //
//  - Коэффициенты дисторсии двух камер                                                //
//  - Матрицу Q - матрицу перевода карты смещений в облако точек                       //
//  - А так же другие матрицы, получаемые в результате операций калибровки             //
// Так же этот класс имеет методы по загрузке и сохранению данных коэффициентов        //
// в хранилище                                                                         //
/*///////////////////////////////////////////////////////////////////////////////////////

class StereoCalibData
{
public:

	//Размер изображения
	cv::Size ImageSize;

	//Матрицы камер
	cv::Mat LeftCameraMatrix;
	cv::Mat RightCameraMatrix;

	//Параметры дисторсии
	cv::Mat LeftCameraDistortions;
	cv::Mat RightCameraDistortions;

 	//Параметры для initUndistortRectifyMap
	cv::Mat LeftCameraRectifiedProjection;
	cv::Mat RightCameraRectifiedProjection;
	cv::Mat LeftCameraRot;
	cv::Mat RightCameraRot;

	//Матриы исправления искажений (не сохраняются)
	cv::Mat LeftMapX, LeftMapY;
	cv::Mat RightMapX, RightMapY;

	//Матрица перехода от карты смещеняи к глубине
	cv::Mat Q;

	//Конструктор по-умолчанию
	StereoCalibData();

	//Создание объекта из файла
	StereoCalibData(const char* filename);

	//Конструктор копирования (создание ссылки)
	StereoCalibData(const StereoCalibData& other);

	//Оператор присвоения (создание ссылки)
	StereoCalibData& operator=(const StereoCalibData& other);

	//Создание полной копии
	StereoCalibData& Clone();

	//Сохраняет данные
	void Save(const char* filename);


};

#endif