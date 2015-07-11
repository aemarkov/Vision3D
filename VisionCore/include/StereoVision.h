#ifndef __STEREOVISION_H__
#define __STEREOVISION_H__

/*///////////////////////////////////////////////////////////////////////////////////////
//                                 StereoVision                                        //
//-------------------------------------------------------------------------------------//
// Этот класс предоставляет функционал по созданию облака точек по ДВУМ изображения    //
// (в.т.ч., полученным с ДВУХ камер).                                                  //
// Основные функции данного класса:                                                    //
//  - осуществление калибровки камеры\камер                                            //
//  - загрузка и сохранение результатов калибровки (с использованием класса            //
//    StereoCameraCalibData)                                                           //
//  - обработка пар снимков, сделанных при помощи откалиюрованных камер и построение   //
//    карты глубины и облака точек (за сохранение облака точек отвечает интерфейс      //
//    IPointCloudStorage)                                                              //
/*///////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

#include <vector>
#include <ostream>

#include "StereoCalibData.h"
#include "IPointCloudStorage.h"
#include "StaticHelpers.h"

class StereoVision
{
public:
	
	//Конструкторы
	StereoVision();
	StereoVision(const char *name);
	StereoVision(StereoCalibData calibData);

	/* Калибровка стерео-камеры (пары камер)
	* param[in] left - изображение с левой камеры (CV_8UC1 - серое)
	* param[in] right - изображение с правой камеры
	* param[in] patternSize - число углов шахматной доски
	(число квадратов на стороне - 1)
	* result - необходимый набор параметров для построения облака точек
	*/
	StereoCalibData StereoVision::Calibrate(const cv::Mat& left, const cv::Mat& right, cv::Size patternSize);

	/* Построение облака точек по двум изображениям
	* param[in] left - левое изображение с откалиброванной камеры
	* param[in] right - правое изображение с откалиброванной камеры
	* result - облако точек
	*/
	IPointCloudStorage* CalculatePointCloud(const cv::Mat& left, const cv::Mat& right) const;

	StereoCalibData GetCalibData();

private:
	StereoCalibData calibData;			//Калибровочные данные
	std::ostream* out;					//Поток отладочного вывода

	//Создает матрицы исправленяи искажений
	void _createUndistortRectifyMaps(StereoCalibData& data);
};

#endif