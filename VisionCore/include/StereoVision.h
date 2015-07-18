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
#include <fstream>

#include "StereoCalibData.h"
#include "PointCloudStorage.h"
#include "StaticHelpers.h"

class StereoVision
{
public:
	
	//Конструкторы
	StereoVision();

	/* Создает объект и загружает настройки из указанных имен файлов*/
	StereoVision(const char* calibFileName);

	/* Создает объект используя заданные настройки*/
	StereoVision(StereoCalibData calibData, cv::Ptr<cv::StereoMatcher> stereoMatcher);

	/* Калибровка стерео-камеры (пары камер)
	* param[in] left - изображение с левой камеры (CV_8UC1 - серое)
	* param[in] right - изображение с правой камеры
	* param[in] patternSize - число углов шахматной доски
	(число квадратов на стороне - 1)
	* result - необходимый набор параметров для построения облака точек
	*/
	StereoCalibData StereoVision::Calibrate(const std::vector<cv::Mat>& left, const std::vector<cv::Mat>& right, cv::Size patternSize);

	/* Построение облака точек по двум изображениям
	* param[in] left - левое изображение с откалиброванной камеры
	* param[in] right - правое изображение с откалиброванной камеры
	* param[out] disparity - карта смещения для визуализации
	* param[in] disparityOnly - строить карту глубины без облака точек
	* result - облако точек
	*/
	PointCloudStorage CalculatePointCloud(const cv::Mat& left, const cv::Mat& right, cv::Mat& disparity, bool disparityOnly = false) const;

	/* Построение облака точек по двум изображениям
	* param[in] left - левое изображение с откалиброванной камеры
	* param[in] right - правое изображение с откалиброванной камеры
	* param[in] disparityOnly - строить карту глубины без облака точек
	* result - облако точек
	*/
	PointCloudStorage CalculatePointCloud(const cv::Mat& left, const cv::Mat& right, bool disparityOnly = false) const;

	//Возвращает параметры калибровки
	StereoCalibData GetCalibData();

	//Задат параметры калибровки
	void SetCalibData(StereoCalibData calibData);

	//Возвращает объект StereoMatcher
	cv::Ptr<cv::StereoMatcher> GetStereoMatcher();

	//Задает объект StereoBM
	void SetStereoMatcher(cv::Ptr<cv::StereoMatcher> stereoMatcher);

private:

	StereoCalibData calibData;						//Калибровочные данные
	cv::Ptr<cv::StereoMatcher> stereoMatcher;		//Объект для построения карты различий
	

	//Создает матрицы исправленяи искажений
	void _createUndistortRectifyMaps(StereoCalibData& data);

	//Устраняет переворот найденных точек
	void _fixChessboardCorners(std::vector<cv::Point2f>& corners, cv::Size patternSize);

	//Строит облако точек
	//Соответствующие публичные методы - обертка вокруг него, для красоты
	PointCloudStorage _calculatePointCloud(const cv::Mat& left, const cv::Mat& right, bool noDisparityOut, cv::Mat& disparity, bool disparityOnly = false) const;
};

#endif