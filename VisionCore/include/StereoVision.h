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

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vector>
#include <fstream>

#include "StereoCalibData.h"

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
	* result - успех калибровки
	*/
	bool Calibrate(const std::vector<cv::Mat>& left, const std::vector<cv::Mat>& right, cv::Size patternSize);

	//Выравнивает изображения
	void Rectify(cv::Mat& left, cv::Mat& right);

	/* Построение облака точек по двум изображениям
	* param[in] left - левые изображения с откалиброванной камеры
	* param[in] right - правые изображения с откалиброванной камеры
	* param[out] disparity - карта смещения для визуализации
	* param[in] disparityOnly - строить карту глубины без облака точек
	* result - облако точек
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr CalculatePointCloud(const std::vector<cv::Mat> & left, const std::vector <cv::Mat> & right, cv::Mat& disparity, bool disparityOnly = false) const;

	/* Построение облака точек по двум изображениям
	* param[in] left - левые изображения с откалиброванной камеры
	* param[in] right - правые изображения с откалиброванной камеры
	* param[in] disparityOnly - строить карту глубины без облака точек
	* result - облако точек
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr CalculatePointCloud(const std::vector<cv::Mat> & left, const std::vector <cv::Mat> & right) const;

	/* Построение облака точек по двум изображениям
	* param[in] left - левое изображение с откалиброванной камеры
	* param[in] right - правое изображение с откалиброванной камеры
	* param[out] disparity - карта смещения для визуализации
	* param[in] disparityOnly - строить карту глубины без облака точек
	* result - облако точек
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr CalculatePointCloud(const cv::Mat & left, const cv::Mat & right, cv::Mat& disparity, bool disparityOnly = false) const;

	/* Построение облака точек по двум изображениям
	* param[in] left - левое изображение с откалиброванной камеры
	* param[in] right - правое изображение с откалиброванной камеры
	* param[in] disparityOnly - строить карту глубины без облака точек
	* result - облако точек
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr CalculatePointCloud(const cv::Mat & left, cv::Mat & right) const;

	//Возвращает параметры калибровки
	StereoCalibData GetCalibData();

	//Задат параметры калибровки
	void SetCalibData(StereoCalibData calibData);

	//Возвращает объект StereoMatcher
	cv::Ptr<cv::StereoMatcher> GetStereoMatcher();

	//Задает объект StereoMathcer
	void SetStereoMatcher(cv::Ptr<cv::StereoMatcher> stereoMatcher);

	StereoVision& operator=(const StereoVision& ohter);

private:

	StereoCalibData calibData;						//Калибровочные данные
	cv::Ptr<cv::StereoMatcher> stereoMatcher;		//Объект для построения карты различий
	

	//Создает матрицы исправленяи искажений
	void _createUndistortRectifyMaps(StereoCalibData& data);
	
	//Усредняет карту различий
	cv::Mat average_disparity(std::vector<cv::Mat>& disparities) const;

	/* Строит облако точек по парам изображений
	 * param[in] left - вектор изображений с левой камеры
	 * param[in] right - вектор изображений с правой камеры
	 * param[in] noDisparityOut - флаг, указывающий, что не надо копировать карту различий в disparityResult
	 * param[out] disparityResult - результирующая карта различий
	 * param[in] - disparityOnly - генерировать только карту различий без облака точек
	 * result - укзаатель на облако точек
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr  _calculatePointCloud(const std::vector<cv::Mat> & left, const std::vector <cv::Mat> & right, bool noDisparityOut, cv::Mat& disparityResult, bool disparityOnly) const;

	//Преобразует облако точек из cv::Mat в pcl::PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr _matToPointCloud(const cv::Mat mat) const;
};

#endif